//! hrtf-render: Steam Audio's default HRTF as an HRIR table (`VRCHRTF1`,
//! `vrc_audio::table`) for the bridge's direction finder.
//!
//! VRChat spatializes voices with Steam Audio's built-in HRTF. Its data is
//! not public (only linked into the shipped `phonon` library), but the API
//! renders it: a context at the game's rate, `iplHRTFCreate` with
//! `IPL_HRTFTYPE_DEFAULT`, then per direction a unit impulse through
//! `iplBinauralEffectApply` gives that direction's pair of HRIRs (and the
//! API's own peak delays).
//!
//! The library is the official SDK's (github.com/ValveSoftware/steam-audio
//! releases, Apache-2.0): only its prebuilt `phonon.dll` / `libphonon.so`
//! is loaded, at run time, nothing else of the download is used.
//!
//! ```text
//! hrtf-render --phonon <phonon.dll | libphonon.so> --out steam-default-48k.bin
//!             [--interp bilinear|nearest] [--az-step 2] [--el-min -30] [--el-max 60]
//!             [--el-step 10] [--rate 48000] [--frame 1024]
//! ```
//!
//! It prints a check afterwards: per azimuth on the horizon, the ITD from
//! the peak delays and from the HRIRs' cross-correlation, and the level
//! difference at 1 and 4 kHz.

use std::ffi::{c_char, c_void};
use std::path::PathBuf;
use std::ptr::null_mut;

use anyhow::{bail, ensure, Context, Result};
use libloading::{Library, Symbol};
use vrc_audio::hrtf::Hrtf;
use vrc_audio::HrirTable;

// -- The parts of phonon.h used (Steam Audio 4.x) --------------------------

type Handle = *mut c_void;

#[repr(C)]
struct ContextSettings {
    version: u32,
    log: *const c_void,
    allocate: *const c_void,
    free: *const c_void,
    simd_level: i32,
    flags: i32,
}

#[repr(C)]
struct AudioSettings {
    sampling_rate: i32,
    frame_size: i32,
}

#[repr(C)]
struct HrtfSettings {
    kind: i32,
    sofa_file_name: *const c_char,
    sofa_data: *const u8,
    sofa_data_size: i32,
    volume: f32,
    norm_type: i32,
}

#[repr(C)]
struct BinauralEffectSettings {
    hrtf: Handle,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct Vector3 {
    x: f32,
    y: f32,
    z: f32,
}

#[repr(C)]
struct BinauralEffectParams {
    direction: Vector3,
    interpolation: i32,
    spatial_blend: f32,
    hrtf: Handle,
    peak_delays: *mut f32,
}

#[repr(C)]
struct AudioBuffer {
    num_channels: i32,
    num_samples: i32,
    data: *mut *mut f32,
}

const HRTFTYPE_DEFAULT: i32 = 0;
const HRTFNORM_NONE: i32 = 0;
const SIMD_AVX2: i32 = 3;
const INTERP_NEAREST: i32 = 0;
const INTERP_BILINEAR: i32 = 1;

type ContextCreate = unsafe extern "system" fn(*mut ContextSettings, *mut Handle) -> i32;
type HrtfCreate = unsafe extern "system" fn(Handle, *mut AudioSettings, *mut HrtfSettings, *mut Handle) -> i32;
type EffectCreate = unsafe extern "system" fn(Handle, *mut AudioSettings, *mut BinauralEffectSettings, *mut Handle) -> i32;
type EffectReset = unsafe extern "system" fn(Handle);
type EffectApply = unsafe extern "system" fn(Handle, *mut BinauralEffectParams, *mut AudioBuffer, *mut AudioBuffer) -> i32;
type Release = unsafe extern "system" fn(*mut Handle);

struct Options {
    phonon: PathBuf,
    out: PathBuf,
    interpolation: i32,
    az_step: f32,
    el: (f32, f32, f32),
    rate: u32,
    frame: usize,
}

fn options() -> Result<Options> {
    let mut o = Options {
        phonon: PathBuf::new(),
        out: PathBuf::new(),
        interpolation: INTERP_BILINEAR,
        az_step: 2.0,
        el: (-30.0, 60.0, 10.0),
        rate: 48_000,
        frame: 1024,
    };
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut value = || args.next().with_context(|| format!("{a} needs a value"));
        match a.as_str() {
            "--phonon" => o.phonon = value()?.into(),
            "--out" => o.out = value()?.into(),
            "--interp" => {
                o.interpolation = match value()?.as_str() {
                    "bilinear" => INTERP_BILINEAR,
                    "nearest" => INTERP_NEAREST,
                    v => bail!("--interp is bilinear or nearest, not {v}"),
                }
            }
            "--az-step" => o.az_step = value()?.parse()?,
            "--el-min" => o.el.0 = value()?.parse()?,
            "--el-max" => o.el.1 = value()?.parse()?,
            "--el-step" => o.el.2 = value()?.parse()?,
            "--rate" => o.rate = value()?.parse()?,
            "--frame" => o.frame = value()?.parse()?,
            _ => bail!("unknown argument {a} (see the source's header for usage)"),
        }
    }
    ensure!(!o.phonon.as_os_str().is_empty() && !o.out.as_os_str().is_empty(), "--phonon and --out are required");
    ensure!(o.az_step > 0.0 && (360.0 / o.az_step).fract().abs() < 1e-4, "--az-step must divide 360");
    ensure!(o.el.2 > 0.0 && o.el.1 >= o.el.0, "bad elevations");
    Ok(o)
}

fn main() -> Result<()> {
    let o = options()?;
    let table = render(&o)?;
    if let Some(dir) = o.out.parent().filter(|d| !d.as_os_str().is_empty()) {
        std::fs::create_dir_all(dir)?;
    }
    table.write(&o.out)?;
    println!("wrote {} ({} directions, {} taps, {} bytes)", o.out.display(), table.directions(), table.taps, std::fs::metadata(&o.out)?.len());
    check(&HrirTable::read(&o.out)?);
    Ok(())
}

/// Every direction's HRIRs, trimmed to what all of them need.
fn render(o: &Options) -> Result<HrirTable> {
    // SAFETY: the official Steam Audio library; its functions are called
    // with the signatures and structs of its phonon.h (4.x).
    unsafe {
        let lib = Library::new(&o.phonon).with_context(|| format!("loading {}", o.phonon.display()))?;
        let context_create: Symbol<ContextCreate> = lib.get(b"iplContextCreate\0")?;
        let hrtf_create: Symbol<HrtfCreate> = lib.get(b"iplHRTFCreate\0")?;
        let effect_create: Symbol<EffectCreate> = lib.get(b"iplBinauralEffectCreate\0")?;
        let effect_reset: Symbol<EffectReset> = lib.get(b"iplBinauralEffectReset\0")?;
        let effect_apply: Symbol<EffectApply> = lib.get(b"iplBinauralEffectApply\0")?;
        let effect_release: Symbol<Release> = lib.get(b"iplBinauralEffectRelease\0")?;
        let hrtf_release: Symbol<Release> = lib.get(b"iplHRTFRelease\0")?;
        let context_release: Symbol<Release> = lib.get(b"iplContextRelease\0")?;

        // The API version this was written against; the library checks it
        // is compatible.
        let version = (4u32 << 16) | (8 << 8) | 1;
        let mut cs = ContextSettings { version, log: std::ptr::null(), allocate: std::ptr::null(), free: std::ptr::null(), simd_level: SIMD_AVX2, flags: 0 };
        let mut ctx: Handle = null_mut();
        ensure!(context_create(&mut cs, &mut ctx) == 0 && !ctx.is_null(), "iplContextCreate failed");
        let mut audio = AudioSettings { sampling_rate: o.rate as i32, frame_size: o.frame as i32 };
        let mut hs = HrtfSettings {
            kind: HRTFTYPE_DEFAULT,
            sofa_file_name: std::ptr::null(),
            sofa_data: std::ptr::null(),
            sofa_data_size: 0,
            volume: 1.0,
            norm_type: HRTFNORM_NONE,
        };
        let mut hrtf: Handle = null_mut();
        ensure!(hrtf_create(ctx, &mut audio, &mut hs, &mut hrtf) == 0 && !hrtf.is_null(), "iplHRTFCreate (default) failed at {} Hz", o.rate);
        let mut es = BinauralEffectSettings { hrtf };
        let mut effect: Handle = null_mut();
        ensure!(effect_create(ctx, &mut audio, &mut es, &mut effect) == 0 && !effect.is_null(), "iplBinauralEffectCreate failed");

        let f = o.frame;
        let (mut input, mut left, mut right) = (vec![0.0f32; f], vec![0.0f32; f], vec![0.0f32; f]);
        let mut in_ptrs = [input.as_mut_ptr()];
        let mut out_ptrs = [left.as_mut_ptr(), right.as_mut_ptr()];
        let mut inb = AudioBuffer { num_channels: 1, num_samples: f as i32, data: in_ptrs.as_mut_ptr() };
        let mut outb = AudioBuffer { num_channels: 2, num_samples: f as i32, data: out_ptrs.as_mut_ptr() };

        let n_el = ((o.el.1 - o.el.0) / o.el.2).round() as usize + 1;
        let n_az = (360.0 / o.az_step).round() as usize;
        let elevations: Vec<f32> = (0..n_el).map(|i| o.el.0 + i as f32 * o.el.2).collect();
        let azimuths: Vec<f32> = (0..n_az).map(|i| -180.0 + i as f32 * o.az_step).collect();
        let mut raw: Vec<(Vec<f32>, Vec<f32>)> = Vec::new();
        let mut delays = Vec::new();
        for &el in &elevations {
            for &az in &azimuths {
                let d = vrc_audio::direction(az, el);
                let mut peak = [0.0f32; 2];
                let mut params = BinauralEffectParams {
                    direction: Vector3 { x: d[0], y: d[1], z: d[2] },
                    interpolation: o.interpolation,
                    spatial_blend: 1.0,
                    hrtf,
                    peak_delays: peak.as_mut_ptr(),
                };
                effect_reset(effect);
                // Silence first at this direction (nothing left to fade in
                // from), then the impulse, then its tail.
                let (mut l, mut r) = (Vec::new(), Vec::new());
                for k in 0..5 {
                    input.fill(0.0);
                    if k == 2 {
                        input[0] = 1.0;
                    }
                    effect_apply(effect, &mut params, &mut inb, &mut outb);
                    if k >= 2 {
                        l.extend_from_slice(&left);
                        r.extend_from_slice(&right);
                    } else {
                        ensure!(left.iter().chain(right.iter()).all(|x| x.abs() < 1e-9), "output before the impulse at az {az} el {el}");
                    }
                }
                raw.push((l, r));
                delays.push(peak);
            }
        }
        effect_release(&mut effect);
        hrtf_release(&mut hrtf);
        context_release(&mut ctx);
        drop(lib);

        // One window for all: from the earliest onset to the latest end.
        let loudest = raw.iter().flat_map(|(l, r)| l.iter().chain(r)).fold(0f32, |m, x| m.max(x.abs()));
        let floor = loudest * 1e-4;
        let onset = |x: &[f32]| x.iter().position(|v| v.abs() > floor).unwrap_or(0);
        let first = raw.iter().map(|(l, r)| onset(l).min(onset(r))).min().unwrap_or(0);
        let last = raw
            .iter()
            .map(|(l, r)| l.iter().rposition(|x| x.abs() > floor).unwrap_or(0).max(r.iter().rposition(|x| x.abs() > floor).unwrap_or(0)))
            .max()
            .unwrap_or(0);
        let taps = (last + 1 - first).div_ceil(16) * 16;
        let mut hrirs = Vec::with_capacity(raw.len() * 2 * taps);
        for (l, r) in &raw {
            for ear in [l, r] {
                let mut h: Vec<f32> = ear[first..].iter().take(taps).copied().collect();
                h.resize(taps, 0.0);
                hrirs.extend(h);
            }
        }
        // The API's peak delays are seconds (measured: about 0.65 ms apart
        // at the side), not samples, and so untouched by the trim.
        let peak_delays = delays;
        println!("HRIRs: onset at {first} samples, {taps} taps kept (of {}), peak {loudest:.4}", 3 * f);
        Ok(HrirTable { rate: o.rate, taps, elevations, azimuths, interpolation: o.interpolation as u32, version, peak_delays, hrirs })
    }
}

/// Prints the horizon: ITD from the peak delays and from the HRIRs, and the
/// level difference at 1 and 4 kHz (left over right, dB).
fn check(t: &HrirTable) {
    println!("{}", t.name());
    println!("{:>6} {:>9} {:>9} {:>8} {:>8}", "az", "itd_api", "itd_xcor", "ild_1k", "ild_4k");
    let el = t.elevations.iter().copied().min_by(|a, b| a.abs().total_cmp(&b.abs())).unwrap_or(0.0);
    for az in (-180..180).step_by(15) {
        let az = az as f32;
        let i = t.nearest(az, el);
        let (l, r) = t.hrir(i);
        let xcor = (-60i32..=60).max_by(|&a, &b| corr(l, r, a).total_cmp(&corr(l, r, b))).unwrap_or(0);
        let resp = t.response(az, el, &[1000.0, 4000.0]);
        let ild = |k: usize| 10.0 * (resp[k].0.norm_sqr() / resp[k].1.norm_sqr().max(1e-20)).log10();
        println!("{az:>6.0} {:>8.0}u {:>8.0}u {:>8.1} {:>8.1}", t.itd_s(az, el) * 1e6, xcor as f32 / t.rate as f32 * 1e6, ild(0), ild(1));
    }
}

/// The cross-correlation of `l` delayed by `lag` with `r`.
fn corr(l: &[f32], r: &[f32], lag: i32) -> f32 {
    (0..l.len() as i32).filter_map(|n| Some(l.get(n as usize)? * r.get((n - lag) as usize)?)).sum()
}
