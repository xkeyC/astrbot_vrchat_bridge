//! vr-probe: pokes the bot's virtual headset by hand.
//!
//!     vr-probe info                       # the latest tapped frame, as JSON-ish text
//!     vr-probe grab eyes.png              # both eyes side by side
//!     vr-probe look --yaw 60 --pitch -20  # turn the head (held while connected)
//!     vr-probe sweep out/ --yaws=-60,0,60 # turn, settle, grab, for each yaw
//!     vr-probe depth out/scene            # stereo depth: PNG, PLY, floor fit
//!     vr-probe scan out/room --down       # head scan: panorama + height map

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::thread::sleep;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use vrc_nav::{GotoOptions, Rig, Survey, SurveyOptions};
use vrc_scene::Kind;
use vrc_stereo::{fit_floor, SgmParams, Stereo};
use vrc_vr::osc::Osc;
use vrc_vr::remote::RemoteHmd;
use vrc_vr::scan;
use vrc_vr::tap::{EyeFrame, EyeTap};
use vrc_vr::walk;
use vrc_vr::Pose;

#[derive(Parser)]
struct Cli {
    /// Monado's remote driver.
    #[arg(long, default_value = "127.0.0.1:4242", global = true)]
    remote: String,
    /// The null compositor's eye tap.
    #[arg(long, default_value = "/dev/shm/vrc-eyes", global = true)]
    tap: PathBuf,
    /// local-multimodal-infra's OCR endpoint ("" for no OCR).
    #[arg(long, default_value = "http://10.88.0.1:17890/v1/ocr/lines", global = true)]
    ocr_url: String,
    #[arg(long, default_value = "ppocrv5-mobile-onnx", global = true)]
    ocr_model: String,
    /// Whitelisted display names, in priority order (comma-separated).
    #[arg(long, value_delimiter = ',', global = true)]
    whitelist: Vec<String>,
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Describe the latest tapped frame.
    Info,
    /// Save the latest frame, both eyes side by side, as PNG.
    Grab { out: PathBuf },
    /// Turn the head and hold it for a while.
    Look {
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        yaw: f32,
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        pitch: f32,
        /// Seconds to hold the connection (the driver keeps the last pose anyway).
        #[arg(long, default_value_t = 1.0)]
        hold: f32,
    },
    /// For each yaw: turn, wait for a frame rendered with that pose, save it.
    Sweep {
        dir: PathBuf,
        #[arg(long, value_delimiter = ',', allow_hyphen_values = true, default_value = "-60,0,60")]
        yaws: Vec<f32>,
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        pitch: f32,
    },
    /// Walk one measured leg: face --yaw (head) and walk --metres (world).
    Walk {
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        yaw: f32,
        #[arg(long, default_value_t = 1.0)]
        metres: f32,
        #[arg(long, default_value_t = 0.6)]
        axis: f32,
    },
    /// Put the hands somewhere: rest (down at the sides), up-behind (raised
    /// behind the head as seen along --yaw), and hold.
    Hands {
        pose: String,
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        yaw: f32,
        #[arg(long, default_value_t = 1.0)]
        hold: f32,
    },
    /// Stereo depth of the latest frame: `<prefix>_depth.png` (left eye |
    /// depth), `<prefix>.ply` (points in the tracking space), and a floor fit.
    Depth {
        prefix: PathBuf,
        /// Match at 1/scale of the eye size.
        #[arg(long, default_value_t = 2)]
        scale: usize,
        #[arg(long, default_value_t = 64)]
        max_disparity: usize,
    },
    /// Look all around by turning the head: `<prefix>_pano.png` (left eyes,
    /// equirectangular) and `<prefix>_map.png` (height map from stereo).
    Scan {
        prefix: PathBuf,
        /// Views in the ring.
        #[arg(long, default_value_t = 5)]
        count: usize,
        #[arg(long, default_value_t = -10.0, allow_hyphen_values = true)]
        pitch: f32,
        /// Also look down at the feet.
        #[arg(long)]
        down: bool,
    },
    /// Survey, then walk to a candidate (or a point by bearing and distance):
    /// `<prefix>_before_*` and `<prefix>_after_*` pictures.
    Goto {
        prefix: PathBuf,
        #[arg(long)]
        candidate: Option<usize>,
        #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
        bearing: f32,
        #[arg(long, default_value_t = 2.0)]
        distance: f32,
    },
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let mut tap = EyeTap::open(&cli.tap);
    match cli.command {
        Command::Info => println!("{}", describe(&latest(&mut tap)?)),
        Command::Grab { out } => {
            let frame = latest(&mut tap)?;
            save_png(&frame, &out)?;
            println!("{}\n-> {}", describe(&frame), out.display());
        }
        Command::Look { yaw, pitch, hold } => {
            let mut hmd = RemoteHmd::connect(&cli.remote)?;
            let head = Pose::looking(yaw, pitch, hmd.state.head.position);
            hmd.set_head(head)?;
            sleep(Duration::from_secs_f32(hold));
        }
        Command::Sweep { dir, yaws, pitch } => {
            std::fs::create_dir_all(&dir)?;
            let mut hmd = RemoteHmd::connect(&cli.remote)?;
            for yaw in yaws {
                hmd.set_head(Pose::looking(yaw, pitch, hmd.state.head.position))?;
                let frame = scan::rendered_at(&mut tap, yaw, pitch, Duration::from_secs(3))?;
                let out = dir.join(format!("yaw{yaw:+.0}_pitch{pitch:+.0}.png"));
                save_png(&frame, &out)?;
                println!("{}\n-> {}", describe(&frame), out.display());
            }
        }
        Command::Walk { yaw, metres, axis } => {
            let mut hmd = RemoteHmd::connect(&cli.remote)?;
            let head = hmd.state.head.position;
            hmd.state.hands_at_rest(head);
            let osc = Osc::connect()?;
            let leg = walk::leg(&mut hmd, &osc, yaw, metres, &walk::WalkParams { axis, ..Default::default() })?;
            for (t, vx, vz) in &leg.samples {
                println!("  {t:5.2} s  vx {vx:+.2}  vz {vz:+.2}");
            }
            println!("walked {:.2} m of {metres} in {:.2} s{}", leg.walked, leg.took.as_secs_f32(), if leg.blocked { ", BLOCKED" } else { "" });
        }
        Command::Hands { ref pose, yaw, hold } => {
            let mut hmd = RemoteHmd::connect(&cli.remote)?;
            let head = hmd.state.head.position;
            match pose.as_str() {
                "rest" => hmd.state.hands_at_rest(head),
                "up-behind" => hmd.state.hands_up_behind(head, yaw),
                other => anyhow::bail!("no hand pose {other}"),
            }
            hmd.send()?;
            sleep(Duration::from_secs_f32(hold));
        }
        Command::Depth { prefix, scale, max_disparity } => {
            depth(&latest(&mut tap)?, &prefix, scale, max_disparity)?
        }
        Command::Scan { ref prefix, count, pitch, down } => {
            scan_around(&cli, prefix, &SurveyOptions { count, pitch, down, ..Default::default() })?
        }
        Command::Goto { ref prefix, candidate, bearing, distance } => goto_cmd(&cli, prefix, candidate, bearing, distance)?,
    }
    Ok(())
}

fn depth(frame: &EyeFrame, prefix: &Path, scale: usize, max_disparity: usize) -> Result<()> {
    if let Some(dir) = prefix.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let stereo = Stereo::from_frame(frame, scale).context("not an 8-bit RGBA/BGRA frame")?;
    let started = Instant::now();
    let disp = stereo.disparity(&SgmParams { max_disparity, ..Default::default() });
    let took = started.elapsed();
    let depth = stereo.depth(&disp);
    let points = stereo.points(&disp, 1);
    let fx = stereo.intrinsics[0];
    println!("{}", describe(frame));
    println!(
        "matched {}x{} (1/{scale}), {max_disparity} disparities (nearest {:.2} m) in {:.0} ms; \
         {:.0}% of pixels have depth",
        disp.width,
        disp.height,
        fx * stereo.baseline / max_disparity as f32,
        took.as_secs_f64() * 1e3,
        disp.density() * 100.0
    );
    let (w, h) = (disp.width, disp.height);
    for (name, rx, ry) in [("centre", 0.5, 0.5), ("lower centre", 0.5, 0.75), ("left", 0.25, 0.5), ("right", 0.75, 0.5)]
    {
        let (cx, cy) = ((rx * w as f32) as usize, (ry * h as f32) as usize);
        let mut v: Vec<f32> = (cy - 4..cy + 4)
            .flat_map(|y| (cx - 4..cx + 4).map(move |x| (x, y)))
            .map(|(x, y)| depth[y * w + x])
            .filter(|d| d.is_finite())
            .collect();
        v.sort_by(f32::total_cmp);
        match v.get(v.len() / 2) {
            Some(d) => println!("  {name}: {d:.2} m ({} of 64 pixels)", v.len()),
            None => println!("  {name}: no depth"),
        }
    }
    let eye = stereo.left_pose.position;
    let xyz: Vec<[f32; 3]> = points.iter().map(|(p, _)| *p).collect();
    match fit_floor(&xyz, eye, 0.5) {
        Some(f) => println!(
            "  floor: {:.3} m below the eye (floor y {:.3}), tilt {:.2} deg, {} points",
            eye[1] - f.height,
            f.height,
            f.tilt_deg,
            f.inliers
        ),
        None => println!("  floor: not found"),
    }

    // Left eye | depth, as PNG.
    let mut rgb = Vec::with_capacity(w * h * 6);
    for y in 0..h {
        for x in 0..w {
            let g = stereo.left.data[y * w + x];
            rgb.extend_from_slice(&[g, g, g]);
        }
        for x in 0..w {
            rgb.extend_from_slice(&depth_colour(depth[y * w + x]));
        }
    }
    let stem = prefix.file_name().map(|s| s.to_string_lossy().into_owned()).unwrap_or_else(|| "depth".into());
    let png_path = prefix.with_file_name(format!("{stem}_depth.png"));
    let mut enc = png::Encoder::new(BufWriter::new(File::create(&png_path)?), (2 * w) as u32, h as u32);
    enc.set_color(png::ColorType::Rgb);
    enc.set_depth(png::BitDepth::Eight);
    enc.write_header()?.write_image_data(&rgb)?;

    // Points with their left-eye grey, as ASCII PLY.
    let ply_path = prefix.with_file_name(format!("{stem}.ply"));
    let mut out = BufWriter::new(File::create(&ply_path)?);
    writeln!(out, "ply\nformat ascii 1.0\nelement vertex {}", points.len())?;
    writeln!(out, "property float x\nproperty float y\nproperty float z")?;
    writeln!(out, "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header")?;
    for (p, (x, y)) in &points {
        let g = stereo.left.data[*y as usize * w + *x as usize];
        writeln!(out, "{:.3} {:.3} {:.3} {g} {g} {g}", p[0], p[1], p[2])?;
    }
    println!("-> {} , {}", png_path.display(), ply_path.display());
    Ok(())
}

/// Near red through yellow, green and cyan to far blue (0.3 to 12 m, log
/// scale); black without depth.
fn depth_colour(d: f32) -> [u8; 3] {
    if !d.is_finite() {
        return [0, 0, 0];
    }
    let t = ((d.max(0.3).ln() - 0.3f32.ln()) / (12f32.ln() - 0.3f32.ln())).clamp(0.0, 1.0);
    let hue = t * 240.0;
    let x = 1.0 - ((hue / 60.0) % 2.0 - 1.0).abs();
    let (r, g, b) = match (hue / 60.0) as u32 {
        0 => (1.0, x, 0.0),
        1 => (x, 1.0, 0.0),
        2 => (0.0, 1.0, x),
        _ => (0.0, x, 1.0),
    };
    [(r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8]
}

fn latest(tap: &mut EyeTap) -> Result<EyeFrame> {
    tap.read()?.context("Monado has not tapped a frame yet")
}

/// A survey: scan all around, stereo, height map, players, candidates;
/// writes the panorama and the map (numbered) and the candidates.
fn scan_around(cli: &Cli, prefix: &Path, opts: &SurveyOptions) -> Result<()> {
    if let Some(dir) = prefix.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let mut rig = rig(cli)?;
    let s = vrc_nav::survey(&mut rig, opts, &[])?;
    report_survey(&s);
    save_survey(&s, prefix)?;
    Ok(())
}

fn rig(cli: &Cli) -> Result<Rig> {
    Rig::connect(
        &cli.remote,
        &cli.tap.to_string_lossy(),
        Some(cli.ocr_url.as_str()),
        &cli.ocr_model,
        cli.whitelist.clone(),
    )
}

fn report_survey(s: &Survey) {
    let t = &s.timings;
    println!(
        "survey: scan {:.0} ms, stereo {:.0} ms, OCR {:.0} ms, map {:.0} ms; floor {:.3} below the eye (tilt {:.2} deg); x {:.3} = metres",
        t.scan.as_secs_f64() * 1e3,
        t.stereo.as_secs_f64() * 1e3,
        t.ocr.as_secs_f64() * 1e3,
        t.map.as_secs_f64() * 1e3,
        s.eye[1] - s.floor.height,
        s.floor.tilt_deg,
        s.metres
    );
    if !s.room.is_empty() {
        println!("room: {}", s.room.join(", "));
    }
    for p in &s.players {
        println!(
            "  player {} (OCR {:?}, {:.2}){}",
            p.name,
            p.text,
            p.score,
            p.whitelist_rank.map(|r| format!(" whitelist #{r}")).unwrap_or_default()
        );
    }
    let m = s.metres;
    for c in &s.candidates {
        println!(
            "  {:>2} {:<8} {:>5.1} m at {:+5.0} deg (walk {:.1} m){}",
            c.id,
            c.kind.name(),
            c.distance * m,
            c.bearing,
            c.path * m,
            match (&c.name, c.kind) {
                (Some(n), _) => format!(" {n}{}", c.whitelist_rank.map(|r| format!(" (whitelist #{r})")).unwrap_or_default()),
                (None, Kind::Platform) => format!(", top {:+.2} m", (c.position[1] - s.floor.height) * m),
                _ => String::new(),
            }
        );
    }
}

fn save_survey(s: &Survey, prefix: &Path) -> Result<()> {
    let stem = prefix.file_name().map(|s| s.to_string_lossy().into_owned()).unwrap_or_else(|| "scan".into());
    let pano = s.marked_panorama(2048);
    write_png(&prefix.with_file_name(format!("{stem}_pano_marked.png")), pano.width, pano.height, &pano.rgb)?;
    let (side, map) = s.marked_map(4);
    write_png(&prefix.with_file_name(format!("{stem}_map_marked.png")), side, side, &map)?;
    std::fs::write(
        prefix.with_file_name(format!("{stem}_candidates.json")),
        serde_json::to_string_pretty(&s.candidates_json())?,
    )?;
    println!("-> {p}_pano_marked.png, {p}_map_marked.png, {p}_candidates.json", p = prefix.display());
    Ok(())
}

/// Survey, pick a candidate (or a point by bearing and distance), walk there.
fn goto_cmd(cli: &Cli, prefix: &Path, candidate: Option<usize>, bearing: f32, distance: f32) -> Result<()> {
    if let Some(dir) = prefix.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let stem = prefix.file_name().map(|s| s.to_string_lossy().into_owned()).unwrap_or_else(|| "goto".into());
    let mut rig = rig(cli)?;
    let s = vrc_nav::survey(&mut rig, &SurveyOptions::default(), &[])?;
    report_survey(&s);
    save_survey(&s, &prefix.with_file_name(format!("{stem}_before")))?;
    let target = match candidate {
        Some(id) => {
            let c = s.candidates.iter().find(|c| c.id == id).context("no such candidate")?;
            println!("going to {} ({}), {:.1} m at {:+.0} deg", c.id, c.kind.name(), c.distance * s.metres, c.bearing);
            [c.position[0], c.position[2]]
        }
        None => {
            let yaw = (s.yaw + bearing).to_radians();
            let d = distance / s.metres;
            println!("going {distance:.1} m at {bearing:+.0} deg");
            [s.eye[0] + yaw.sin() * d, s.eye[2] - yaw.cos() * d]
        }
    };
    let report = vrc_nav::goto(&mut rig, s, target, &GotoOptions::default())?;
    for (i, l) in report.legs.iter().enumerate() {
        println!(
            "  leg {}: {:.1} m left, heading {:+.0} deg, walked {:.2} of {:.2} m{}",
            i + 1,
            l.before,
            l.yaw,
            l.walked,
            l.planned,
            if l.blocked { ", BLOCKED" } else { "" }
        );
    }
    println!(
        "{} in {:.1} s, {:.2} m left{}",
        if report.arrived { "ARRIVED" } else { "STOPPED" },
        report.took.as_secs_f32(),
        report.remaining,
        report.reason.as_ref().map(|r| format!(" ({r})")).unwrap_or_default()
    );
    let after = vrc_nav::survey(&mut rig, &SurveyOptions::default(), &[])?;
    save_survey(&after, &prefix.with_file_name(format!("{stem}_after")))?;
    Ok(())
}


fn write_png(path: &Path, w: usize, h: usize, rgb: &[u8]) -> Result<()> {
    let mut enc = png::Encoder::new(BufWriter::new(File::create(path)?), w as u32, h as u32);
    enc.set_color(png::ColorType::Rgb);
    enc.set_depth(png::BitDepth::Eight);
    enc.write_header()?.write_image_data(rgb)?;
    Ok(())
}

fn describe(f: &EyeFrame) -> String {
    let [fx, fy, cx, cy] = f.views[0].fov.intrinsics(f.width, f.height);
    let (yaw, pitch) = f.views[0].pose.yaw_pitch();
    format!(
        "frame {} seq {}: {}x{} per eye, VkFormat {}, baseline {:.4} m, fx {fx:.1} fy {fy:.1} cx {cx:.1} cy {cy:.1}, \
         looking yaw {yaw:+.1} pitch {pitch:+.1}, left eye at {:?}",
        f.frame_id,
        f.seq,
        f.width,
        f.height,
        f.format,
        f.baseline(),
        f.views[0].pose.position,
    )
}

fn save_png(frame: &EyeFrame, out: &Path) -> Result<()> {
    let (w, h) = (frame.width as usize, frame.height as usize);
    let eyes = [frame.eye_rgb8(0)?, frame.eye_rgb8(1)?];
    let mut rgb = Vec::with_capacity(w * h * 6);
    for y in 0..h {
        for eye in &eyes {
            rgb.extend_from_slice(&eye[y * w * 3..(y + 1) * w * 3]);
        }
    }
    let mut enc = png::Encoder::new(BufWriter::new(File::create(out)?), (2 * w) as u32, h as u32);
    enc.set_color(png::ColorType::Rgb);
    enc.set_depth(png::BitDepth::Eight);
    enc.write_header()?.write_image_data(&rgb)?;
    Ok(())
}
