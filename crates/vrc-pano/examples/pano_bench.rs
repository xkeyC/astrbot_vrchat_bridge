//! Times decoding a whole tapped frame (both eyes, RGBA, 1920²):
//! `cargo run --release -p vrc-pano --example pano_bench -- <frame.rgba> [runs]`
//! (`RAYON_NUM_THREADS` to try fewer cores).

use std::time::Instant;

use vrc_pano::{decode, PanoParams};
use vrc_vr::tap::{format, EyeFrame};

fn main() -> anyhow::Result<()> {
    let path = std::env::args().nth(1).ok_or_else(|| anyhow::anyhow!("a frame's .rgba"))?;
    let runs: usize = std::env::args().nth(2).and_then(|n| n.parse().ok()).unwrap_or(20);
    let pixels = std::fs::read(&path)?;
    let side = ((pixels.len() / 8) as f64).sqrt() as u32;
    let frame = EyeFrame {
        seq: 2,
        frame_id: 1,
        display_time_ns: 0,
        capture_ns: 0,
        width: side,
        height: side,
        format: format::R8G8B8A8_SRGB,
        bytes_per_pixel: 4,
        views: Default::default(),
        pixels,
    };
    let params = PanoParams::default();
    decode(&frame, &params)?;
    let mut times = Vec::new();
    for _ in 0..runs {
        let t = Instant::now();
        let p = decode(&frame, &params)?;
        times.push(t.elapsed().as_secs_f64() * 1e3);
        std::hint::black_box(p);
    }
    times.sort_by(f64::total_cmp);
    let t = Instant::now();
    let classified = vrc_pano::classify(&frame);
    let classify_ms = t.elapsed().as_secs_f64() * 1e3;
    // The parts: the colour alone (no depth route), the calibration's fit.
    let vrc_pano::Seen::Pano { mut code, head } = classified else { anyhow::bail!("not a pano frame") };
    code.route = vrc_pano::Route::None;
    let t = Instant::now();
    for _ in 0..runs {
        std::hint::black_box(vrc_pano::decode_as(&frame, code, head, &params)?);
    }
    let colour_ms = t.elapsed().as_secs_f64() * 1e3 / runs as f64;
    let t = Instant::now();
    let calib = vrc_pano::Calibration::fit(frame.eye(1), frame.width, frame.height);
    let fit_ms = t.elapsed().as_secs_f64() * 1e3;
    // What fresh output memory costs here (page faults: the colour, depth
    // and mask of a frame, 26 MB).
    let t = Instant::now();
    for _ in 0..runs {
        let mut v = vec![0u8; 26 << 20];
        v.iter_mut().step_by(4096).for_each(|b| *b = 1);
        std::hint::black_box(v);
    }
    println!("fresh 26 MB touched: {:.1} ms", t.elapsed().as_secs_f64() * 1e3 / runs as f64);
    println!(
        "colour alone {colour_ms:.1} ms; calibration fit {fit_ms:.2} ms (residual {:.2}, a {:.4}, b {:.4}, cells out {} + {})",
        calib.residual, calib.channels[0].a, calib.channels[0].b, calib.covered, calib.dropped
    );
    let p = decode(&frame, &params)?;
    let t = Instant::now();
    let e = p.heading_equirect(2048);
    let eq = t.elapsed().as_secs_f64() * 1e3;
    println!(
        "{} threads: decode median {:.1} ms (min {:.1}, max {:.1}); classify {classify_ms:.2} ms ({}); equirect 2048 {eq:.1} ms (coverage {:.3})",
        rayon::current_num_threads(),
        times[runs / 2],
        times[0],
        times[runs - 1],
        classified.name(),
        e.pano.coverage
    );
    Ok(())
}
