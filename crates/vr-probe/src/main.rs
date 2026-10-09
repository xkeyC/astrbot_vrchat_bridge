//! vr-probe: pokes the bot's virtual headset by hand.
//!
//!     vr-probe info                       # the latest tapped frame, as JSON-ish text
//!     vr-probe grab eyes.png              # both eyes side by side
//!     vr-probe look --yaw 60 --pitch -20  # turn the head (held while connected)
//!     vr-probe sweep out/ --yaws=-60,0,60 # turn, settle, grab, for each yaw
//!     vr-probe walk --yaw 0 --metres 1    # one measured leg
//!
//! Depth, surveys and walks to a place come from the bridge (the avatar's
//! panorama: `/v1/vr/pano`, `/v1/vr/survey`, `/v1/vr/goto`).

use std::fs::File;
use std::io::BufWriter;
use std::path::{Path, PathBuf};
use std::thread::sleep;
use std::time::Duration;

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
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
            hmd.state.hands_at_rest(head, yaw);
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
                "rest" => hmd.state.hands_at_rest(head, yaw),
                "up-behind" => hmd.state.hands_up_behind(head, yaw),
                other => anyhow::bail!("no hand pose {other}"),
            }
            hmd.send()?;
            sleep(Duration::from_secs_f32(hold));
        }
    }
    Ok(())
}

fn latest(tap: &mut EyeTap) -> Result<EyeFrame> {
    tap.read()?.context("Monado has not tapped a frame yet")
}

/// A frame in a line: its size, format, the eyes' intrinsics and pose.
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
