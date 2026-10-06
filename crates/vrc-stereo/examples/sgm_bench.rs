//! Times SGM on the CPU and on the GPU (when there is one) for a synthetic
//! pair, at the sizes the bot matches (640x640, 64 disparities) and full
//! resolution (1280x1280, 128):
//!
//!     cargo run --release -p vrc-stereo --example sgm_bench

use std::time::Instant;

use vrc_stereo::{Gray, SgmParams};

fn pair(w: usize, h: usize) -> (Gray, Gray) {
    let mut seed = 7u32;
    let mut noise = || {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        (seed % 256) as u8
    };
    let src_w = w + 160;
    let src: Vec<u8> = (0..src_w * h).map(|_| noise()).collect();
    let (mut l, mut r) = (vec![0u8; w * h], vec![0u8; w * h]);
    for y in 0..h {
        let s = 4 + y * 40 / h;
        for x in 0..w {
            l[y * w + x] = src[y * src_w + x + 80];
            r[y * w + x] = src[y * src_w + x + 80 + s];
        }
    }
    (Gray { width: w, height: h, data: l }, Gray { width: w, height: h, data: r })
}

fn time(runs: usize, mut f: impl FnMut()) -> f64 {
    f(); // warm up (the GPU compiles and allocates on first use)
    let t = Instant::now();
    for _ in 0..runs {
        f();
    }
    t.elapsed().as_secs_f64() * 1000.0 / runs as f64
}

fn main() {
    for (w, nd) in [(640, 64), (1280, 128)] {
        let (l, r) = pair(w, w);
        let p = SgmParams { max_disparity: nd, ..Default::default() };
        let cpu = time(3, || {
            vrc_stereo::sgm::sgm(&l, &r, &p);
        });
        print!("{w}x{w}/{nd}: CPU {cpu:.1} ms");
        #[cfg(feature = "cuda")]
        if vrc_stereo::gpu::available() {
            let gpu = time(20, || {
                vrc_stereo::gpu::sgm(&l, &r, &p).expect("GPU");
            });
            print!(", GPU {gpu:.1} ms ({:.0}x)", cpu / gpu);
        }
        println!();
    }
}
