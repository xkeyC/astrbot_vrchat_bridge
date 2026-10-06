//! [`sgm`](crate::sgm::sgm) on an NVIDIA GPU (CUDA through `cudarc`; the
//! kernels are compiled at run time by NVRTC, so building needs no CUDA
//! toolkit and a machine without one runs the CPU version).
//!
//! The same steps as the CPU version, one kernel each: census, census costs,
//! the 8 aggregation paths (one thread block per scanline of a direction,
//! one thread per disparity, the step's neighbours through shared memory and
//! its minimum by warp reductions; a direction touches each pixel once, so
//! the paths add up without atomics), the right image's best disparities,
//! then winner-take-all with the uniqueness, subpixel and left/right checks.
//! Integer costs are the CPU version's exactly, and floating point is kept
//! from fusing multiply-adds: the disparities are the same, bit for bit
//! (tested when a GPU is there).

use std::sync::{Arc, Mutex, OnceLock};

use cudarc::driver::{CudaContext, CudaFunction, CudaSlice, CudaStream, LaunchConfig, PushKernelArg};
use cudarc::nvrtc::{compile_ptx_with_opts, CompileOptions};

use crate::sgm::{Disparity, Gray, SgmParams};

const KERNELS: &str = r#"
#define CENSUS_R 3
#define OUTSIDE 48

extern "C" __global__ void census(const unsigned char* img, unsigned long long* out, int w, int h) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= w || y >= h) return;
    unsigned char c = img[y * w + x];
    unsigned long long v = 0;
    for (int dy = -CENSUS_R; dy <= CENSUS_R; dy++) {
        int yy = min(max(y + dy, 0), h - 1);
        for (int dx = -CENSUS_R; dx <= CENSUS_R; dx++) {
            if (dx == 0 && dy == 0) continue;
            int xx = min(max(x + dx, 0), w - 1);
            v = (v << 1) | (unsigned long long)(img[yy * w + xx] < c);
        }
    }
    out[y * w + x] = v;
}

extern "C" __global__ void costs(const unsigned long long* cl, const unsigned long long* cr, unsigned char* cost, int w, int h, int nd) {
    long long i = (long long)blockIdx.x * blockDim.x + threadIdx.x;
    long long n = (long long)w * h * nd;
    if (i >= n) return;
    int d = (int)(i % nd);
    long long p = i / nd;
    int x = (int)(p % w);
    int y = (int)(p / w);
    cost[i] = x >= d ? (unsigned char)__popcll(cl[p] ^ cr[(long long)y * w + x - d]) : (unsigned char)OUTSIDE;
}

// One scanline of direction (dx, dy) per block, from starts[2*blockIdx.x];
// one thread per disparity (blockDim.x >= nd, a multiple of 32).
extern "C" __global__ void aggregate(const unsigned char* cost, unsigned short* sum, const int* starts,
                                     int w, int h, int nd, int dx, int dy, int p1, int p2) {
    extern __shared__ unsigned short sh[];
    unsigned short* prev = sh;              // nd
    unsigned int* wmin = (unsigned int*)(sh + ((nd + 1) & ~1)); // one per warp
    int d = threadIdx.x;
    int warps = blockDim.x / 32;
    int x = starts[2 * blockIdx.x];
    int y = starts[2 * blockIdx.x + 1];
    unsigned int prev_min = 0;
    bool first = true;
    while (x >= 0 && x < w && y >= 0 && y < h) {
        long long p = ((long long)y * w + x) * nd;
        unsigned int v = 0xffff;
        if (d < nd) {
            unsigned int c = cost[p + d];
            if (first) {
                v = c;
            } else {
                unsigned int m = min((unsigned int)prev[d], prev_min + (unsigned int)p2);
                if (d > 0) m = min(m, (unsigned int)prev[d - 1] + (unsigned int)p1);
                if (d + 1 < nd) m = min(m, (unsigned int)prev[d + 1] + (unsigned int)p1);
                v = c + m - prev_min;
            }
            sum[p + d] = (unsigned short)(sum[p + d] + v);
        }
        __syncthreads(); // everyone has read prev
        if (d < nd) prev[d] = (unsigned short)v;
        unsigned int m = v;
        for (int o = 16; o > 0; o >>= 1) m = min(m, __shfl_down_sync(0xffffffffu, m, o));
        if ((d & 31) == 0) wmin[d >> 5] = m;
        __syncthreads();
        unsigned int all = wmin[0];
        for (int k = 1; k < warps; k++) all = min(all, wmin[k]);
        prev_min = all;
        first = false;
        x += dx;
        y += dy;
        __syncthreads(); // wmin read before the next step writes it
    }
}

// The right image's best disparity: for right pixel xr, the d of the left
// pixel xr + d that would show it.
extern "C" __global__ void right_best(const unsigned short* sum, unsigned short* best, int w, int h, int nd) {
    int xr = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y;
    if (xr >= w || y >= h) return;
    unsigned int min_v = 0xffff;
    unsigned short b = 0xffff;
    int n = min(nd, w - xr);
    for (int d = 0; d < n; d++) {
        unsigned int s = sum[((long long)y * w + xr + d) * nd + d];
        if (s < min_v) { min_v = s; b = (unsigned short)d; }
    }
    best[y * w + xr] = b;
}

extern "C" __global__ void choose(const unsigned short* sum, const unsigned short* rbest, float* out,
                                  int w, int h, int nd, float uniqueness, float lr_tolerance) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y;
    if (x >= w || y >= h) return;
    const unsigned short* s = sum + ((long long)y * w + x) * nd;
    int n = min(nd, x + 1);
    unsigned int best = 0xffff;
    int best_d = 0;
    for (int d = 0; d < n; d++) {
        if (s[d] < best) { best = s[d]; best_d = d; }
    }
    unsigned int other = 0xffff;
    for (int d = 0; d < n; d++) {
        if (abs(d - best_d) > 1 && s[d] < other) other = s[d];
    }
    float nan = __int_as_float(0x7fc00000);
    out[y * w + x] = nan;
    if ((float)best * (1.0f + uniqueness) > (float)other) return;
    float disp = (float)best_d;
    if (best_d > 0 && best_d + 1 < n) {
        float a = (float)s[best_d - 1], b = (float)best, c = (float)s[best_d + 1];
        float denom = a - 2.0f * b + c;
        if (denom > 0.0f) disp += (a - c) / (2.0f * denom);
    }
    float xr = roundf((float)x - disp);
    if (xr < 0.0f) return;
    unsigned short back = rbest[y * w + (int)xr];
    if (back != 0xffff && fabsf((float)back - disp) <= lr_tolerance) out[y * w + x] = disp;
}
"#;

/// The GPU, its kernels, and buffers for the last size matched.
struct Gpu {
    stream: Arc<CudaStream>,
    census: CudaFunction,
    costs: CudaFunction,
    aggregate: CudaFunction,
    right_best: CudaFunction,
    choose: CudaFunction,
    buffers: Option<Buffers>,
}

struct Buffers {
    w: usize,
    h: usize,
    nd: usize,
    left: CudaSlice<u8>,
    right: CudaSlice<u8>,
    cl: CudaSlice<u64>,
    cr: CudaSlice<u64>,
    cost: CudaSlice<u8>,
    sum: CudaSlice<u16>,
    rbest: CudaSlice<u16>,
    out: CudaSlice<f32>,
    /// The scanline starts of each of the 8 directions.
    starts: Vec<(i32, i32, CudaSlice<i32>, u32)>,
}

const DIRECTIONS: [(i32, i32); 8] = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)];

impl Gpu {
    fn new() -> anyhow::Result<Gpu> {
        let ctx = CudaContext::new(0)?;
        let ptx = compile_ptx_with_opts(KERNELS, CompileOptions { fmad: Some(false), ..Default::default() })?;
        let module = ctx.load_module(ptx)?;
        Ok(Gpu {
            stream: ctx.default_stream(),
            census: module.load_function("census")?,
            costs: module.load_function("costs")?,
            aggregate: module.load_function("aggregate")?,
            right_best: module.load_function("right_best")?,
            choose: module.load_function("choose")?,
            buffers: None,
        })
    }

    fn buffers(&mut self, w: usize, h: usize, nd: usize) -> anyhow::Result<&mut Buffers> {
        if self.buffers.as_ref().is_none_or(|b| (b.w, b.h, b.nd) != (w, h, nd)) {
            let s = &self.stream;
            let mut starts = Vec::new();
            for (dx, dy) in DIRECTIONS {
                // Where a scanline begins: its previous pixel is outside.
                let mut list: Vec<i32> = Vec::new();
                for y in 0..h as i32 {
                    for x in 0..w as i32 {
                        let (px, py) = (x - dx, y - dy);
                        if px < 0 || px >= w as i32 || py < 0 || py >= h as i32 {
                            list.extend([x, y]);
                        }
                    }
                }
                let n = (list.len() / 2) as u32;
                starts.push((dx, dy, s.clone_htod(&list)?, n));
            }
            self.buffers = Some(Buffers {
                w,
                h,
                nd,
                left: s.alloc_zeros(w * h)?,
                right: s.alloc_zeros(w * h)?,
                cl: s.alloc_zeros(w * h)?,
                cr: s.alloc_zeros(w * h)?,
                cost: s.alloc_zeros(w * h * nd)?,
                sum: s.alloc_zeros(w * h * nd)?,
                rbest: s.alloc_zeros(w * h)?,
                out: s.alloc_zeros(w * h)?,
                starts,
            });
        }
        Ok(self.buffers.as_mut().expect("just made"))
    }

    fn sgm(&mut self, left: &Gray, right: &Gray, p: &SgmParams) -> anyhow::Result<Disparity> {
        let (w, h, nd) = (left.width, left.height, p.max_disparity);
        anyhow::ensure!((1..=1024).contains(&nd), "max_disparity must be within 1..1024");
        let stream = self.stream.clone();
        let (census, costs, aggregate, right_best, choose) =
            (self.census.clone(), self.costs.clone(), self.aggregate.clone(), self.right_best.clone(), self.choose.clone());
        let b = self.buffers(w, h, nd)?;
        let (wi, hi, ndi) = (w as i32, h as i32, nd as i32);
        stream.memcpy_htod(&left.data, &mut b.left)?;
        stream.memcpy_htod(&right.data, &mut b.right)?;
        let tile = LaunchConfig { grid_dim: (w.div_ceil(16) as u32, h.div_ceil(16) as u32, 1), block_dim: (16, 16, 1), shared_mem_bytes: 0 };
        for (img, out) in [(&b.left, &mut b.cl), (&b.right, &mut b.cr)] {
            let mut l = stream.launch_builder(&census);
            l.arg(img).arg(out).arg(&wi).arg(&hi);
            unsafe { l.launch(tile) }?;
        }
        let n = (w * h * nd) as u32;
        let mut l = stream.launch_builder(&costs);
        l.arg(&b.cl).arg(&b.cr).arg(&mut b.cost).arg(&wi).arg(&hi).arg(&ndi);
        unsafe { l.launch(LaunchConfig::for_num_elems(n)) }?;
        stream.memset_zeros(&mut b.sum)?;
        let threads = nd.div_ceil(32) * 32;
        let shared = (nd.div_ceil(2) * 2 * 2 + threads / 32 * 4) as u32;
        let (p1, p2) = (p.p1 as i32, p.p2 as i32);
        for (dx, dy, starts, lines) in &b.starts {
            let mut l = stream.launch_builder(&aggregate);
            l.arg(&b.cost).arg(&mut b.sum).arg(starts).arg(&wi).arg(&hi).arg(&ndi).arg(dx).arg(dy).arg(&p1).arg(&p2);
            let cfg = LaunchConfig { grid_dim: (*lines, 1, 1), block_dim: (threads as u32, 1, 1), shared_mem_bytes: shared };
            unsafe { l.launch(cfg) }?;
        }
        let rows = LaunchConfig { grid_dim: (w.div_ceil(128) as u32, h as u32, 1), block_dim: (128, 1, 1), shared_mem_bytes: 0 };
        let mut l = stream.launch_builder(&right_best);
        l.arg(&b.sum).arg(&mut b.rbest).arg(&wi).arg(&hi).arg(&ndi);
        unsafe { l.launch(rows) }?;
        let mut l = stream.launch_builder(&choose);
        l.arg(&b.sum).arg(&b.rbest).arg(&mut b.out).arg(&wi).arg(&hi).arg(&ndi).arg(&p.uniqueness).arg(&p.lr_tolerance);
        unsafe { l.launch(rows) }?;
        let data = stream.clone_dtoh(&b.out)?;
        Ok(Disparity { width: w, height: h, data })
    }
}

/// The GPU, set up on first use; `None` when there is none (no driver, no
/// NVRTC, `VRC_STEREO_CPU` set) or it failed once.
fn gpu() -> Option<&'static Mutex<Gpu>> {
    static GPU: OnceLock<Option<Mutex<Gpu>>> = OnceLock::new();
    GPU.get_or_init(|| {
        if std::env::var_os("VRC_STEREO_CPU").is_some() {
            return None;
        }
        // cudarc panics when it finds no CUDA library: that is "no GPU".
        match std::panic::catch_unwind(Gpu::new) {
            Ok(Ok(gpu)) => Some(Mutex::new(gpu)),
            Ok(Err(e)) => {
                eprintln!("vrc-stereo: no GPU matching ({e:#}); on the CPU");
                None
            }
            Err(_) => {
                eprintln!("vrc-stereo: no CUDA here; matching on the CPU");
                None
            }
        }
    })
    .as_ref()
}

/// Whether matching runs on the GPU.
pub fn available() -> bool {
    gpu().is_some()
}

/// [`sgm`](crate::sgm::sgm) on the GPU; `None` without one (or if it fails:
/// the caller matches on the CPU).
pub fn sgm(left: &Gray, right: &Gray, p: &SgmParams) -> Option<Disparity> {
    let gpu = gpu()?;
    let mut g = gpu.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
    match g.sgm(left, right, p) {
        Ok(d) => Some(d),
        Err(e) => {
            eprintln!("vrc-stereo: GPU matching failed ({e:#}); on the CPU");
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn noise_pair(w: usize, h: usize, shift: usize) -> (Gray, Gray) {
        let mut seed = 99u32;
        let mut noise = || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            (seed % 256) as u8
        };
        let src_w = w + 80;
        let src: Vec<u8> = (0..src_w * h).map(|_| noise()).collect();
        let (mut l, mut r) = (vec![0u8; w * h], vec![0u8; w * h]);
        for y in 0..h {
            // A slanted plane and a step: disparities change along the image.
            let s = shift + (y * 20 / h) + if (y / 17) % 3 == 0 { 9 } else { 0 };
            for x in 0..w {
                l[y * w + x] = src[y * src_w + x + 40];
                r[y * w + x] = src[y * src_w + (x + 40 + s).min(src_w - 1)];
            }
        }
        (Gray { width: w, height: h, data: l }, Gray { width: w, height: h, data: r })
    }

    #[test]
    fn the_gpu_matches_like_the_cpu() {
        if !available() {
            eprintln!("no GPU: skipped");
            return;
        }
        for (w, h, nd) in [(160, 80, 32), (257, 131, 64), (640, 640, 64)] {
            let (l, r) = noise_pair(w, h, 5);
            let p = SgmParams { max_disparity: nd, ..Default::default() };
            let cpu = crate::sgm::sgm(&l, &r, &p);
            let gpu = sgm(&l, &r, &p).expect("the GPU matched");
            let differ = cpu.data.iter().zip(&gpu.data).filter(|(a, b)| a.to_bits() != b.to_bits()).count();
            assert_eq!(differ, 0, "{w}x{h}/{nd}: {differ} of {} differ", cpu.data.len());
        }
    }
}
