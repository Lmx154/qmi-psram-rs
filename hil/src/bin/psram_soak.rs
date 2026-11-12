//! Soak (data integrity over time) of the qmi-psram driver, and proof that DMA reads and writes the
//! PSRAM window: ISSI IS66WVS16M8FBLL in socket U24 on QMI chip select 1 (GPIO0). The test never
//! programs or erases the flash and never starts core 1.
//!
//! S0 init, S1 positive control of the mismatch counter, S2 DMA into and out of the window, S3 soak
//! for SOAK_SECONDS (build time, default 60) with two aggressors in thread mode (a CPU spinner and a
//! DMA copy loop between two SRAM buffers on DMA channel 2), S4 the driver is still healthy.
//!
//! The S2 transfers use DMA channel 3 through its registers with a bounded polling wait, so that a
//! bus error or a hang is reported and does not panic in an interrupt handler.
#![no_std]
#![no_main]

use core::future::poll_fn;
use core::hint::black_box;
use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering::Relaxed, Ordering::SeqCst, fence};
use core::task::Poll;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::pac;
use embassy_rp::pac::dma::vals::{DataSize, TreqSel};
use embassy_rp::peripherals::DMA_CH2;
use embassy_rp::{bind_interrupts, dma};
use embassy_time::{Delay, Duration, Instant, Timer};
use panic_probe as _;
use qmi_psram::rp2350::{Cs1Pin, Rp2350Qmi};
use qmi_psram::{Chip, Config, Psram};

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH2>;
});

const MIB: usize = 1024 * 1024;
const SIZE: usize = 16 * MIB;
const DIE: usize = 8 * MIB;
const CHIP: Chip = Chip::IS66WVS16M8FBLL;

const fn parse(s: Option<&str>, default: u32) -> u32 {
    let Some(s) = s else { return default };
    let b = s.as_bytes();
    if b.is_empty() {
        return default;
    }
    let mut i = 0;
    let mut v = 0u32;
    while i < b.len() {
        assert!(b[i] >= b'0' && b[i] <= b'9');
        v = v * 10 + (b[i] - b'0') as u32;
        i += 1;
    }
    v
}

const SOAK_SECONDS: u32 = parse(option_env!("SOAK_SECONDS"), 60);
const _: () = assert!(SOAK_SECONDS >= 1);
/// The soak gives up this long after SOAK_SECONDS.
const SOAK_GRACE_SECONDS: u64 = 60;
/// Every RETENTION_EVERY-th pass is followed by RETENTION_SECONDS without access and a second verify.
const RETENTION_EVERY: u32 = 8;
const RETENTION_SECONDS: u64 = 3;
const PROGRESS_SECONDS: u64 = 10;
/// Words filled or verified between two yields to the aggressors.
const SLICE_WORDS: usize = 512;

/// DMA channel of the S2 transfers (registers, polled) and the longest wait for one transfer.
const S2_CHANNEL: usize = 3;
const DMA_TIMEOUT_MS: u64 = 200;
const CHUNK_WORDS: usize = 16 * 1024;
const CHUNK_BYTES: usize = CHUNK_WORDS * 4;

static AGG_STOP: AtomicBool = AtomicBool::new(false);
static AGG_DONE: AtomicU32 = AtomicU32::new(0);
static SPIN_CHUNKS: AtomicU32 = AtomicU32::new(0);
static DMA_COPIES: AtomicU32 = AtomicU32::new(0);
static DMA_BAD: AtomicU32 = AtomicU32::new(0);

static mut BUF_A: [u32; CHUNK_WORDS] = [0; CHUNK_WORDS];
static mut BUF_B: [u32; CHUNK_WORDS] = [0; CHUNK_WORDS];
const AGG_WORDS: usize = 4096;
static mut AGG_SRC: [u32; AGG_WORDS] = [0; AGG_WORDS];
static mut AGG_DST: [u32; AGG_WORDS] = [0; AGG_WORDS];

/// Reports one verdict line; the format string starts with `{=str}`, which receives the tag.
macro_rules! check {
    ($failures:expr, $cond:expr, $fmt:literal $(, $arg:expr)* $(,)?) => {
        if $cond {
            defmt::info!($fmt, "[PASS]" $(, $arg)*);
        } else {
            $failures += 1;
            defmt::error!($fmt, "[FAIL]" $(, $arg)*);
        }
    };
}

fn pattern(i: u32) -> u32 {
    let mut x = i.wrapping_mul(0x9E37_79B9) ^ 0x5BD1_E995;
    x ^= x >> 15;
    x = x.wrapping_mul(0x2C1B_3C6D);
    x ^ (x >> 12)
}

/// Writes `words` words at `addr`; word k holds `pattern(first_idx + k) ^ seed`.
#[inline(never)]
fn fill_at(addr: usize, words: usize, first_idx: u32, seed: u32) {
    let p = addr as *mut u32;
    for k in 0..words {
        // SAFETY: the caller passes a programmed window or an SRAM buffer and a range inside it.
        unsafe {
            p.add(k)
                .write_volatile(pattern(first_idx + k as u32) ^ seed)
        };
    }
}

/// The mismatching words seen by `verify_at`: how many, and the first eight (address, expected,
/// found).
#[derive(Clone, Copy)]
struct Rec {
    count: u32,
    first: [(u32, u32, u32); 8],
}

impl Rec {
    const NEW: Rec = Rec {
        count: 0,
        first: [(0, 0, 0); 8],
    };

    #[cold]
    #[inline(never)]
    fn push(&mut self, addr: u32, expected: u32, got: u32) {
        if let Some(slot) = self.first.get_mut(self.count as usize) {
            *slot = (addr, expected, got);
        }
        self.count += 1;
    }
}

/// Compares `words` words at `addr` with `pattern(first_idx + k) ^ seed` and records every
/// difference in `rec`.
#[inline(never)]
fn verify_at(addr: usize, words: usize, first_idx: u32, seed: u32, rec: &mut Rec) {
    let p = addr as *const u32;
    for k in 0..words {
        let expected = pattern(first_idx + k as u32) ^ seed;
        // SAFETY: as in `fill_at`.
        let got = unsafe { p.add(k).read_volatile() };
        if got != expected {
            rec.push((addr + 4 * k) as u32, expected, got);
        }
    }
}

fn log_rec(tag: &'static str, rec: &Rec) {
    for &(addr, expected, got) in rec.first.iter().take(rec.count.min(8) as usize) {
        defmt::info!(
            "[INFO] {=str} mismatch: address {=u32:#010x} expected {=u32:#010x} found {=u32:#010x}",
            tag,
            addr,
            expected,
            got
        );
    }
}

/// MiB/s x 100 for `bytes` in `us`.
fn rate(bytes: u64, us: u64) -> u32 {
    (bytes * 100 * 1_000_000 / (us.max(1) * MIB as u64)) as u32
}

async fn yield_now() {
    let mut done = false;
    poll_fn(|cx| {
        if done {
            Poll::Ready(())
        } else {
            done = true;
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    })
    .await
}

/// Arithmetic in chunks of about 30 us, then yield. Thread mode.
#[embassy_executor::task]
async fn spinner() {
    let mut x = 12345u32;
    while !AGG_STOP.load(Relaxed) {
        for _ in 0..1000 {
            x = black_box(x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223));
        }
        SPIN_CHUNKS.fetch_add(1, Relaxed);
        yield_now().await;
    }
    black_box(x);
    AGG_DONE.fetch_add(1, Relaxed);
}

/// DMA memory-to-memory copy of 16 KiB, over and over, on a channel the S2 proof does not use.
#[embassy_executor::task]
async fn dma_copier(mut ch: dma::Channel<'static>) {
    // SAFETY: AGG_SRC and AGG_DST are used by this task only.
    let (src, dst) = unsafe { (&mut *addr_of_mut!(AGG_SRC), &mut *addr_of_mut!(AGG_DST)) };
    for (i, v) in src.iter_mut().enumerate() {
        *v = (i as u32).wrapping_mul(0x9E37_79B1) ^ 0xA5A5_5A5A;
    }
    while !AGG_STOP.load(Relaxed) {
        // SAFETY: both slices are in SRAM, reachable by DMA, and live for the whole task.
        unsafe { ch.copy(&src[..], &mut dst[..]) }.await;
        let n = DMA_COPIES.fetch_add(1, Relaxed) + 1;
        if n % 16 == 0 {
            if src[..] != dst[..] {
                DMA_BAD.fetch_add(1, Relaxed);
            }
            dst.fill(0);
        }
    }
    AGG_DONE.fetch_add(1, Relaxed);
}

#[derive(Clone, Copy, defmt::Format)]
enum DmaErr {
    /// The transfer did not finish within the bound and was aborted.
    Timeout { still_busy: bool, remaining: u32 },
    /// The channel reports a bus error.
    Bus {
        read: bool,
        write: bool,
        read_addr: u32,
        write_addr: u32,
    },
}

/// Copies `words` 32-bit words from `src` to `dst` with the registers of channel `S2_CHANNEL`,
/// polling for completion for at most `DMA_TIMEOUT_MS`. Returns the time in microseconds.
fn dma_copy_words(src: usize, dst: usize, words: usize) -> Result<u64, DmaErr> {
    let ch = pac::DMA.ch(S2_CHANNEL);
    // Clear READ_ERROR and WRITE_ERROR (write one to clear); EN stays off, nothing is triggered.
    ch.al1_ctrl().write_value((1 << 30) | (1 << 29));
    ch.read_addr().write_value(src as u32);
    ch.write_addr().write_value(dst as u32);
    ch.trans_count().write(|w| {
        w.set_mode(0.into());
        w.set_count(words as u32);
    });
    fence(SeqCst);
    cortex_m::asm::dsb();
    let t0 = Instant::now();
    ch.ctrl_trig().write(|w| {
        w.set_treq_sel(TreqSel::PERMANENT);
        w.set_data_size(DataSize::SIZE_WORD);
        w.set_incr_read(true);
        w.set_incr_write(true);
        w.set_chain_to(S2_CHANNEL as u8);
        w.set_en(true);
    });
    let bound = Duration::from_millis(DMA_TIMEOUT_MS);
    loop {
        let s = ch.ctrl_trig().read();
        if s.ahb_error() || s.read_error() || s.write_error() {
            return Err(DmaErr::Bus {
                read: s.read_error(),
                write: s.write_error(),
                read_addr: ch.read_addr().read(),
                write_addr: ch.write_addr().read(),
            });
        }
        if !s.busy() {
            let us = t0.elapsed().as_micros();
            fence(SeqCst);
            return Ok(us);
        }
        if t0.elapsed() > bound {
            let remaining = ch.trans_count().read().count();
            pac::DMA
                .chan_abort()
                .write(|w| w.set_chan_abort(1 << S2_CHANNEL));
            let mut spin = 1_000_000u32;
            while ch.ctrl_trig().read().busy() && spin > 0 {
                spin -= 1;
            }
            return Err(DmaErr::Timeout {
                still_busy: ch.ctrl_trig().read().busy(),
                remaining,
            });
        }
    }
}

struct S2 {
    chunks: u32,
    bytes_each_way: u64,
    w_us: u64,
    r_us: u64,
    w_rec: Rec,
    r_rec: Rec,
    /// Words that differed from the expected data before the transfer (destination pre-filled
    /// with something else): proves the compare sees an untransferred destination.
    w_pre: u32,
    r_pre: u32,
    expected_pre: u32,
    err: Option<(&'static str, usize, DmaErr)>,
}

const S2_REGIONS: [usize; 5] = [
    0,
    0x0012_0000,
    DIE - 128 * 1024,
    0x00B3_0000,
    SIZE - 256 * 1024,
];
const S2_CHUNKS_PER_REGION: usize = 4;

fn run_s2(base: usize) -> S2 {
    let buf_a = addr_of_mut!(BUF_A) as usize;
    let buf_b = addr_of_mut!(BUF_B) as usize;
    let mut s = S2 {
        chunks: 0,
        bytes_each_way: 0,
        w_us: 0,
        r_us: 0,
        w_rec: Rec::NEW,
        r_rec: Rec::NEW,
        w_pre: 0,
        r_pre: 0,
        expected_pre: 0,
        err: None,
    };
    'regions: for region in S2_REGIONS {
        for c in 0..S2_CHUNKS_PER_REGION {
            let off = region + c * CHUNK_BYTES;
            let win = base + off;
            let idx = (off / 4) as u32;
            let wseed = pattern(off as u32) | 1;
            let rseed = pattern(off as u32 ^ 0x5555_5555) | 1;

            // SRAM to window. The window chunk holds other data first; the CPU reads back what DMA wrote.
            fill_at(win, CHUNK_WORDS, idx, !wseed);
            let mut pre = Rec::NEW;
            verify_at(win, CHUNK_WORDS, idx, wseed, &mut pre);
            s.w_pre += pre.count;
            fill_at(buf_a, CHUNK_WORDS, idx, wseed);
            match dma_copy_words(buf_a, win, CHUNK_WORDS) {
                Ok(us) => s.w_us += us,
                Err(e) => {
                    s.err = Some(("write", off, e));
                    break 'regions;
                }
            }
            verify_at(win, CHUNK_WORDS, idx, wseed, &mut s.w_rec);

            // Window to SRAM. The window holds new data written by the CPU, the SRAM buffer poison.
            fill_at(win, CHUNK_WORDS, idx, rseed);
            fill_at(buf_b, CHUNK_WORDS, 0, 0xDEAD_BEEF);
            let mut pre = Rec::NEW;
            verify_at(buf_b, CHUNK_WORDS, idx, rseed, &mut pre);
            s.r_pre += pre.count;
            match dma_copy_words(win, buf_b, CHUNK_WORDS) {
                Ok(us) => s.r_us += us,
                Err(e) => {
                    s.err = Some(("read", off, e));
                    break 'regions;
                }
            }
            verify_at(buf_b, CHUNK_WORDS, idx, rseed, &mut s.r_rec);

            s.chunks += 1;
            s.bytes_each_way += CHUNK_BYTES as u64;
            s.expected_pre += CHUNK_WORDS as u32;
        }
    }
    s
}

/// Fills or verifies the whole window with `seed` in slices, yielding to the aggressors between
/// them. Err when `hard` has passed.
async fn sweep(
    base: usize,
    seed: u32,
    write: bool,
    rec: &mut Rec,
    hard: Instant,
) -> Result<(), ()> {
    let mut w = 0;
    while w < SIZE / 4 {
        if Instant::now() > hard {
            return Err(());
        }
        let addr = base + 4 * w;
        if write {
            fill_at(addr, SLICE_WORDS, w as u32, seed);
        } else {
            verify_at(addr, SLICE_WORDS, w as u32, seed, rec);
        }
        w += SLICE_WORDS;
        yield_now().await;
    }
    Ok(())
}

fn pass_seed(pass: u32) -> u32 {
    pass.wrapping_mul(0x9E37_79B1) ^ 0xA5A5_1234
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut failures = 0u32;
    // QMI CS1 and GPIO0 belong to the driver from here on; DMA channel 3 belongs to S2.
    let _qmi_cs1 = p.QMI_CS1;
    let _cs1_pin = p.PIN_0;
    let _ch3 = p.DMA_CH3;
    let clk = embassy_rp::clocks::clk_sys_freq();

    // S0: start.
    defmt::info!(
        "[INFO] S0 psram_soak: {=str} on QMI CS1 = GPIO0, clk_sys {} Hz, SOAK_SECONDS {}",
        CHIP.name,
        clk,
        SOAK_SECONDS
    );
    // SAFETY: QMI CS1 and GPIO0 are held unused above and core 1 is not started. DMA touches the
    // window only between calls of `init` and `remap`, and every transfer has finished by then.
    let qmi = unsafe { Rp2350Qmi::new(Cs1Pin::Gpio0) };
    let mut psram = Psram::new(qmi);
    let info = match psram.init(&mut Delay, clk) {
        Ok(info) => info,
        Err(e) => {
            defmt::error!("[FAIL] S0 init: {} (stopping)", e);
            hil_common::exit(failures + 1)
        }
    };
    let t = psram.timing().unwrap();
    let base = psram.base() as usize;
    check!(
        failures,
        info.id.manufacturer == 0x9D
            && info.id.kgd == 0x5D
            && info.id.density_mbit() == Some(128)
            && info.size == SIZE,
        "{=str} S0 init: ID {=u8:02x} {=u8:02x} (expect 9D 5D), density {} Mbit (expect 128), size {} bytes, window at {=usize:#010x}",
        info.id.manufacturer,
        info.id.kgd,
        info.id.density_mbit(),
        info.size,
        base
    );
    defmt::info!(
        "[INFO] S0 timing: SCK {} Hz (CLKDIV {}), RXDELAY {}",
        t.sck_hz(),
        t.clkdiv,
        t.rx_delay
    );

    // S1: positive control of the mismatch counter.
    let n1 = 256 * 1024 / 4;
    let s1_seed = 0x1CE5_0001u32;
    fill_at(base, n1, 0, s1_seed);
    let mut clean0 = Rec::NEW;
    verify_at(base, n1, 0, s1_seed, &mut clean0);
    // (a) The compare against a different seed.
    let mut wrong_seed = Rec::NEW;
    verify_at(base, n1, 0, s1_seed ^ 0x8000_0001, &mut wrong_seed);
    // (b) Wrong RXDELAY values, which the smoke test shows fail at this CLKDIV.
    let mut bad_rx = [(0u8, Rec::NEW), (7u8, Rec::NEW)];
    let mut remap_ok = true;
    for (rx, rec) in bad_rx.iter_mut() {
        psram.set_config(Config {
            max_sck_hz: Some(clk / u32::from(t.clkdiv)),
            rx_delay: Some(*rx),
        });
        remap_ok &= psram.remap().is_ok();
        verify_at(base, n1, 0, s1_seed, rec);
    }
    // (c) One flipped bit.
    psram.set_config(Config::DEFAULT);
    remap_ok &= psram.remap().is_ok() && psram.timing() == Some(t);
    let victim = DIE / 4 + 3;
    fill_at(base + 4 * (victim - 3), 16, (victim - 3) as u32, s1_seed);
    let w = base as *mut u32;
    // SAFETY: inside the window.
    let saved = unsafe { w.add(victim).read_volatile() };
    unsafe { w.add(victim).write_volatile(saved ^ (1 << 16)) };
    let mut bit = Rec::NEW;
    verify_at(
        base + 4 * (victim - 3),
        16,
        (victim - 3) as u32,
        s1_seed,
        &mut bit,
    );
    unsafe { w.add(victim).write_volatile(saved) };
    let mut bit_restored = Rec::NEW;
    verify_at(
        base + 4 * (victim - 3),
        16,
        (victim - 3) as u32,
        s1_seed,
        &mut bit_restored,
    );
    // Correct set-up restored: zero.
    let mut restored = Rec::NEW;
    verify_at(base, n1, 0, s1_seed, &mut restored);
    check!(
        failures,
        clean0.count == 0
            && wrong_seed.count == n1 as u32
            && bad_rx[0].1.count > 0
            && bad_rx[1].1.count > 0
            && bit.count == 1
            && bit.first[0].0 == (base + 4 * victim) as u32
            && bit_restored.count == 0
            && remap_ok
            && restored.count == 0,
        "{=str} S1 positive control: first verify {} (expect 0); wrong seed {} of {} words; RXDELAY 0 gives {}, RXDELAY 7 gives {} (expect > 0); one flipped bit counted {} (expect 1), after restoring {}; correct set-up restored (remap ok {}): {} (expect 0)",
        clean0.count,
        wrong_seed.count,
        n1,
        bad_rx[0].1.count,
        bad_rx[1].1.count,
        bit.count,
        bit_restored.count,
        remap_ok,
        restored.count
    );
    log_rec("S1 RXDELAY 7", &bad_rx[1].1);

    // S2: DMA into and out of the window.
    let s2 = run_s2(base);
    let w_rate = rate(s2.bytes_each_way, s2.w_us);
    let r_rate = rate(s2.bytes_each_way, s2.r_us);
    defmt::info!(
        "[STAT] S2 DMA channel {} 32-bit words, {} chunks of {} KiB each way ({} KiB written, {} KiB read) at window offsets {=usize:#x}, {=usize:#x}, {=usize:#x} (straddles the 8 MiB die boundary), {=usize:#x}, {=usize:#x} (ends at 16 MiB)",
        S2_CHANNEL,
        s2.chunks,
        CHUNK_BYTES / 1024,
        s2.bytes_each_way / 1024,
        s2.bytes_each_way / 1024,
        S2_REGIONS[0],
        S2_REGIONS[1],
        S2_REGIONS[2],
        S2_REGIONS[3],
        S2_REGIONS[4]
    );
    defmt::info!(
        "[STAT] S2 DMA SRAM -> window {}.{=u32:02} MiB/s, window -> SRAM {}.{=u32:02} MiB/s (transfer time only)",
        w_rate / 100,
        w_rate % 100,
        r_rate / 100,
        r_rate % 100
    );
    defmt::info!(
        "[STAT] S2 controls: destination differed from the expected data before the transfer in {} (write) and {} (read) of {} words each",
        s2.w_pre,
        s2.r_pre,
        s2.expected_pre
    );
    if let Some((dir, off, e)) = &s2.err {
        defmt::error!(
            "[FAIL] S2 FINDING: DMA {=str} at window offset {=usize:#x} did not complete: {}",
            *dir,
            *off,
            e
        );
        failures += 1;
    }
    log_rec("S2 DMA write, CPU read", &s2.w_rec);
    log_rec("S2 DMA read, CPU compare", &s2.r_rec);
    check!(
        failures,
        s2.err.is_none()
            && s2.bytes_each_way >= MIB as u64
            && s2.w_rec.count == 0
            && s2.r_rec.count == 0
            && s2.w_pre == s2.expected_pre
            && s2.r_pre > 0,
        "{=str} S2 DMA: {} chunks completed, mismatches after DMA write {} and after DMA read {} (expect 0), {} bytes each way (expect >= 1 MiB)",
        s2.chunks,
        s2.w_rec.count,
        s2.r_rec.count,
        s2.bytes_each_way
    );

    // S3: soak with the aggressors.
    AGG_STOP.store(false, Relaxed);
    let ch2 = dma::Channel::new(p.DMA_CH2, Irqs);
    spawner.spawn(spinner().unwrap());
    spawner.spawn(dma_copier(ch2).unwrap());
    let start = Instant::now();
    let soak = Duration::from_secs(u64::from(SOAK_SECONDS));
    let hard = start + soak + Duration::from_secs(SOAK_GRACE_SECONDS);
    let mut rec = Rec::NEW;
    let (mut passes, mut retention_checks, mut retention_bad) = (0u32, 0u32, 0u32);
    let (mut w_us, mut v_us) = (0u64, 0u64);
    let (mut written, mut verified) = (0u64, 0u64);
    let mut timed_out = false;
    let mut next_progress = start + Duration::from_secs(PROGRESS_SECONDS);
    while Instant::now() - start < soak {
        let seed = pass_seed(passes);
        let t0 = Instant::now();
        if sweep(base, seed, true, &mut rec, hard).await.is_err() {
            timed_out = true;
            break;
        }
        w_us += t0.elapsed().as_micros();
        written += SIZE as u64;
        let t0 = Instant::now();
        if sweep(base, seed, false, &mut rec, hard).await.is_err() {
            timed_out = true;
            break;
        }
        v_us += t0.elapsed().as_micros();
        verified += SIZE as u64;
        passes += 1;
        if passes % RETENTION_EVERY == 0 {
            let before = rec.count;
            Timer::after_secs(RETENTION_SECONDS).await;
            if sweep(base, seed, false, &mut rec, hard).await.is_err() {
                timed_out = true;
                break;
            }
            verified += SIZE as u64;
            retention_checks += 1;
            retention_bad += rec.count - before;
        }
        let now = Instant::now();
        if now >= next_progress {
            next_progress += Duration::from_secs(PROGRESS_SECONDS);
            defmt::info!(
                "[STAT] S3 progress t={}s passes={} mismatches={} spinner_chunks={} dma_copies={}",
                (now - start).as_secs(),
                passes,
                rec.count,
                SPIN_CHUNKS.load(Relaxed),
                DMA_COPIES.load(Relaxed)
            );
        }
    }
    let elapsed = (Instant::now() - start).as_secs();
    AGG_STOP.store(true, Relaxed);
    let stop_by = Instant::now() + Duration::from_secs(2);
    while AGG_DONE.load(Relaxed) < 2 && Instant::now() < stop_by {
        Timer::after_millis(10).await;
    }
    let aggressors_stopped = AGG_DONE.load(Relaxed) == 2;
    let (spin, copies, copy_bad) = (
        SPIN_CHUNKS.load(Relaxed),
        DMA_COPIES.load(Relaxed),
        DMA_BAD.load(Relaxed),
    );
    let (wr, vr) = (rate(written, w_us), rate(verified, v_us));
    defmt::info!(
        "[STAT] S3 {} s: passes {}, written {} MiB, verified {} MiB, mismatching words {}, retention checks {} (mismatches in them {})",
        elapsed,
        passes,
        written / MIB as u64,
        verified / MIB as u64,
        rec.count,
        retention_checks,
        retention_bad
    );
    defmt::info!(
        "[STAT] S3 under load: fill {}.{=u32:02} MiB/s, verify {}.{=u32:02} MiB/s (wall time including the aggressors)",
        wr / 100,
        wr % 100,
        vr / 100,
        vr % 100
    );
    defmt::info!(
        "[STAT] S3 aggressors: spinner_chunks={} dma_copies={} dma_copy_bad={} stopped={}",
        spin,
        copies,
        copy_bad,
        aggressors_stopped
    );
    log_rec("S3", &rec);
    if timed_out {
        defmt::error!(
            "[FAIL] S3 did not finish within SOAK_SECONDS + {} s",
            SOAK_GRACE_SECONDS
        );
        failures += 1;
    }
    check!(
        failures,
        rec.count == 0
            && passes >= 2
            && !timed_out
            && spin > 0
            && copies > 0
            && copy_bad == 0
            && aggressors_stopped,
        "{=str} S3 soak: mismatching words {} (expect 0), {} full passes (expect >= 2), aggressors ran: spinner {}, DMA copies {}, copy errors {}",
        rec.count,
        passes,
        spin,
        copies,
        copy_bad
    );

    // S4: the driver is still healthy.
    let mut s4_ok = true;
    let again = psram.init(&mut Delay, clk);
    s4_ok &= again == Ok(info);
    let quick_seed = 0x0F1E_2D3Cu32;
    let mut quick = Rec::NEW;
    let quick_words = MIB / 4;
    let quick_at = 4 * MIB;
    fill_at(
        base + quick_at,
        quick_words,
        (quick_at / 4) as u32,
        quick_seed,
    );
    verify_at(
        base + quick_at,
        quick_words,
        (quick_at / 4) as u32,
        quick_seed,
        &mut quick,
    );
    let last_at = SIZE - 64 * 1024;
    fill_at(base + last_at, 16 * 1024, (last_at / 4) as u32, !quick_seed);
    verify_at(
        base + last_at,
        16 * 1024,
        (last_at / 4) as u32,
        !quick_seed,
        &mut quick,
    );
    check!(
        failures,
        s4_ok && quick.count == 0,
        "{=str} S4 end: second init {} (ID equal to S0: {}), 1 MiB at 4 MiB plus the last 64 KiB: {} mismatches (expect 0)",
        again,
        s4_ok,
        quick.count
    );
    hil_common::exit(failures)
}
