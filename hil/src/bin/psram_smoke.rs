//! On-board check of the qmi-psram driver: ISSI IS66WVS16M8FBLL in socket U24 on QMI chip select 1
//! (GPIO0); clock and SD0-3 are shared with the in-package flash the program runs from. The chip is
//! not power-cycled between runs, so the first `init` starts in SPI or QPI mode. The test never
//! programs or erases the flash: P9 simulates the QMI reset that flash programming causes by
//! writing the M1 registers.
//!
//! A PIO state machine reads GPIO0 (the pin keeps its QMI function) and times every CE# low
//! pulse. The DWT cycle counter times each call the driver makes into the back-end.
#![no_std]
#![no_main]

use core::hint::black_box;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering::Relaxed};

use defmt_rtt as _;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{self, Common, Pio, StateMachine};
use embassy_rp::{bind_interrupts, interrupt};
use embassy_time::{Delay, Instant, Timer};
use panic_probe as _;
use qmi_psram::rp2350::{CACHED_BASE, Cs1Pin, Rp2350Qmi, UNCACHED_BASE};
use qmi_psram::{BusError, Chip, Config, DirectClock, Psram, QmiBus, Width, WindowRegs, opcode};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

const MIB: usize = 1024 * 1024;
const SIZE: usize = 16 * MIB;
const DIE: usize = 8 * MIB;
const PAGE: usize = 1024;
const CHIP: Chip = Chip::IS66WVS16M8FBLL;

const QMI_BASE: usize = 0x400d_0000;
const XIP_CTRL: *mut u32 = 0x400c_8000 as *mut u32;
const WRITABLE_M1: u32 = 1 << 11;

/// Calls into the back-end and their longest duration in DWT cycles since the last reset. Each
/// back-end call holds at most one critical section, so the duration bounds the time interrupts
/// are off.
static TRANSFERS: AtomicU32 = AtomicU32::new(0);
static TRANSFER_MAX: AtomicU32 = AtomicU32::new(0);
static WINDOWS: AtomicU32 = AtomicU32::new(0);
static WINDOW_MAX: AtomicU32 = AtomicU32::new(0);

/// A timer task on an interrupt executor that keeps interrupts firing while `init` runs.
static TICKS: AtomicU32 = AtomicU32::new(0);
static TICK_RUN: AtomicBool = AtomicBool::new(false);
static TICK_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
const TICK_PRIORITY: u8 = 0xE0;

#[interrupt]
unsafe fn SWI_IRQ_1() {
    unsafe { TICK_EXECUTOR.on_interrupt() }
}

#[embassy_executor::task]
async fn ticker() {
    while TICK_RUN.load(Relaxed) {
        Timer::after_micros(20).await;
        TICKS.fetch_add(1, Relaxed);
    }
}

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

fn cycles() -> u32 {
    cortex_m::peripheral::DWT::cycle_count()
}

fn reset_stats() {
    for s in [&TRANSFERS, &TRANSFER_MAX, &WINDOWS, &WINDOW_MAX] {
        s.store(0, Relaxed);
    }
}

/// SysTick fires every `SYSTICK_PERIOD` cycles while armed and records the longest gap between
/// two of its interrupts. Interrupts masked for D cycles give a gap of at least D, at most
/// D + one period: an upper bound of the interrupts-off time that excludes everything the
/// driver does with interrupts on.
const SYSTICK_PERIOD: u32 = 300;
static SYSTICK_LAST: AtomicU32 = AtomicU32::new(0);
static SYSTICK_GAP: AtomicU32 = AtomicU32::new(0);

#[cortex_m_rt::exception]
fn SysTick() {
    let now = cycles();
    let last = SYSTICK_LAST.swap(now, Relaxed);
    if last != 0 {
        SYSTICK_GAP.fetch_max(now.wrapping_sub(last), Relaxed);
    }
}

fn systick_arm() {
    SYSTICK_LAST.store(0, Relaxed);
    SYSTICK_GAP.store(0, Relaxed);
    // SAFETY: SysTick is used by nothing else in this program.
    let mut syst = unsafe { cortex_m::Peripherals::steal() }.SYST;
    syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    syst.set_reload(SYSTICK_PERIOD - 1);
    syst.clear_current();
    syst.enable_interrupt();
    syst.enable_counter();
}

/// Disarms SysTick and returns the longest gap in cycles.
fn systick_disarm() -> u32 {
    // SAFETY: as in `systick_arm`.
    let mut syst = unsafe { cortex_m::Peripherals::steal() }.SYST;
    syst.disable_interrupt();
    syst.disable_counter();
    SYSTICK_GAP.load(Relaxed)
}

/// The back-end with every call timed.
struct Timed<Q>(Q);

// SAFETY: forwards every call to `Q` unchanged.
unsafe impl<Q: QmiBus> QmiBus for Timed<Q> {
    fn transfer(
        &mut self,
        clock: DirectClock,
        width: Width,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), BusError> {
        let t0 = cycles();
        let r = self.0.transfer(clock, width, write, read);
        TRANSFER_MAX.fetch_max(cycles().wrapping_sub(t0), Relaxed);
        TRANSFERS.fetch_add(1, Relaxed);
        r
    }

    fn program_window(&mut self, regs: &WindowRegs) -> Result<(), BusError> {
        let t0 = cycles();
        let r = self.0.program_window(regs);
        WINDOW_MAX.fetch_max(cycles().wrapping_sub(t0), Relaxed);
        WINDOWS.fetch_add(1, Relaxed);
        r
    }

    fn flush_cache(&mut self) {
        self.0.flush_cache()
    }

    fn window_base(&self) -> usize {
        self.0.window_base()
    }
}

/// Times every CE# low pulse on GPIO0 in PIO cycles (clk_sys). The FIFO keeps the first eight
/// pulses after a drain.
struct CeMonitor<'d> {
    sm: StateMachine<'d, PIO0, 0>,
    clk: u32,
}

impl<'d> CeMonitor<'d> {
    fn new(common: &mut Common<'d, PIO0>, mut sm: StateMachine<'d, PIO0, 0>, clk: u32) -> Self {
        // Two cycles per loop; x counts down from all ones, so !x is the number of loops.
        let prg = embassy_rp::pio::program::pio_asm!(
            ".wrap_target",
            "wait 1 pin 0",
            "wait 0 pin 0",
            "mov x, !null",
            "low:",
            "jmp pin high",
            "jmp x-- low",
            "high:",
            "mov isr, !x",
            "push noblock",
            ".wrap",
        );
        let loaded = common.load_program(&prg.program);
        let mut cfg = pio::Config::default();
        cfg.use_program(&loaded, &[]);
        cfg.fifo_join = pio::FifoJoin::RxOnly;
        // IN_BASE and JMP_PIN stay 0, which is GPIO0. The PIO never owns the pin.
        sm.set_config(&cfg);
        sm.set_enable(true);
        Self { sm, clk }
    }

    /// Pulses since the last drain (at most eight are kept) and the longest in nanoseconds,
    /// rounded up, plus four cycles for the edge detection.
    fn drain(&mut self) -> (u32, u32) {
        let (mut n, mut longest) = (0u32, 0u32);
        while let Some(loops) = self.sm.rx().try_pull() {
            n += 1;
            longest = longest.max(loops);
        }
        let cycles = if n == 0 {
            0
        } else {
            2 * u64::from(longest) + 4
        };
        (n, ns(cycles, self.clk))
    }
}

fn ns(cycles: u64, clk: u32) -> u32 {
    (cycles * 1_000_000_000).div_ceil(u64::from(clk)) as u32
}

fn pattern(i: u32) -> u32 {
    let mut x = i.wrapping_mul(0x9E37_79B9) ^ 0x5BD1_E995;
    x ^= x >> 15;
    x = x.wrapping_mul(0x2C1B_3C6D);
    x ^ (x >> 12)
}

#[inline(never)]
fn fill(base: usize, first: usize, words: usize, seed: u32) {
    let p = base as *mut u32;
    for i in first..first + words {
        // SAFETY: the caller passes a programmed window and a range inside it.
        unsafe { p.add(i).write_volatile(pattern(i as u32) ^ seed) };
    }
}

#[inline(never)]
fn read_xor(base: usize, first: usize, words: usize) -> u32 {
    let p = base as *const u32;
    let mut x = 0;
    for i in first..first + words {
        // SAFETY: as in `fill`.
        x ^= unsafe { p.add(i).read_volatile() };
    }
    x
}

#[derive(defmt::Format)]
struct Mismatch {
    count: u32,
    offset: u32,
    expected: u32,
    got: u32,
}

#[inline(never)]
fn verify(base: usize, first: usize, words: usize, seed: u32) -> Mismatch {
    let p = base as *const u32;
    let mut m = Mismatch {
        count: 0,
        offset: 0,
        expected: 0,
        got: 0,
    };
    for i in first..first + words {
        let expected = pattern(i as u32) ^ seed;
        // SAFETY: as in `fill`.
        let got = unsafe { p.add(i).read_volatile() };
        if got != expected {
            if m.count == 0 {
                (m.offset, m.expected, m.got) = ((i * 4) as u32, expected, got);
            }
            m.count += 1;
        }
    }
    m
}

/// The bytes at the 8-aligned offset `a` of a window filled with `seed`.
fn expected_bytes(a: usize, seed: u32) -> [u8; 8] {
    let lo = (pattern((a / 4) as u32) ^ seed).to_le_bytes();
    let hi = (pattern((a / 4 + 1) as u32) ^ seed).to_le_bytes();
    [lo[0], lo[1], lo[2], lo[3], hi[0], hi[1], hi[2], hi[3]]
}

fn rd8(a: usize) -> u8 {
    // SAFETY: callers pass addresses inside the programmed window.
    unsafe { (a as *const u8).read_volatile() }
}

fn wr8(a: usize, v: u8) {
    // SAFETY: as in `rd8`.
    unsafe { (a as *mut u8).write_volatile(v) }
}

/// LDRH/STRH/LDR/STR at possibly unaligned addresses (the M33 splits them on the bus).
fn rd16(a: usize) -> u16 {
    let v: u32;
    // SAFETY: as in `rd8`; unaligned halfword loads are allowed on normal memory.
    unsafe {
        core::arch::asm!("ldrh {v}, [{a}]", a = in(reg) a, v = out(reg) v, options(nostack, preserves_flags))
    };
    v as u16
}

fn wr16(a: usize, v: u16) {
    // SAFETY: as in `rd16`.
    unsafe {
        core::arch::asm!("strh {v}, [{a}]", a = in(reg) a, v = in(reg) u32::from(v), options(nostack, preserves_flags))
    };
}

fn rd32(a: usize) -> u32 {
    let v: u32;
    // SAFETY: as in `rd16`.
    unsafe {
        core::arch::asm!("ldr {v}, [{a}]", a = in(reg) a, v = out(reg) v, options(nostack, preserves_flags))
    };
    v
}

fn wr32(a: usize, v: u32) {
    // SAFETY: as in `rd16`.
    unsafe {
        core::arch::asm!("str {v}, [{a}]", a = in(reg) a, v = in(reg) v, options(nostack, preserves_flags))
    };
}

/// Guard bytes around boundary address `a`, then whether only `len` bytes at `o` changed to `v`.
fn guard(a: usize) {
    for g in a - 4..a + 5 {
        wr8(g, 0x77);
    }
}

fn guards_ok(a: usize, o: usize, len: usize, v: u32) -> bool {
    (a - 4..a + 5).all(|g| {
        let expected = if (o..o + len).contains(&g) {
            (v >> (8 * (g - o))) as u8
        } else {
            0x77
        };
        rd8(g) == expected
    })
}

/// Failing byte, 16-bit and 32-bit cases around the boundary at offset `b`.
fn boundary(base: usize, b: usize) -> [u32; 3] {
    let a = base + b;
    let mut bad = [0u32; 3];
    for k in 0..6 {
        wr8(a - 3 + k, 0x11 * (k as u8 + 1));
    }
    for k in 0..6 {
        if rd8(a - 3 + k) != 0x11 * (k as u8 + 1) {
            bad[0] += 1;
        }
    }
    // Ends at the boundary, straddles it, starts at it.
    for (k, o) in [a - 2, a - 1, a].into_iter().enumerate() {
        let v = 0xA55Au16.rotate_left(3 * k as u32);
        guard(a);
        wr16(o, v);
        if rd16(o) != v || !guards_ok(a, o, 2, u32::from(v)) {
            bad[1] += 1;
        }
    }
    for (k, o) in [a - 3, a - 2, a - 1, a].into_iter().enumerate() {
        let v = 0xA1B2_C3D4u32.rotate_left(5 * k as u32);
        guard(a);
        wr32(o, v);
        if rd32(o) != v || !guards_ok(a, o, 4, v) {
            bad[2] += 1;
        }
    }
    bad
}

fn m1_regs() -> [u32; 5] {
    // SAFETY: reads of QMI M1_TIMING, M1_RFMT, M1_RCMD, M1_WFMT, M1_WCMD.
    core::array::from_fn(|k| unsafe { ((QMI_BASE + 0x20 + 4 * k) as *const u32).read_volatile() })
}

fn reg_list(r: &WindowRegs) -> [u32; 5] {
    [r.timing, r.rfmt, r.rcmd, r.wfmt, r.wcmd]
}

fn writable() -> bool {
    // SAFETY: read of XIP_CTRL.CTRL.
    unsafe { XIP_CTRL.read_volatile() & WRITABLE_M1 != 0 }
}

/// What a QMI reset does to window 1, as programming the flash through the boot ROM would: the M1
/// registers to their reset values and the window read-only. Runs from RAM with the QMI idle.
///
/// # Safety
///
/// Call in a critical section with nothing else using the XIP window.
#[unsafe(link_section = ".data.ram_func")]
#[inline(never)]
unsafe fn reset_m1_registers() -> bool {
    const T: u32 = WindowRegs::RESET.timing;
    const RF: u32 = WindowRegs::RESET.rfmt;
    const RC: u32 = WindowRegs::RESET.rcmd;
    const WF: u32 = WindowRegs::RESET.wfmt;
    const WC: u32 = WindowRegs::RESET.wcmd;
    let csr = QMI_BASE as *mut u32;
    let mut ok = true;
    // SAFETY: QMI and XIP_CTRL register accesses.
    unsafe {
        let saved = csr.read_volatile();
        csr.write_volatile(saved | 1);
        let mut spin = 100_000u32;
        while csr.read_volatile() & 2 != 0 {
            if spin == 0 {
                ok = false;
                break;
            }
            spin -= 1;
        }
        let m1 = (QMI_BASE + 0x20) as *mut u32;
        m1.write_volatile(T);
        m1.add(1).write_volatile(RF);
        m1.add(2).write_volatile(RC);
        m1.add(3).write_volatile(WF);
        m1.add(4).write_volatile(WC);
        XIP_CTRL.write_volatile(XIP_CTRL.read_volatile() & !WRITABLE_M1);
        csr.write_volatile(saved & !1);
        core::arch::asm!("dsb", "isb", options(nostack, preserves_flags));
    }
    ok
}

/// Cleans every XIP cache line through the top 16 kB of the maintenance window (RP2350-E11
/// workaround), then invalidates every line. For the test's own use of the cached alias.
fn xip_clean_invalidate() {
    for line in 0..2048 {
        // SAFETY: byte writes to the cache maintenance window only perform the operation.
        unsafe { ((0x1bff_c001 + 8 * line) as *mut u8).write_volatile(0) };
    }
    for line in 0..2048 {
        // SAFETY: as above.
        unsafe { ((0x1800_0000 + 8 * line) as *mut u8).write_volatile(0) };
    }
    cortex_m::asm::dsb();
}

/// Reads 8 bytes at `a` straight from the chip: QPI EBh, address, wait cycles, data.
fn direct_read(
    bus: &mut Timed<Rp2350Qmi>,
    clock: DirectClock,
    a: usize,
) -> Result<[u8; 8], BusError> {
    const WAIT: usize = CHIP.quad_read_wait_cycles as usize / 2;
    let mut buf = [0u8; WAIT + 8];
    let cmd = [opcode::QUAD_READ, (a >> 16) as u8, (a >> 8) as u8, a as u8];
    bus.transfer(clock, Width::Quad, &cmd, &mut buf)?;
    let mut out = [0u8; 8];
    out.copy_from_slice(&buf[WAIT..]);
    Ok(out)
}

/// MiB/s x 100 for `bytes` in `us`.
fn rate(bytes: usize, us: u64) -> u32 {
    (bytes as u64 * 100 * 1_000_000 / (us.max(1) * MIB as u64)) as u32
}

fn lcg(a: u32) -> u32 {
    a.wrapping_mul(1_664_525).wrapping_add(1_013_904_223)
}

/// `n` single-word reads at pseudo-random addresses of the window at `base`; nanoseconds each.
#[inline(never)]
fn random_reads(base: usize, n: u32, clk: u32) -> u32 {
    let p = base as *const u32;
    let (mut a, mut x) = (1u32, 0u32);
    let t0 = cycles();
    for _ in 0..n {
        a = lcg(a);
        // SAFETY: the index is masked to the 16 MiB window.
        x ^= unsafe { p.add((a >> 8) as usize & (SIZE / 4 - 1)).read_volatile() };
    }
    let dt = cycles().wrapping_sub(t0);
    black_box(x);
    ns(u64::from(dt), clk) / n
}

#[inline(never)]
fn random_writes(base: usize, n: u32, clk: u32) -> u32 {
    let p = base as *mut u32;
    let mut a = 7u32;
    let t0 = cycles();
    for _ in 0..n {
        a = lcg(a);
        // SAFETY: as in `random_reads`.
        unsafe { p.add((a >> 8) as usize & (SIZE / 4 - 1)).write_volatile(a) };
    }
    ns(u64::from(cycles().wrapping_sub(t0)), clk) / n
}

/// Sequential write and read of `words` words at word `first`: (write us, read us, mismatches).
fn sequential(base: usize, first: usize, words: usize, seed: u32) -> (u64, u64, u32) {
    let t0 = Instant::now();
    fill(base, first, words, seed);
    let w = t0.elapsed().as_micros();
    let t0 = Instant::now();
    black_box(read_xor(base, first, words));
    let r = t0.elapsed().as_micros();
    (w, r, verify(base, first, words, seed).count)
}

fn pass_window(mask: u8) -> (u8, u8) {
    if mask == 0 {
        return (0, 0);
    }
    (mask.trailing_zeros() as u8, 7 - mask.leading_zeros() as u8)
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut failures = 0u32;
    // QMI CS1 and GPIO0 belong to the driver from here on.
    let _qmi_cs1 = p.QMI_CS1;
    let _cs1_pin = p.PIN_0;
    let clk = embassy_rp::clocks::clk_sys_freq();
    defmt::info!(
        "[INFO] psram_smoke: {=str} on QMI CS1 = GPIO0, clk_sys {} Hz",
        CHIP.name,
        clk
    );

    // SAFETY: the debug block and the NVIC priority of a software interrupt nothing else uses.
    unsafe {
        let mut core = cortex_m::Peripherals::steal();
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();
        core.NVIC.set_priority(interrupt::SWI_IRQ_1, TICK_PRIORITY);
    }
    let tick_spawner = TICK_EXECUTOR.start(interrupt::SWI_IRQ_1);
    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);
    let mut ce = CeMonitor::new(&mut common, sm0, clk);

    // SAFETY: QMI CS1 and GPIO0 are held unused above, core 1 is not started, no DMA reads XIP.
    let qmi = unsafe { Rp2350Qmi::new(Cs1Pin::Gpio0) };
    let mut psram = Psram::new(Timed(qmi));

    // P1: first init, with a timer interrupt firing every 20 us.
    TICK_RUN.store(true, Relaxed);
    tick_spawner.spawn(ticker().unwrap());
    Timer::after_millis(2).await;
    let ticks0 = TICKS.load(Relaxed);
    ce.drain();
    let r1 = psram.init(&mut Delay, clk);
    let ticks = TICKS.load(Relaxed) - ticks0;
    TICK_RUN.store(false, Relaxed);
    let (pulses1, ce1) = ce.drain();
    let info = match r1 {
        Ok(info) => info,
        Err(e) => {
            defmt::error!("[FAIL] P1 init: {} (stopping)", e);
            hil_common::exit(failures + 1)
        }
    };
    let t = psram.timing().unwrap();
    let regs = t.registers(&CHIP);
    let regs_ok = m1_regs() == reg_list(&regs) && writable();
    check!(
        failures,
        info.id.manufacturer == 0x9D
            && info.id.kgd == 0x5D
            && info.id.density_mbit() == Some(128)
            && info.size == SIZE
            && regs_ok
            && ticks > 0,
        "{=str} P1 init: ID {=u8:02x} {=u8:02x} (expect 9D 5D), density {} Mbit (expect 128), size {} bytes; M1 registers and WRITABLE_M1 as programmed: {}; {} timer interrupts served during init (expect > 0)",
        info.id.manufacturer,
        info.id.kgd,
        info.id.density_mbit(),
        info.size,
        regs_ok,
        ticks
    );
    defmt::info!(
        "[INFO] P1 timing: SCK {} Hz (CLKDIV {}), RXDELAY {}, COOLDOWN {}, PAGEBREAK {}, SELECT_SETUP {}, SELECT_HOLD {}, MAX_SELECT {} ({} ns), MIN_DESELECT {} (CE# high >= {} ns), CE# low bound {} ns; direct mode CLKDIV {} RXDELAY {}",
        t.sck_hz(),
        t.clkdiv,
        t.rx_delay,
        t.cooldown,
        t.page_break,
        t.select_setup,
        t.select_hold,
        t.max_select,
        t.max_select_ns(),
        t.min_deselect,
        t.deselect_ns(),
        t.select_low_max_ns(),
        t.direct.clkdiv,
        t.direct.rx_delay
    );
    defmt::info!(
        "[INFO] P1 M1 registers: TIMING {=u32:#010x} RFMT {=u32:#010x} RCMD {=u32:#06x} WFMT {=u32:#010x} WCMD {=u32:#06x}; window handed out at {=usize:#010x}",
        regs.timing,
        regs.rfmt,
        regs.rcmd,
        regs.wfmt,
        regs.wcmd,
        psram.base() as usize
    );

    // P2: init again, the chip now in QPI mode.
    reset_stats();
    ce.drain();
    systick_arm();
    let r2 = psram.init(&mut Delay, clk);
    let init_gap = systick_disarm();
    let init_cs = TRANSFER_MAX.load(Relaxed).max(WINDOW_MAX.load(Relaxed));
    let init_calls = (TRANSFERS.load(Relaxed), WINDOWS.load(Relaxed));
    let (pulses2, ce2) = ce.drain();
    check!(
        failures,
        r2 == Ok(info),
        "{=str} P2 second init (chip in QPI mode): {}",
        r2
    );

    // P3: the whole 16 MiB, pattern and inverse, then access widths at the boundaries.
    let base = psram.as_mut_slice().as_mut_ptr() as usize;
    let words = SIZE / 4;
    let t0 = Instant::now();
    fill(base, 0, words, 0);
    let fill_us = t0.elapsed().as_micros();
    let m = verify(base, 0, words, 0);
    fill(base, 0, words, !0);
    let m_inv = verify(base, 0, words, !0);
    check!(
        failures,
        m.count == 0 && m_inv.count == 0,
        "{=str} P3 16 MiB address-derived pattern: {}; inverse: {} (16 MiB written in {} ms)",
        m,
        m_inv,
        fill_us / 1000
    );
    let page = boundary(base, PAGE);
    let die = boundary(base, DIE);
    // Put the inverse pattern back where the boundary checks wrote.
    fill(base, (PAGE - 8) / 4, 4, !0);
    fill(base, (DIE - 8) / 4, 4, !0);
    check!(
        failures,
        page == [0; 3] && die == [0; 3],
        "{=str} P3 byte (6), 16-bit (3) and unaligned 32-bit (4) accesses with guard bytes, failing cases: 1024-byte page boundary {}, 8 MiB die boundary {}",
        page,
        die
    );

    // P4: the same compare finds one injected bit error.
    let victim = DIE / 4 + 3;
    let w = base as *mut u32;
    // SAFETY: inside the window.
    let saved = unsafe { w.add(victim).read_volatile() };
    unsafe { w.add(victim).write_volatile(saved ^ (1 << 16)) };
    let hit = verify(base, victim - 3, 16, !0);
    unsafe { w.add(victim).write_volatile(saved) };
    let clean = verify(base, victim - 3, 16, !0);
    check!(
        failures,
        hit.count == 1
            && hit.offset as usize == victim * 4
            && hit.got == hit.expected ^ (1 << 16)
            && clean.count == 0,
        "{=str} P4 injected bit 16 error at offset {=usize:#x}: compare reports {}; after restoring {}",
        victim * 4,
        hit,
        clean
    );

    // P5: retention.
    Timer::after_secs(3).await;
    let m = verify(base, 0, words, !0);
    check!(
        failures,
        m.count == 0,
        "{=str} P5 retention: 16 MiB verified after 3 s without access: {}",
        m
    );

    // P6: read back in direct mode what went in through the window.
    let clock = t.direct;
    let mut bus = psram.release();
    let handed_out = bus.window_base();
    if handed_out == CACHED_BASE {
        bus.flush_cache();
    }
    ce.drain();
    let addrs = [
        0,
        PAGE - 8,
        PAGE,
        DIE - 8,
        DIE,
        SIZE - 8,
        0x0012_3458,
        0x00AB_CDE0,
    ];
    let (mut differ, mut control, mut bus_error) = (0u32, 0u32, None);
    for a in addrs {
        match direct_read(&mut bus, clock, a) {
            Ok(got) => {
                differ += u32::from(got != expected_bytes(a, !0));
                control += u32::from(got == expected_bytes(a, 0));
            }
            Err(e) => bus_error = Some(e),
        }
    }
    let (pulses6, ce6) = ce.drain();
    check!(
        failures,
        differ == 0 && control == 0 && bus_error.is_none() && pulses6 as usize == addrs.len(),
        "{=str} P6 chip versus window: {} locations (page and die edges, ends) read in direct mode with QPI EBh, {} differ from what was written through the window; the non-inverted pattern matches {} (control, expect 0); bus error {}; {} CE# pulses",
        addrs.len(),
        differ,
        control,
        bus_error,
        pulses6
    );
    let mut psram = Psram::new(bus);
    let r6 = psram.init(&mut Delay, clk);
    let base = psram.as_mut_slice().as_mut_ptr() as usize;
    let kept = verify(base, 0, 64 * 1024 / 4, !0);
    check!(
        failures,
        r6 == Ok(info),
        "{=str} P6 init after release: {}",
        r6
    );
    defmt::info!(
        "[INFO] P6 after that init's software reset, {} of the first 16384 words differ from what was written",
        kept.count
    );

    // P7: RXDELAY sweep at the chosen CLKDIV and the next slower one.
    let region = MIB / 4;
    let n = 256 * 1024 / 4;
    fill(base, region, n, 0);
    let mut masks = [0u8; 2];
    for (k, div) in [t.clkdiv, t.clkdiv + 1].into_iter().enumerate() {
        for rx in 0..=7u8 {
            psram.set_config(Config {
                max_sck_hz: Some(clk / u32::from(div)),
                rx_delay: Some(rx),
            });
            let ok = psram.remap().is_ok()
                && psram.timing().map(|x| x.clkdiv) == Some(div)
                && verify(UNCACHED_BASE, region, n, 0).count == 0;
            masks[k] |= u8::from(ok) << rx;
        }
        let (lo, hi) = pass_window(masks[k]);
        defmt::info!(
            "[INFO] P7 CLKDIV {} ({} Hz): RXDELAY pass mask {=u8:08b} (bit n = RXDELAY n), window {}..{}, centre {}",
            div,
            clk / u32::from(div),
            masks[k],
            lo,
            hi,
            (lo + hi) / 2
        );
    }
    psram.set_config(Config::DEFAULT);
    let restored =
        psram.remap().is_ok() && psram.timing() == Some(t) && verify(base, region, n, 0).count == 0;
    let r = t.rx_delay;
    let both_sides = r > 0 && r < 7 && (masks[0] >> (r - 1)) & 0b111 == 0b111;
    check!(
        failures,
        both_sides && restored,
        "{=str} P7 RXDELAY {} at CLKDIV {} passes with a passing neighbour on each side: {}; default timing restored, 256 KiB verified: {}",
        r,
        t.clkdiv,
        both_sides,
        restored
    );

    // P8: throughput through both aliases.
    let first = 4 * MIB / 4;
    let words_1m = MIB / 4;
    let (uw, ur, um) = sequential(UNCACHED_BASE, first, words_1m, 0x1111_1111);
    let (cw, cr, cm) = sequential(CACHED_BASE, first, words_1m, 0x2222_2222);
    xip_clean_invalidate();
    let landed = verify(UNCACHED_BASE, first, words_1m, 0x2222_2222).count;
    let ur_ns = random_reads(UNCACHED_BASE, 20_000, clk);
    let uw_ns = random_writes(UNCACHED_BASE, 20_000, clk);
    let cr_ns = random_reads(CACHED_BASE, 20_000, clk);
    let cw_ns = random_writes(CACHED_BASE, 20_000, clk);
    xip_clean_invalidate();
    let (a, b, c, d) = (rate(MIB, uw), rate(MIB, ur), rate(MIB, cw), rate(MIB, cr));
    defmt::info!(
        "[STAT] P8 sequential 1 MiB, uncached {=usize:#010x}: write {}.{=u32:02} MiB/s, read {}.{=u32:02} MiB/s; cached {=usize:#010x}: write {}.{=u32:02} MiB/s, read {}.{=u32:02} MiB/s",
        UNCACHED_BASE,
        a / 100,
        a % 100,
        b / 100,
        b % 100,
        CACHED_BASE,
        c / 100,
        c % 100,
        d / 100,
        d % 100
    );
    defmt::info!(
        "[STAT] P8 single 4-byte accesses at random addresses (20000 each, address computation included): uncached read {} ns, write {} ns; cached read {} ns, write {} ns",
        ur_ns,
        uw_ns,
        cr_ns,
        cw_ns
    );
    check!(
        failures,
        um == 0 && cm == 0 && landed == 0,
        "{=str} P8 data of the throughput runs: uncached {} mismatches, cached {}, cached writes read back uncached after a clean {}",
        um,
        cm,
        landed
    );

    // P9: remap after the M1 registers were reset.
    let first = 12 * MIB / 4;
    fill(base, first, n, 0x5A5A_5A5A);
    if base == CACHED_BASE {
        xip_clean_invalidate();
    }
    // SAFETY: interrupts are off and nothing else uses the window.
    let reset_ok = critical_section::with(|_| unsafe { reset_m1_registers() });
    let after_reset = m1_regs();
    let was_writable = writable();
    reset_stats();
    ce.drain();
    systick_arm();
    let r9 = psram.remap();
    let remap_gap = systick_disarm();
    let remap_cs = WINDOW_MAX.load(Relaxed);
    let remap_calls = (TRANSFERS.load(Relaxed), WINDOWS.load(Relaxed));
    let (pulses9, _) = ce.drain();
    let m = verify(base, first, n, 0x5A5A_5A5A);
    let regs_ok = m1_regs() == reg_list(&t.registers(&CHIP)) && writable();
    check!(
        failures,
        reset_ok
            && after_reset == reg_list(&WindowRegs::RESET)
            && !was_writable
            && r9.is_ok()
            && remap_calls == (0, 1)
            && pulses9 == 0
            && m.count == 0
            && regs_ok,
        "{=str} P9 remap: M1 registers at reset values {} and window read-only {}; remap {}, back-end transfers {} (expect 0), CE# pulses {} (expect 0); 256 KiB pattern read back: {}; registers as programmed: {}",
        after_reset == reg_list(&WindowRegs::RESET),
        !was_writable,
        r9,
        remap_calls.0,
        pulses9,
        m,
        regs_ok
    );

    // P10: chip-select low time.
    let burst = 2 * MIB / 4;
    ce.drain();
    black_box(read_xor(base, burst, 4096));
    let (_, ce_read) = ce.drain();
    fill(base, burst, 4096, 0);
    let (_, ce_write) = ce.drain();
    let direct = ce1.max(ce2).max(ce6);
    let tcem = CHIP.tcem_ns;
    let bound = t.select_low_max_ns();
    check!(
        failures,
        pulses1 == 6
            && pulses2 == 6
            && direct <= tcem
            && t.max_select_ns() <= tcem
            && ce_read <= bound
            && ce_write <= bound
            && bound <= tcem,
        "{=str} P10 CE# low: longest direct-mode {} ns (P1 {} ns over {} pulses, P2 {} ns over {}, P6 {} ns); MAX_SELECT x 64 / clk_sys {} ns; window bursts measured {} ns reading, {} ns writing, computed bound {} ns; tCEM {} ns",
        direct,
        ce1,
        pulses1,
        ce2,
        pulses2,
        ce6,
        t.max_select_ns(),
        ce_read,
        ce_write,
        bound,
        tcem
    );

    // P11: interrupts-off time.
    defmt::info!(
        "[STAT] P11 interrupts off (longest SysTick gap, ticks every {} ns): init at most {} ns, remap at most {} ns; longest whole back-end call: init {} ns ({} transfers, {} window writes), remap {} ns",
        ns(u64::from(SYSTICK_PERIOD), clk),
        ns(u64::from(init_gap), clk),
        ns(u64::from(remap_gap), clk),
        ns(u64::from(init_cs), clk),
        init_calls.0,
        init_calls.1,
        ns(u64::from(remap_cs), clk)
    );

    hil_common::exit(failures)
}
