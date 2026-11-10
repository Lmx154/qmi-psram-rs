#![allow(unsafe_code)]

use std::cell::Cell;
use std::rc::Rc;
use std::vec;
use std::vec::Vec;

use embedded_hal::delay::DelayNs;

use crate::*;

const GOOD_ID: [u8; 3] = [0x9D, 0x5D, 0x80];
const MHZ: u32 = 1_000_000;

#[test]
fn id_good_is_128_mbit() {
    let id = Id::from_bytes(GOOD_ID);
    assert_eq!(id.density_code(), 0b100);
    assert_eq!(id.density_mbit(), Some(128));
    assert_eq!(id.check(&Chip::IS66WVS16M8FBLL), Ok(()));
    // Bits 44:40 are reserved and ignored.
    assert_eq!(
        Id::from_bytes([0x9D, 0x5D, 0x9F]).check(&Chip::IS66WVS16M8FBLL),
        Ok(())
    );
}

#[test]
fn id_failed_die() {
    let id = Id::from_bytes([0x9D, 0x55, 0x80]);
    assert_eq!(id.check(&Chip::IS66WVS16M8FBLL), Err(IdError::FailedDie));
    let id = Id::from_bytes([0x9D, 0x5C, 0x80]);
    assert_eq!(
        id.check(&Chip::IS66WVS16M8FBLL),
        Err(IdError::UnknownKgd(0x5C))
    );
}

#[test]
fn id_wrong_manufacturer() {
    // The APS6404L that embassy-rp's psram module expects answers 0x0D 0x5D.
    let id = Id::from_bytes([0x0D, 0x5D, 0x80]);
    assert_eq!(
        id.check(&Chip::IS66WVS16M8FBLL),
        Err(IdError::WrongManufacturer(0x0D))
    );
}

#[test]
fn id_wrong_density() {
    let id = Id::from_bytes([0x9D, 0x5D, 0x60]);
    assert_eq!(id.density_mbit(), Some(64));
    assert_eq!(
        id.check(&Chip::IS66WVS16M8FBLL),
        Err(IdError::WrongDensity(0b011))
    );
    assert_eq!(Id::from_bytes([0x9D, 0x5D, 0xA0]).density_mbit(), None);
}

#[test]
fn id_nothing_answering() {
    for b in [0x00, 0xFF] {
        assert_eq!(
            Id::from_bytes([b; 3]).check(&Chip::IS66WVS16M8FBLL),
            Err(IdError::NoResponse)
        );
    }
}

/// The properties `Timing::compute` promises, checked from the fields alone.
fn assert_limits(t: &Timing, chip: &Chip) {
    let clk = u64::from(t.clk_sys_hz);
    let ps = |cycles: u64| cycles * 1_000_000_000_000 / clk;
    let div = u64::from(t.clkdiv);
    // SCK at most the EBh/38h limit.
    assert!(t.sck_hz() <= chip.max_hz_quad_read && t.sck_hz() <= chip.max_hz_quad_write);
    assert!(u64::from(t.clk_sys_hz) <= div * u64::from(chip.max_hz_quad_read));
    // CE# low bound inside tCEM, and it covers MAX_SELECT plus a full access.
    assert!(u64::from(t.select_low_max_ns()) <= u64::from(chip.tcem_ns));
    assert!(u64::from(t.max_select_ns()) < u64::from(chip.tcem_ns));
    let access = (2 + 6 + u64::from(chip.quad_read_wait_cycles) + 16) * div;
    assert!(
        u64::from(t.select_low_max_cycles) >= u64::from(t.max_select) * 64 + access,
        "{t:?}"
    );
    assert!(t.cooldown == 0 || (1..=63).contains(&t.max_select));
    // CE# high at least one clock period plus 2.1 ns of pad skew.
    let high = div.div_ceil(2) + u64::from(t.min_deselect);
    assert!(ps(high) + 1 >= ps(div) + 2_100, "{t:?}");
    assert!(ps(high) + 1 >= u64::from(chip.min_clock_period_ps) + 2_100);
    // CE# setup and hold with the pad delays.
    let half = div * 1_000_000_000_000 / (2 * clk);
    assert!(half + ps(u64::from(t.select_setup)) >= 4_100 + u64::from(chip.tcsp_ps));
    assert!(half + ps(1 + u64::from(t.select_hold)) >= 2_500 + u64::from(chip.thd_ps));
    // The Read ID in direct mode fits in tCEM with the software allowance.
    assert!(ps(56 * u64::from(t.direct.clkdiv) + 128) <= u64::from(chip.tcem_ns) * 1000);
    assert!(t.direct.rx_delay <= 3);
    assert_eq!(t.page_break, 1024);
}

fn timing(clk: u32) -> Timing {
    Timing::compute(clk, &Chip::IS66WVS16M8FBLL, &Config::DEFAULT).unwrap()
}

#[test]
fn timing_at_150_mhz() {
    let chip = Chip::IS66WVS16M8FBLL;
    let t = timing(150 * MHZ);
    assert_limits(&t, &chip);
    assert_eq!((t.clkdiv, t.sck_hz()), (2, 75 * MHZ));
    assert_eq!(t.rx_delay, 2);
    assert_eq!((t.select_setup, t.select_hold), (1, 0));
    assert_eq!((t.cooldown, t.max_select, t.min_deselect), (1, 7, 2));
    assert_eq!(t.max_select_ns(), 2987);
    assert_eq!(
        t.direct,
        DirectClock {
            clkdiv: 2,
            rx_delay: 2
        }
    );
}

#[test]
fn timing_at_125_mhz() {
    let t = timing(125 * MHZ);
    assert_limits(&t, &Chip::IS66WVS16M8FBLL);
    assert_eq!((t.clkdiv, t.rx_delay, t.select_setup), (2, 2, 0));
    assert_eq!((t.max_select, t.min_deselect), (5, 2));
}

#[test]
fn timing_at_48_mhz() {
    let t = timing(48 * MHZ);
    assert_limits(&t, &Chip::IS66WVS16M8FBLL);
    assert_eq!((t.clkdiv, t.sck_hz(), t.rx_delay), (1, 48 * MHZ, 1));
    assert_eq!((t.cooldown, t.max_select, t.min_deselect), (1, 1, 1));
}

#[test]
fn timing_at_200_mhz() {
    let t = timing(200 * MHZ);
    assert_limits(&t, &Chip::IS66WVS16M8FBLL);
    assert_eq!((t.clkdiv, t.sck_hz(), t.rx_delay), (2, 100 * MHZ, 3));
    assert_eq!((t.select_setup, t.max_select), (1, 10));
}

#[test]
fn timing_limits_hold_from_1_to_400_mhz() {
    let chip = Chip::IS66WVS16M8FBLL;
    let mut valid = 0;
    for clk in (1..=400).map(|m| m * MHZ) {
        match Timing::compute(clk, &chip, &Config::DEFAULT) {
            Ok(t) => {
                assert_limits(&t, &chip);
                valid += 1;
            }
            // Below about 47 MHz the 7-byte Read ID no longer fits in 4 us.
            Err(e) => assert!(
                clk < 47 * MHZ && e == TimingError::ChipSelectLow,
                "{clk} {e:?}"
            ),
        }
    }
    assert!(valid > 350);
}

#[test]
fn timing_cap_and_rx_override() {
    let chip = Chip::IS66WVS16M8FBLL;
    let capped = Config {
        max_sck_hz: Some(50 * MHZ),
        rx_delay: None,
    };
    let t = Timing::compute(150 * MHZ, &chip, &capped).unwrap();
    assert_limits(&t, &chip);
    assert_eq!((t.clkdiv, t.sck_hz()), (3, 50 * MHZ));
    let over = Config {
        max_sck_hz: None,
        rx_delay: Some(6),
    };
    let t = Timing::compute(150 * MHZ, &chip, &over).unwrap();
    assert_eq!((t.rx_delay, t.direct.rx_delay), (6, 3));
    let bad = Config {
        max_sck_hz: None,
        rx_delay: Some(8),
    };
    assert_eq!(
        Timing::compute(150 * MHZ, &chip, &bad),
        Err(TimingError::RxDelay)
    );
}

#[test]
fn timing_errors() {
    let chip = Chip::IS66WVS16M8FBLL;
    assert_eq!(
        Timing::compute(0, &chip, &Config::DEFAULT),
        Err(TimingError::NoClock)
    );
    let tiny = Config {
        max_sck_hz: Some(100_000),
        rx_delay: None,
    };
    assert_eq!(
        Timing::compute(150 * MHZ, &chip, &tiny),
        Err(TimingError::ClockDivider)
    );
    assert_eq!(
        Timing::compute(12 * MHZ, &chip, &Config::DEFAULT),
        Err(TimingError::ChipSelectLow)
    );
}

#[test]
fn register_encoding() {
    let chip = Chip::IS66WVS16M8FBLL;
    // The setting the bring-up test proved: CLKDIV 4, RXDELAY 2, COOLDOWN 1, PAGEBREAK 1024,
    // MAX_SELECT 7, MIN_DESELECT 4.
    let mut t = timing(150 * MHZ);
    t.clkdiv = 4;
    t.select_setup = 0;
    t.min_deselect = 4;
    let regs = t.registers(&chip);
    assert_eq!(regs.timing, 0x600E_4204);
    // Read EBh: prefix, address, suffix, dummy and data quad, 8-bit prefix, 24 dummy bits.
    assert_eq!(regs.rfmt, 0x0006_12AA);
    assert_eq!(regs.rcmd, 0xEB);
    assert_eq!(regs.wfmt, 0x0000_12AA);
    assert_eq!(regs.wcmd, 0x38);
    assert_eq!(regs.atrans, WindowRegs::RESET.atrans);
    assert!(regs.writable);
    // CLKDIV 256 is encoded as 0, SELECT_SETUP and SELECT_HOLD land in bits 25 and 24:23.
    t.clkdiv = 256;
    t.select_setup = 1;
    t.select_hold = 3;
    assert_eq!(t.registers(&chip).timing & 0x0380_00FF, 0x0380_0000);
    assert_eq!(
        DirectClock {
            clkdiv: 4,
            rx_delay: 2
        }
        .csr_bits(),
        0x8100_0000
    );
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Mode {
    Spi,
    Qpi,
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum Event {
    Flush,
    Transfer(Width, Vec<u8>, usize),
    Window(WindowRegs),
}

/// A chip behind the trait: SPI/QPI state machine, reset with recovery time, Read ID.
struct Fake {
    mode: Mode,
    answers: bool,
    id: [u8; 3],
    reset_enabled: bool,
    ready_at_ns: u64,
    now_ns: Rc<Cell<u64>>,
    fail_transfer: Option<usize>,
    transfers: usize,
    events: Vec<Event>,
    clocks: Vec<DirectClock>,
    memory: Vec<u8>,
}

impl Fake {
    fn new(mode: Mode, now_ns: Rc<Cell<u64>>) -> Self {
        Self {
            mode,
            answers: true,
            id: GOOD_ID,
            reset_enabled: false,
            ready_at_ns: 0,
            now_ns,
            fail_transfer: None,
            transfers: 0,
            events: Vec::new(),
            clocks: Vec::new(),
            memory: vec![0xA5; Chip::IS66WVS16M8FBLL.size],
        }
    }

    fn commands(&self) -> Vec<(Width, u8)> {
        self.events
            .iter()
            .filter_map(|e| match e {
                Event::Transfer(w, out, _) => Some((*w, out[0])),
                _ => None,
            })
            .collect()
    }

    fn windows(&self) -> Vec<WindowRegs> {
        self.events
            .iter()
            .filter_map(|e| match e {
                Event::Window(r) => Some(*r),
                _ => None,
            })
            .collect()
    }
}

// SAFETY: `window_base` points at `memory`, a heap buffer of the full chip size that lives as
// long as the fake and that nothing else refers to.
unsafe impl QmiBus for Fake {
    fn transfer(
        &mut self,
        clock: DirectClock,
        width: Width,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), BusError> {
        self.events
            .push(Event::Transfer(width, write.to_vec(), read.len()));
        self.clocks.push(clock);
        self.transfers += 1;
        if self.fail_transfer == Some(self.transfers) {
            return Err(BusError::Timeout);
        }
        // Nothing driving the data lines reads as the pull-up level.
        read.fill(0xFF);
        if !self.answers {
            return Ok(());
        }
        let cmd = write[0];
        let ready = self.now_ns.get() >= self.ready_at_ns;
        let reset_enabled = core::mem::take(&mut self.reset_enabled);
        match (self.mode, width) {
            (Mode::Spi, Width::Single) => match cmd {
                opcode::RESET_ENABLE => self.reset_enabled = true,
                opcode::RESET if reset_enabled => {
                    self.ready_at_ns =
                        self.now_ns.get() + u64::from(Chip::IS66WVS16M8FBLL.reset_us) * 1000;
                }
                opcode::ENTER_QPI if ready => self.mode = Mode::Qpi,
                opcode::READ_ID if ready && write.len() == 4 => {
                    let n = read.len().min(3);
                    read[..n].copy_from_slice(&self.id[..n]);
                }
                _ => {}
            },
            // QPI mode: commands arrive a nibble per clock.
            (Mode::Qpi, Width::Quad) => match cmd {
                opcode::EXIT_QPI => self.mode = Mode::Spi,
                opcode::READ_ID if read.len() >= 6 => read[3..6].copy_from_slice(&self.id),
                _ => {}
            },
            // SPI mode with a quad transaction: the chip sees only the SD0 bits, never a whole
            // command. QPI mode with a single-width transaction: garbage nibbles.
            _ => {}
        }
        Ok(())
    }

    fn program_window(&mut self, regs: &WindowRegs) -> Result<(), BusError> {
        self.events.push(Event::Window(*regs));
        Ok(())
    }

    fn flush_cache(&mut self) {
        self.events.push(Event::Flush);
    }

    fn window_base(&self) -> usize {
        self.memory.as_ptr() as usize
    }
}

struct FakeDelay(Rc<Cell<u64>>);

impl DelayNs for FakeDelay {
    fn delay_ns(&mut self, ns: u32) {
        self.0.set(self.0.get() + u64::from(ns));
    }
}

fn setup(mode: Mode) -> (Psram<Fake>, FakeDelay) {
    let now = Rc::new(Cell::new(0));
    (Psram::new(Fake::new(mode, now.clone())), FakeDelay(now))
}

#[test]
fn init_from_spi_mode() {
    let (mut psram, mut delay) = setup(Mode::Spi);
    assert!(psram.is_empty() && psram.as_slice().is_empty() && psram.base().is_null());
    let info = psram.init(&mut delay, 150 * MHZ).unwrap();
    assert_eq!(info.id, Id::from_bytes(GOOD_ID));
    assert_eq!(info.size, 16 * 1024 * 1024);
    assert_eq!(psram.len(), info.size);
    assert_eq!(psram.as_slice().len(), info.size);
    let timing = psram.timing().unwrap();
    let fake = psram.release();
    assert_eq!(fake.mode, Mode::Qpi);
    assert_eq!(fake.events[0], Event::Flush);
    assert_eq!(
        fake.commands(),
        [
            (Width::Quad, opcode::EXIT_QPI),
            (Width::Single, opcode::RESET_ENABLE),
            (Width::Single, opcode::RESET),
            (Width::Single, opcode::READ_ID),
            (Width::Single, opcode::ENTER_QPI),
            (Width::Quad, opcode::READ_ID),
        ]
    );
    assert_eq!(fake.windows(), [timing.registers(&Chip::IS66WVS16M8FBLL)]);
    assert!(fake.clocks.iter().all(|c| *c == timing.direct));
    // No write command, no memory change.
    assert!(fake.memory.iter().all(|&b| b == 0xA5));
}

#[test]
fn init_from_qpi_mode_and_again() {
    let (mut psram, mut delay) = setup(Mode::Qpi);
    psram.init(&mut delay, 150 * MHZ).unwrap();
    // A second init finds the chip in QPI mode, as after the first.
    psram.init(&mut delay, 150 * MHZ).unwrap();
    let fake = psram.release();
    assert_eq!(fake.mode, Mode::Qpi);
    assert_eq!(fake.windows().len(), 2);
}

#[test]
fn init_waits_for_the_reset() {
    let (mut psram, _) = setup(Mode::Spi);
    // A delay that does not wait: the Read ID arrives before the chip is ready.
    let mut no_wait = FakeDelay(Rc::new(Cell::new(0)));
    assert_eq!(
        psram.init(&mut no_wait, 150 * MHZ),
        Err(Error::Id(IdError::NoResponse))
    );
}

#[test]
fn init_rejects_a_wrong_id() {
    let (mut psram, mut delay) = setup(Mode::Spi);
    psram.qmi.id = [0x0D, 0x5D, 0x80];
    assert_eq!(
        psram.init(&mut delay, 150 * MHZ),
        Err(Error::Id(IdError::WrongManufacturer(0x0D)))
    );
    assert!(psram.as_mut_slice().is_empty());
    let fake = psram.release();
    assert!(fake.windows().is_empty());
    assert_eq!(fake.mode, Mode::Spi);
}

#[test]
fn init_with_no_chip() {
    let (mut psram, mut delay) = setup(Mode::Spi);
    psram.qmi.answers = false;
    assert_eq!(
        psram.init(&mut delay, 150 * MHZ),
        Err(Error::Id(IdError::NoResponse))
    );
    assert!(psram.release().windows().is_empty());
}

#[test]
fn init_bus_and_timing_errors() {
    let (mut psram, mut delay) = setup(Mode::Spi);
    psram.qmi.fail_transfer = Some(4);
    assert_eq!(
        psram.init(&mut delay, 150 * MHZ),
        Err(Error::Bus(BusError::Timeout))
    );
    let (mut psram, mut delay) = setup(Mode::Spi);
    assert_eq!(
        psram.init(&mut delay, 0),
        Err(Error::Timing(TimingError::NoClock))
    );
    // A bad clock is caught before anything is sent.
    assert!(psram.release().events.is_empty());
}

#[test]
fn a_failed_init_after_a_good_one_hands_nothing_out() {
    let (mut psram, mut delay) = setup(Mode::Spi);
    psram.init(&mut delay, 150 * MHZ).unwrap();
    assert_eq!(psram.len(), 16 * 1024 * 1024);
    assert_eq!(
        psram.init(&mut delay, 0),
        Err(Error::Timing(TimingError::NoClock))
    );
    assert!(psram.is_empty() && psram.as_slice().is_empty() && psram.base().is_null());
    assert_eq!(psram.info(), None);
    assert_eq!(psram.remap(), Err(Error::NotInitialized));
}

#[test]
fn remap_sends_no_chip_command() {
    let (mut psram, mut delay) = setup(Mode::Spi);
    assert_eq!(psram.remap(), Err(Error::NotInitialized));
    psram.init(&mut delay, 150 * MHZ).unwrap();
    psram.as_mut_slice()[..4].copy_from_slice(&[1, 2, 3, 4]);
    let before = psram.qmi.events.len();
    psram.remap().unwrap();
    // A new configuration takes effect through remap, still without a chip command.
    psram.set_config(Config {
        max_sck_hz: Some(50 * MHZ),
        rx_delay: Some(3),
    });
    psram.remap().unwrap();
    assert_eq!(
        (
            psram.timing().unwrap().clkdiv,
            psram.timing().unwrap().rx_delay
        ),
        (3, 3)
    );
    assert_eq!(psram.as_slice()[..4], [1, 2, 3, 4]);
    let fake = psram.release();
    let after = &fake.events[before..];
    assert_eq!(after.len(), 2);
    assert!(after.iter().all(|e| matches!(e, Event::Window(_))));
    assert_eq!(fake.windows()[0], fake.windows()[1]);
}

#[test]
fn qpi_id_mismatch_is_an_error() {
    // A chip that answers in SPI mode but whose SD2/SD3 are dead: the QPI ID differs.
    struct DeadUpperLines(Fake);
    // SAFETY: forwards to `Fake`.
    unsafe impl QmiBus for DeadUpperLines {
        fn transfer(
            &mut self,
            clock: DirectClock,
            width: Width,
            write: &[u8],
            read: &mut [u8],
        ) -> Result<(), BusError> {
            self.0.transfer(clock, width, write, read)?;
            if width == Width::Quad {
                read.iter_mut().for_each(|b| *b &= 0x33);
            }
            Ok(())
        }
        fn program_window(&mut self, regs: &WindowRegs) -> Result<(), BusError> {
            self.0.program_window(regs)
        }
        fn flush_cache(&mut self) {
            self.0.flush_cache()
        }
        fn window_base(&self) -> usize {
            self.0.window_base()
        }
    }
    let now = Rc::new(Cell::new(0));
    let mut psram = Psram::new(DeadUpperLines(Fake::new(Mode::Spi, now.clone())));
    assert_eq!(
        psram.init(&mut FakeDelay(now), 150 * MHZ),
        Err(Error::QpiMismatch {
            found: [0x11, 0x11, 0x00]
        })
    );
    assert!(psram.release().0.windows().is_empty());
}
