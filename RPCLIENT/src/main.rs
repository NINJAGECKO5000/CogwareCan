//! CogwareCan display: subscribes to the gauges it shows, renders them in
//! whatever units it likes, and listens for over-the-air updates.
//!
//! The display never knows which ECU is on the other end. It asks for gauge
//! IDs, reads canonical values back, and converts for presentation. Swapping
//! a Speeduino for a Haltech, a MaxxECU or an OBD2 car changes nothing here.

#![no_std]
#![no_main]

use adafruit_qt_py_rp2040::entry;
use adafruit_qt_py_rp2040::{hal, Pins, XOSC_CRYSTAL_FREQ};
use core::fmt::Write;
use core::iter::once;
use panic_halt as _;

use cogware_can::xfer::{BufferSink, Receiver, RxState};
use cogware_can::subscribe::Subscription;
use cogware_can::{feed_frame, Gauge, Unit};
use cogware_can::{AFR_PRI, BAT_VOL, CLNT, IAT, MAP, MASTERALIVE, RPM, TPS, VSS};
use embedded_graphics::{
    mono_font::{ascii::FONT_5X7, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal::spi::MODE_0;
use fugit::RateExtU32;
use hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::PinState,
    pac,
    pio::PIOExt,
    timer::Timer,
    watchdog::Watchdog,
    Sio, I2C,
    {gpio::FunctionSpi, spi::Spi},
};
use mcp2515::{error::Error, regs::OpMode, CanSpeed, McpSpeed, MCP2515};
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use ws2812_pio::Ws2812;

/// This display's address on the bus, fixed at build time. The master uses it
/// to target over-the-air updates at one node instead of all of them.
const NODE_ADDR: u8 = 0x07;

/// Room to stage an incoming firmware image before it is applied.
const OTA_STAGING: usize = 4096;

/// The gauges this screen shows. Subscribing to only these keeps the bus
/// quiet: the master broadcasts nothing nobody asked for.
const WANTED: [u8; 9] = [
    Gauge::Rpm.id() as u8,
    Gauge::Map.id() as u8,
    Gauge::Clnt.id() as u8,
    Gauge::Iat.id() as u8,
    Gauge::AfrPri.id() as u8,
    Gauge::BatVol.id() as u8,
    Gauge::Tps.id() as u8,
    Gauge::Vss.id() as u8,
    Gauge::Masteralive.id() as u8,
];

/// How long to gather gauge frames before redrawing, in timer ticks (µs).
const FRAME_PERIOD_US: u64 = 100_000;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let i2c = I2C::i2c1(
        pac.I2C1,
        pins.sda1.reconfigure(),
        pins.scl1.reconfigure(),
        400.kHz(),
        &mut pac.RESETS,
        125_000_000.Hz(),
    );
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let sclk = pins.sclk.into_function::<FunctionSpi>();
    let mosi = pins.mosi.into_function::<FunctionSpi>();
    let miso = pins.miso.into_function::<FunctionSpi>();
    let cs = pins.a3.into_push_pull_output();
    let spi = Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sclk)).init(
        &mut pac.RESETS,
        125_000_000u32.Hz(),
        16_000_000u32.Hz(),
        MODE_0,
    );
    let mut can = MCP2515::new(spi, cs);
    can.init(
        &mut timer,
        mcp2515::Settings {
            mode: OpMode::Normal,
            can_speed: CanSpeed::Kbps1000,
            mcp_speed: McpSpeed::MHz16,
            clkout_en: false,
        },
    )
    .unwrap();

    let led = pins.neopixel_data.into_function();
    pins.neopixel_power.into_push_pull_output_in_state(PinState::High);
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(led, &mut pio, sm0, clocks.peripheral_clock.freq(), timer.count_down());

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_5X7)
        .text_color(BinaryColor::On)
        .build();
    let mut text = TextBuf::new();

    // Draw one formatted line at (x, y). Rebuilt each frame into a fixed
    // buffer, so the display needs no allocator.
    macro_rules! row {
        ($x:expr, $y:expr, $($arg:tt)*) => {{
            text.clear();
            let _ = write!(text, $($arg)*);
            let _ = Text::with_baseline(text.as_str(), Point::new($x, $y), text_style, Baseline::Top)
                .draw(&mut display);
        }};
    }
    /// A row showing a gauge value, or "--" while it has never been received.
    macro_rules! gauge_row {
        ($x:expr, $y:expr, $label:literal, $val:expr, $fmt:literal) => {{
            match $val {
                Some(v) => row!($x, $y, concat!($label, ": ", $fmt), v),
                None => row!($x, $y, concat!($label, ": --")),
            }
        }};
    }

    // ---- Subscribe. Ask the master for the gauge IDs this screen shows and
    // keep asking until every one has been acknowledged.
    display.clear(BinaryColor::Off).ok();
    row!(0, 0, "CogwareCan node {}", NODE_ADDR);
    row!(0, 10, "subscribing...");
    display.flush().ok();

    let mut sub = Subscription::new(&WANTED);
    while !sub.is_complete() {
        ws.write(brightness(once(wheel(0)), 32)).unwrap();
        // Ask only for what has not been acknowledged yet, eight IDs a frame.
        for req in sub.request() {
            can.send_message(req).ok();
        }
        // Collect acks for a moment before asking again.
        let deadline = timer.get_counter().ticks().wrapping_add(20_000);
        while timer.get_counter().ticks() < deadline {
            if let Ok(frame) = can.read_message() {
                sub.feed(&frame);
            }
        }
    }
    ws.write(brightness(once(wheel(79)), 32)).unwrap();

    // ---- Over-the-air updates. The master can push a new image to this node
    // at any time; the receiver is a state machine we hand frames to.
    let mut staging = [0u8; OTA_STAGING];
    let mut ota = Receiver::new(NODE_ADDR, BufferSink::new(&mut staging));

    let mut alive: u8 = 0;
    loop {
        // ---- Gather. Every frame is either a gauge, an update frame, or
        // something for another node. None of them can panic the display.
        let deadline = timer.get_counter().ticks().wrapping_add(FRAME_PERIOD_US);
        while timer.get_counter().ticks() < deadline {
            let frame = match can.read_message() {
                Ok(f) => f,
                // Nothing waiting, or a controller error: try again.
                Err(Error::NoMessage) | Err(_) => continue,
            };
            if feed_frame(&frame).is_ok() {
                continue;
            }
            // Not a gauge: it may be part of an update aimed at this node.
            if let Some(reply) = ota.feed(&frame) {
                can.send_message(reply).ok();
            }
        }

        // ---- Render. Gauges hold canonical units; the screen picks its own.
        display.clear(BinaryColor::Off).ok();

        if ota.state() == RxState::Receiving {
            // An update is in flight. Show progress instead of the dashboard.
            let (done, total) = ota.progress();
            row!(0, 10, "FIRMWARE UPDATE");
            row!(0, 25, "node {} receiving", NODE_ADDR);
            row!(0, 40, "{} / {} bytes", done, total);
        } else {
            gauge_row!(0, 0, "RPM", RPM.get(), "{}");
            // MAP is absolute; subtract atmospheric to show boost like a gauge.
            gauge_row!(0, 10, "BST", MAP.to(Unit::PSI).map(|p| p - 14.7), "{:.1}");
            gauge_row!(0, 20, "CLT", CLNT.as_f32(), "{:.0}C");
            gauge_row!(0, 30, "IAT", IAT.as_f32(), "{:.0}C");
            gauge_row!(0, 40, "BAT", BAT_VOL.as_f32(), "{:.1}V");
            gauge_row!(0, 50, "TPS", TPS.as_f32(), "{:.0}%");

            gauge_row!(66, 0, "AFR", AFR_PRI.as_f32(), "{:.1}");
            gauge_row!(66, 10, "LAM", AFR_PRI.reading().and_then(|r| r.lambda()), "{:.2}");
            gauge_row!(66, 20, "MPH", VSS.to(Unit::MPH), "{:.0}");
            gauge_row!(66, 30, "F", CLNT.to(Unit::FAHRENHEIT), "{:.0}");
            // MASTERALIVE counts up while the ECU link is healthy and clears
            // when it drops, so "--" here means the master lost the ECU.
            gauge_row!(66, 40, "SRV", MASTERALIVE.get(), "{}");
            row!(66, 50, "CLI: {}", alive);
        }

        display.flush().ok();
        alive = alive.wrapping_add(1);
        ws.write(brightness(once(wheel(link_colour(alive))), 32)).unwrap();
    }
}

/// Green-ish while the master's heartbeat is running, red when it has stopped.
fn link_colour(alive: u8) -> u8 {
    if MASTERALIVE.is_set() {
        alive
    } else {
        0
    }
}

/// A short line of text built without an allocator. Writes past the end are
/// dropped, which for a 128-pixel display is the right failure.
struct TextBuf {
    buf: [u8; 24],
    len: usize,
}

impl TextBuf {
    const fn new() -> Self {
        TextBuf { buf: [0; 24], len: 0 }
    }

    fn clear(&mut self) {
        self.len = 0;
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
    }
}

impl Write for TextBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for &b in s.as_bytes() {
            if self.len < self.buf.len() {
                self.buf[self.len] = b;
                self.len += 1;
            }
        }
        Ok(())
    }
}

/// Convert a number from `0..=255` to an RGB color triplet.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        (255 - (wheel_pos * 3), 0, wheel_pos * 3).into()
    } else if wheel_pos < 170 {
        wheel_pos -= 85;
        (0, wheel_pos * 3, 255 - (wheel_pos * 3)).into()
    } else {
        wheel_pos -= 170;
        (wheel_pos * 3, 255 - (wheel_pos * 3), 0).into()
    }
}
