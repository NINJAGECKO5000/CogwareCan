// SPDX-License-Identifier: GPL-3.0-only
//! CogwareCan master: a Speeduino on UART becomes gauge frames on the CAN bus.
//!
//! Core 0 polls the ECU and fills the gauge table. Core 1 owns the CAN
//! controller: it answers subscribe requests from displays and broadcasts the
//! gauges they asked for. The two cores share nothing but the gauge table,
//! which is safe to touch from either because every access is inside a
//! critical section.
//!
//! Swapping the ECU means changing one call. For a CAN-broadcast ECU on a
//! second controller, replace the UART block with:
//!
//!     haltech::SOURCE.feed_default(&ecu_frame);
//!
//! and for an unknown ECU, let the registry work it out:
//!
//!     cogware_can::CAN_SOURCES.iter().find_map(|s| s.feed_default(&ecu_frame));

#![no_std]
#![no_main]

use bank0::*;
use cogware_can::{speeduino, subscribe::Publisher, MASTERALIVE};
use core::iter::once;
use embedded_hal::spi::MODE_0;
use fugit::RateExtU32;
use hal::clocks::{init_clocks_and_plls, Clock};
use hal::pio::PIOExt;
use hal::uart::{DataBits, StopBits, UartConfig, UartDevice, UartPeripheral, ValidUartPinout};
use hal::{gpio::*, spi::*, Sio, Timer, Watchdog};
use hal::{
    multicore::{Multicore, Stack},
    pac::{self, SPI1},
};
use mcp2515::{error::Error, regs::OpMode, CanSpeed, McpSpeed, MCP2515};
use panic_halt as _;
use rp2040_hal as hal;
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GD25Q64CS;
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

static mut CORE1_STACK: Stack<4096> = Stack::new();

/// Longest realtime payload the ECU can declare (the length byte is a u8).
const RT_BUF: usize = 256;
/// Give up on a stalled ECU after this long and try again.
const UART_TIMEOUT_US: u64 = 50_000;

#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let mut sio = Sio::new(pac.SIO);
    let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let sclk = pins.gpio14.into_function::<FunctionSpi>();
    let mosi = pins.gpio15.into_function::<FunctionSpi>();
    let miso = pins.gpio8.into_function::<FunctionSpi>();
    let cs = pins.gpio19.into_push_pull_output();
    let spi = Spi::<_, _, _, 8>::new(pac.SPI1, (mosi, miso, sclk)).init(
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

    let led = pins.gpio21.into_function();
    pins.gpio20.into_push_pull_output_in_state(PinState::High);
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(led, &mut pio, sm0, clocks.peripheral_clock.freq(), timer.count_down());

    let uartpins = (
        pins.gpio24.into_function::<FunctionUart>(),
        pins.gpio25.into_function::<FunctionUart>(),
    );
    let uart = UartPeripheral::new(pac.UART1, uartpins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    // SAFETY: core 1 is spawned once and nothing else touches this stack.
    let stack = unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK.mem) };
    let _can_task = core1.spawn(stack, move || can_out(can));

    // ================= core 0: ECU in =================
    let mut heartbeat: u8 = 0;
    let mut body = [0u8; RT_BUF];
    ws.write(brightness(once(wheel(0)), 32)).unwrap();

    loop {
        // Ask for a realtime packet, then read its 3-byte header and exactly
        // the payload that header declares. Reading a fixed count instead
        // leaves the UART FIFO out of step with the ECU on the next round.
        uart.write_full_blocking(b"n");

        let mut header = [0u8; speeduino::N_HEADER_LEN];
        if !read_exact(&uart, &timer, &mut header, UART_TIMEOUT_US) {
            link_lost(&mut ws);
            continue;
        }
        let len = match speeduino::n_payload_len(&header) {
            Ok(n) if n <= RT_BUF => n,
            // Bad echo, wrong format byte, or a longer packet than we buffer.
            _ => {
                link_lost(&mut ws);
                continue;
            }
        };
        if !read_exact(&uart, &timer, &mut body[..len], UART_TIMEOUT_US) {
            link_lost(&mut ws);
            continue;
        }

        // The header is already validated, so hand the payload straight over.
        // Fields past the end of a short packet are left untouched.
        speeduino::apply_realtime(&body[..len]);

        heartbeat = heartbeat.wrapping_add(1);
        MASTERALIVE.set(heartbeat as i32);
        ws.write(brightness(once(wheel(heartbeat)), 32)).unwrap();
    }
}

/// The ECU went quiet: clear the heartbeat so displays can show a fault, and
/// leave the gauges at their last values rather than blanking the screen.
fn link_lost<L: SmartLedsWrite<Color = RGB8>>(ws: &mut L) {
    MASTERALIVE.clear();
    let _ = ws.write(brightness(once(RGB8::new(255, 0, 0)), 32));
}

/// Fill `buf` from the UART, giving up after `timeout_us`. Returns false on
/// timeout, so a dead ECU costs one round instead of hanging the core.
fn read_exact<D: UartDevice, P: ValidUartPinout<D>>(
    uart: &UartPeripheral<hal::uart::Enabled, D, P>,
    timer: &Timer,
    buf: &mut [u8],
    timeout_us: u64,
) -> bool {
    let deadline = timer.get_counter().ticks().wrapping_add(timeout_us);
    let mut got = 0;
    while got < buf.len() {
        if timer.get_counter().ticks() > deadline {
            return false;
        }
        match uart.read_raw(&mut buf[got..]) {
            Ok(n) => got += n,
            Err(nb::Error::WouldBlock) => {}
            // Framing or parity error: the discarded bytes are already lost,
            // so abandon this packet and resynchronise on the next header.
            Err(nb::Error::Other(_)) => return false,
        }
    }
    true
}

type Can = MCP2515<
    Spi<
        Enabled,
        SPI1,
        (
            Pin<Gpio15, FunctionSpi, PullDown>,
            Pin<Gpio8, FunctionSpi, PullDown>,
            Pin<Gpio14, FunctionSpi, PullDown>,
        ),
    >,
    Pin<Gpio19, FunctionSio<SioOutput>, PullDown>,
>;

// ================= core 1: CAN out =================
//
// Everything here except the two driver calls lives in the crate, so a board
// with a different CAN controller reuses the whole loop unchanged.
fn can_out(mut can: Can) -> ! {
    let mut displays = Publisher::new();

    loop {
        // A display asking for gauges is recorded and acknowledged. Anything
        // else on the bus yields no acks and is left alone.
        match can.read_message() {
            Ok(frame) => {
                for ack in displays.feed(&frame) {
                    can.send_message(ack).ok();
                }
                continue;
            }
            Err(Error::TxBusy) => continue,
            Err(_) => {}
        }

        // One round of whatever anyone asked for, lowest ID first. A gauge
        // this ECU never supplies yields no frame, so displays can tell
        // "not supported" from "zero".
        for frame in displays.broadcast() {
            can.send_message(frame).ok();
        }
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
