//! Continuously changes the color of the Neopixel on a Adafruit QT Py RP2040 board
#![no_std]
#![no_main]
extern crate alloc;
use adafruit_qt_py_rp2040::entry;
use adafruit_qt_py_rp2040::{hal, Pins, XOSC_CRYSTAL_FREQ};
use alloc::string::String;
use embedded_hal::delay::DelayNs;
use panic_halt as _;

use alloc::format;
use core::iter::once;
use embedded_alloc::Heap;
use embedded_graphics::{
    mono_font::{ascii::FONT_5X7, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::{Baseline, Text},
};
use mcp2515::{error::Error, frame::CanFrame, regs::OpMode, CanSpeed, McpSpeed, MCP2515};
use ws2812_pio::Ws2812;

use embedded_hal::spi::MODE_0;
use embedded_hal_0_2::can::{ExtendedId, Frame, Id, StandardId};
use fugit::RateExtU32;
use hal::rosc::RingOscillator;
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
use rand::distributions::{Distribution, Uniform};
use rand::rngs::SmallRng;
use rand::RngCore;
use rand::{Rng, SeedableRng};

// use smart_leds_trait::SmartLedsWrite;

use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);
    let mut pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
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
    let mut i2c = I2C::i2c1(
        pac.I2C1,
        pins.sda1.reconfigure(), // sda
        pins.scl1.reconfigure(), // scl
        400.kHz(),
        &mut pac.RESETS,
        125_000_000.Hz(),
    );
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    let yoffset = 20;

    let style = PrimitiveStyleBuilder::new()
        .stroke_width(1)
        .stroke_color(BinaryColor::On)
        .build();

    Rectangle::new(Point::new(0, 0), Size::new(127, 63))
        .into_styled(style)
        .draw(&mut display)
        .unwrap();

    display.flush().unwrap();

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    //let mut delay = DelayNs;

    let sclk = pins.sclk.into_function::<FunctionSpi>();
    let mosi = pins.mosi.into_function::<FunctionSpi>();
    let miso = pins.miso.into_function::<FunctionSpi>();
    let cs = pins.a3.into_push_pull_output();
    let spi_dev = pac.SPI0;
    let spi_pin_layout = (mosi, miso, sclk);

    let spi = Spi::<_, _, _, 8>::new(spi_dev, spi_pin_layout).init(
        &mut pac.RESETS,
        125_000_000u32.Hz(),
        16_000_000u32.Hz(),
        MODE_0,
    );
    let mut can = MCP2515::new(spi, cs);
    can.init(
        &mut timer,
        mcp2515::Settings {
            mode: OpMode::Normal,          // Loopback for testing and example
            can_speed: CanSpeed::Kbps1000, // Many options supported.
            mcp_speed: McpSpeed::MHz16,    // Currently 16MHz and 8MHz chips are supported.
            clkout_en: false,
        },
    )
    .unwrap();

    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let led = pins.neopixel_data.into_function();

    pins.neopixel_power
        .into_push_pull_output_in_state(PinState::High);

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        led,
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );
    let mut n: u8 = 128;
    let mut timer = timer; // rebind to force a copy of the timer
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_5X7)
        .text_color(BinaryColor::On)
        .build();
    let mut majic = String::new();
    let mut master_ack: bool = false;
    let mut gauge0 = 0;
    let mut gauge1 = 1;
    let mut gauge2 = 2; // val here would be detirm by config loaded on the device, values point to what gauge values the client wants where, master sends data like that to client
    let mut gauge3 = 3;
    let mut gauge4 = 4;
    let mut gauge5 = 5;
    let mut listenaddr: u8 = 0xFF;

    let mut majic = format!("INITING:");

    display.clear(BinaryColor::Off).ok();
    Text::with_baseline(&majic, Point::new(25, 64), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    display.flush().unwrap();

    let gnr = RingOscillator::new(pac.ROSC).initialize();
    let mut small_rng = SmallRng::from_rng(gnr).expect("dingle");
    let addr_range = 0x01..=0x13;
    let mut raddr: u8 = small_rng.gen_range(addr_range.clone()); //small_rng.gen();

    let rdel: u8 = small_rng.gen_range(1..=100); //small_rng.gen();

    let mut token: u8 = small_rng.gen_range(0..=255); //small_rng.gen();
    let mut node_id = 255;
    timer.delay_ms(rdel.into()); //random delay to try and offset address requests

    while !master_ack {
        // ADDR REQ FN
        match can.read_message() {
            Ok(frame) => {
                let data = frame.data();

                ws.write(brightness(once(wheel(32)), 32)).unwrap();
                if frame.id() == Id::Standard(StandardId::new(0x00).expect("bad address")) {
                    if data[1] == token && data[0] == raddr {
                        ws.write(brightness(once(wheel(200)), 32)).unwrap(); // color to show master acknowledged
                        node_id = raddr;
                        master_ack = true;
                        break;
                    }

                    // master responded to someone else at that address reroll
                    if (data[0] != raddr && data[1] == token)
                        || (data[0] == raddr && data[1] != token)
                    {
                        small_rng.gen_range(addr_range.clone()); //reroll master said its taken
                        continue;
                    }

                    if data[0] != raddr && data[1] != token {
                        let frame = CanFrame::new(
                            Id::Standard(StandardId::new(0x015).expect("bad address")),
                            &[raddr, token],
                        )
                        .unwrap();
                        can.send_message(frame); //master is doing something else try again
                    }
                }
                if frame.id() != Id::Standard(StandardId::new(0x00).expect("bad address")) {
                    let frame = CanFrame::new(
                        Id::Standard(StandardId::new(0x015).expect("bad address")),
                        &[raddr, token],
                    )
                    .unwrap();
                    can.send_message(frame);
                } else {
                }
            }
            Err(Error::NoMessage) => {
                let frame = CanFrame::new(
                    Id::Standard(StandardId::new(0x015).expect("bad address")),
                    &[raddr, token],
                )
                .unwrap();
                can.send_message(frame);
            }
            Err(_) => panic!("Oh no!"),
        }
    }
    let bingus = format!("ADDR: {:?} TOKEN: {:?}", node_id, token);
    master_ack = false;

    ws.write(brightness(once(wheel(32)), 32)).unwrap(); //write NEO red to show we are here and waiting for master response

    while !master_ack {
        //CLI -> MASTER DATA REQUEST FN
        match can.read_message() {
            Ok(frame) => {
                if frame.id() != Id::Standard(StandardId::new(node_id.into()).expect("bad address"))
                {
                    let frame = CanFrame::new(
                        Id::Standard(StandardId::new(0x014).expect("bad address")),
                        &[node_id, gauge0, gauge1, gauge2, gauge3, gauge4, gauge5, 0],
                    )
                    .unwrap();
                    can.send_message(frame);
                } else if frame.id()
                    == Id::Standard(StandardId::new(node_id.into()).expect("bad address"))
                {
                    ws.write(brightness(once(wheel(200)), 32)).unwrap(); // color to show master acknowledged

                    break;
                }
            }
            Err(Error::NoMessage) => {
                let frame = CanFrame::new(
                    Id::Standard(StandardId::new(0x014).expect("bad address")),
                    &[node_id, gauge0, gauge1, gauge2, gauge3, gauge4, gauge5, 0],
                )
                .unwrap();
                can.send_message(frame);
            }
            Err(_) => panic!("Oh no!"),
        }

        //make sure master ack message for guage values
    }

    while !master_ack {
        //master assigning us a listenaddr and what values match what
        ws.write(brightness(once(wheel(64)), 32)).unwrap(); //write NEO random color to show we are here and waiting for master response
        match can.read_message() {
            Ok(frame) => {
                if frame.id() == Id::Standard(StandardId::new(node_id.into()).expect("bad address"))
                {
                    let listenaddr = frame.data()[0];
                    let listenagain = frame.data()[7];
                    let frame = CanFrame::new(
                        Id::Standard(StandardId::new(0x000).expect("bad address")),
                        &[node_id], // tell master we acknowledge
                    )
                    .unwrap();
                    can.send_message(frame);
                    ws.write(brightness(once(wheel(180)), 32)).unwrap(); // color to show master acknowledged
                    break;
                } else {
                }
            }
            Err(Error::NoMessage) => (),
            Err(_) => panic!("Oh no!"),
        }
        //TODO if master has more gauges to add if requested more then 6
    }

    loop {
        match can.read_message() {
            Ok(frame) => {
                if frame.id()
                    == Id::Standard(StandardId::new(listenaddr.into()).expect("bad address"))
                {
                    gauge0 = frame.data()[0];
                    gauge1 = frame.data()[1];
                    gauge2 = frame.data()[2];
                    gauge3 = frame.data()[3];
                    gauge4 = frame.data()[4];
                    gauge5 = frame.data()[5];
                    majic = format!(
                        "val0:{:?} val1:{:?} val2:{:?} val3:{:?} val4:{:?} val5:{:?}",
                        gauge0, gauge1, gauge2, gauge3, gauge4, gauge5
                    );
                }
            }
            Err(Error::NoMessage) => (),
            Err(_) => panic!("Oh no!"),
        }

        display.clear(BinaryColor::Off);
        Text::with_baseline(&majic, Point::new(0, 20), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        Text::with_baseline(&majic, Point::new(-128, 28), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        Text::with_baseline(&bingus, Point::new(24, 36), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        display.flush().unwrap();

        ws.write(brightness(once(wheel(gauge0)), 32)).unwrap();
        //timer.delay_ms(20);
    }
}
/// Convert a number from `0..=255` to an RGB color triplet.
///
/// The colours are a transition from red, to green, to blue and back to red.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        // No green in this sector - red and blue only
        (255 - (wheel_pos * 3), 0, wheel_pos * 3).into()
    } else if wheel_pos < 170 {
        // No red in this sector - green and blue only
        wheel_pos -= 85;
        (0, wheel_pos * 3, 255 - (wheel_pos * 3)).into()
    } else {
        // No blue in this sector - red and green only
        wheel_pos -= 170;
        (wheel_pos * 3, 255 - (wheel_pos * 3), 0).into()
    }
}
