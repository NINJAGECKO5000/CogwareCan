//! Continuously changes the color of the Neopixel on a Adafruit QT Py RP2040 board
#![no_std]
#![no_main]
extern crate alloc;
use adafruit_qt_py_rp2040::entry;
use adafruit_qt_py_rp2040::{hal, Pins, XOSC_CRYSTAL_FREQ};
use alloc::string::String;
use panic_halt as _;

use alloc::format;
use alloc::vec::Vec;
use rp2040_hal::gpio::bank0::*;
use rp2040_hal::gpio::{FunctionSio, Pin, PullDown, SioOutput};
use rp2040_hal::pac::SPI0;
use rp2040_hal::spi::Enabled;
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

use cogware_can::{cli_wri, Gauge, *};
use embedded_hal::spi::MODE_0;
use embedded_hal_0_2::can::{Frame, Id, StandardId};
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
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

#[global_allocator]
static HEAP: Heap = Heap::empty();
static CONFIGGAUGES: [u8; 10] = [0x20, 0x24, 0x25, 0x26, 0x28, 0x29, 0x2D, 0x35, 0x70,0x70];
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
    let mut _timer = timer; // rebind to force a copy of the timer
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_5X7)
        .text_color(BinaryColor::On)
        .build();

    let mut gaugelisten = Vec::new();
    for i in CONFIGGAUGES {
        gaugelisten.push(i);
    }
    display.clear(BinaryColor::Off).ok();
    //let mut can = clirequest(can, gaugelisten.clone());
    let masterack = Id::Standard(StandardId::ZERO);
    let clirequest = Id::Standard(StandardId::new(0x015).expect("bad address"));
    for val in &gaugelisten {
        'read: loop {
            ws.write(brightness(once(wheel(0)), 32)).unwrap();
            match can.read_message() {
                Ok(frame) => {
                    if frame.id() == masterack && frame.data()[0] == *val {
                        ws.write(brightness(once(wheel(79)), 32)).unwrap(); // color to show master acknowledged
                        break 'read;
                    }
                }
                Err(Error::NoMessage) => {}
                Err(_) => {}
            }
            let frame = CanFrame::new(clirequest, &[*val]).unwrap();
            can.send_message(frame).ok();
        }
    }
    ws.write(brightness(once(wheel(180)), 32)).unwrap();
    let mut dispgauge0: String;
    let mut dispgauge1: String;
    let mut dispgauge2: String;
    let mut dispgauge3: String;
    let mut dispgauge4: String;
    let mut dispgauge5: String;
    let mut dispgauge6: String;
    let mut dispgauge7: String;
    let mut dispgauge8: String;
    let mut dispgauge9: String;
    let mut bingus: u8 = 0;
    loop {
        let timeout = timer.get_counter().ticks() + 16_000;
        while timer.get_counter().ticks() <= timeout {
            match can.read_message() {
                Ok(frame) => {
                    // bingles = format!("{:?} {:?}", frame.id(), frame.data());
                    if let Id::Standard(standard_id) = frame.id() {
                        let primitive_id: u16 = standard_id.as_raw();
                        if gaugelisten.contains(&primitive_id.try_into().unwrap()) {
                            cli_wri(frame, primitive_id);
                        }
                    }
                }
                Err(Error::NoMessage) => {}
                Err(_) => panic!("Oh no!"),
            }
        }
        let boost = (MAP.get() as f64 * 0.145038) - 14.5038;
        dispgauge0 = format!("STA: {:?}", STA_TIME.get());
        dispgauge1 = format!("BOOST: {:.1}", boost);
        dispgauge2 = format!("IAT: {:?}", ((IAT.get() * 2) -91));
        dispgauge3 = format!("CLNT: {:?}", ((CLNT.get() * 2) -91));
        dispgauge4 = format!("BATVOL: {:?}", BAT_VOL.get());
        dispgauge5 = format!("AFR: {:?}", (AFR_PRI.get() as f64 / 10.00));
        dispgauge6 = format!("RPM: {:?}", RPM.get());
        dispgauge7 = format!("TPS: {:?}", TPS.get());
        dispgauge8 = format!("CliAlive: {:?}", bingus);
        dispgauge9 = format!("ServAli: {:?}", MASTERALIVE.get());
        bingus = bingus.wrapping_add(1);
        display.clear(BinaryColor::Off).ok();

        Text::with_baseline(&dispgauge0, Point::new(0, 10), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        Text::with_baseline(&dispgauge1, Point::new(0, 20), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        Text::with_baseline(&dispgauge2, Point::new(0, 30), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        Text::with_baseline(&dispgauge3, Point::new(0, 40), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        Text::with_baseline(&dispgauge4, Point::new(0, 50), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        Text::with_baseline(&dispgauge5, Point::new(60, 10), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        Text::with_baseline(&dispgauge6, Point::new(60, 20), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        Text::with_baseline(&dispgauge7, Point::new(60, 30), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();

        Text::with_baseline(&dispgauge8, Point::new(60, 40), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();

        Text::with_baseline(&dispgauge9, Point::new(60, 50), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        display.flush().unwrap();

        ws.write(brightness(
            once(wheel(STA_TIME.get().try_into().unwrap())),
            32,
        ))
        .unwrap();
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
fn clirequest(
    mut can: MCP2515<
        Spi<
            Enabled,
            SPI0,
            (
                Pin<Gpio3, FunctionSpi, PullDown>,
                Pin<Gpio4, FunctionSpi, PullDown>,
                Pin<Gpio6, FunctionSpi, PullDown>,
            ),
        >,
        Pin<Gpio26, FunctionSio<SioOutput>, PullDown>,
    >,
    vecboi: Vec<u8>,
) -> MCP2515<
    Spi<
        Enabled,
        SPI0,
        (
            Pin<Gpio3, FunctionSpi, PullDown>,
            Pin<Gpio4, FunctionSpi, PullDown>,
            Pin<Gpio6, FunctionSpi, PullDown>,
        ),
    >,
    Pin<Gpio26, FunctionSio<SioOutput>, PullDown>,
> {
    let masterack = Id::Standard(StandardId::ZERO);
    let clirequest = Id::Standard(StandardId::new(0x015).expect("bad address"));

    for val in &vecboi {
        'read: loop {
            match can.read_message() {
                Ok(frame) => {
                    if frame.id() == masterack && frame.data()[0] == *val {
                        break 'read;
                    }
                }
                Err(Error::NoMessage) => {}
                Err(_) => panic!("Oh no!"),
            }
            let frame = CanFrame::new(clirequest, &[*val]).unwrap();
            can.send_message(frame).ok();
        }
    }

    can
}