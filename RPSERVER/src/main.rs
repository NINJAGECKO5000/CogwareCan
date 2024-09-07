#![no_std]
#![no_main]
#![allow(unused)]
use bank0::*;
extern crate alloc;
use alloc::vec::Vec;
use cogware_can::{server_framegen, speeduino_n_writer, Gauge, MASTERALIVE, STA_TIME};
use core::{cell::Cell, convert::TryInto, default, iter::once, u8};
use embedded_alloc::Heap;
use embedded_hal::spi::MODE_0;
use embedded_hal_0_2::can::{Frame, Id, StandardId};
use fugit::RateExtU32;
use hal::clocks::{init_clocks_and_plls, Clock};
use hal::pio::PIOExt;
use hal::{entry, gpio::*, spi::*, Sio, Timer, Watchdog};
use hal::{
    multicore::{Multicore, Stack},
    pac::{self, SPI1},
    spi::Enabled,
    uart::{DataBits, StopBits, UartConfig, UartPeripheral},
};
use mcp2515::{error::Error, frame::CanFrame, regs::OpMode, CanSpeed, McpSpeed, MCP2515};
use panic_halt as _;
use rp2040_hal as hal;
use rp2040_hal::pac::adc::cs::W;
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GD25Q64CS;
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[global_allocator]
static HEAP: Heap = Heap::empty();

static mut CORE1_STACK: Stack<4096> = Stack::new();

#[rp2040_hal::entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 2048;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

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
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let sclk = pins.gpio14.into_function::<FunctionSpi>();
    let mosi = pins.gpio15.into_function::<FunctionSpi>();
    let miso = pins.gpio8.into_function::<FunctionSpi>();
    let cs = pins.gpio19.into_push_pull_output();
    let spi_dev = pac.SPI1;
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
            mode: OpMode::Normal,
            can_speed: CanSpeed::Kbps1000,
            mcp_speed: McpSpeed::MHz16,
            clkout_en: false,
        },
    )
    .unwrap();

    let led = pins.gpio21.into_function();

    pins.gpio20.into_push_pull_output_in_state(PinState::High);

    // Configure the addressable LED
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        led,
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

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
    // init_gauges();

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        core1_task(can)
    });

    //=======================================^^^ everything above is init=====================
    let mut colorset: u8 = 0;
    let mut byteamount: u8 = 0;
    ws.write(brightness(once(wheel(0)), 32)).unwrap(); //master waiting

    loop {
        //======================================CORE 0 LOOP=======================================
        let mut buf: [u8; 126] = [0u8; 126];
        uart.write_full_blocking(b"n");
        while uart.uart_is_busy() {
            match uart.read_full_blocking(&mut buf) {
                Ok(()) => {}
                Err(e) => {
                    MASTERALIVE.set(0);
                }
            }
        }
        speeduino_n_writer(buf); // main writer for speeduino "n" command
        colorset = colorset.wrapping_add(1);
        MASTERALIVE.set(colorset as u32);
        ws.write(brightness(once(wheel(colorset)), 32)).unwrap();
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
fn core1_task(
    //==============================CORE 1 CAN OUT LOOP==============================================
    mut can: MCP2515<
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
    >,
) {
    let mut cliaskedaddr = Vec::new();
    let masterack = Id::Standard(StandardId::ZERO);
    let clirequest = Id::Standard(StandardId::new(0x15).expect("bad address"));
    loop {
        //let timeout = timer.get_counter().ticks() + 15_000;

        match can.read_message() {
            Ok(frame) => {
                if frame.id() == clirequest {
                    let ackcode = frame.data()[0];
                    for i in frame.data() {
                        if !cliaskedaddr.contains(i) {
                            cliaskedaddr.push(*i);
                        }
                    }
                    let frame = CanFrame::new(
                        masterack,
                        &[ackcode], //tell cli we ack
                    )
                    .unwrap();
                    can.send_message(frame).ok();
                    continue;
                }
            }
            Err(Error::NoMessage) => {}
            Err(Error::TxBusy) => {
                continue;
            }
            Err(_) => {}
        }
        if cliaskedaddr.is_empty() {
            continue;
        }
        for i in &cliaskedaddr {
            match Gauge::from_repr(*i as _) {
                Some(gauge) => match gauge.to_frame() {
                    Some(frame) => can.send_message(frame).ok(),
                    None => None,
                },
                None => None,
            };
        }
    }
}
