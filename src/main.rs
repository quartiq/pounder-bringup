#![no_std]
#![no_main]
use stm32h7xx_hal::{
    prelude::*,
    qspi::{Qspi, QspiError, QspiMode},
};

use embedded_hal::digital::v2::OutputPin;

use mcp23017;
use stm32h7xx_hal as hal;

use ad9959;

// pick a panicking behavior
// extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
extern crate cortex_m_semihosting;
extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

use cortex_m_rt::entry;

use cortex_m_semihosting::{dbg, hprintln};

struct QspiInterface {
    qspi: Qspi,
}

// TODO: Refactor this to use the nb interface.
impl ad9959::Interface for QspiInterface {
    type Error = QspiError;

    fn configure_mode(&mut self, mode: ad9959::Mode) -> Result<(), QspiError> {
        match mode {
            ad9959::Mode::FourBitSerial => self.qspi.configure_mode(QspiMode::FourBit),
        }
    }

    fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), QspiError> {
        if (addr & 0x80) != 0 {
            return Err(QspiError::InvalidAddress);
        }

        self.qspi.write(addr, &data)
    }

    fn read(&mut self, addr: u8, mut dest: &mut [u8]) -> Result<(), QspiError> {
        if (addr & 0x80) != 0 {
            return Err(QspiError::InvalidAddress);
        }
        self.qspi.read(0x80_u8 | addr, &mut dest)
    }
}


#[entry]
fn main() -> ! {
    let dp = hal::stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();

    // Configure the clock frequencies of the chip.
    let rcc = dp.RCC.constrain();
    let mut clocks = rcc.sysclk(96.mhz()).freeze(vos, &dp.SYSCFG);

    // Instantiate the QUADSPI pins and peripheral interface.
    let gpioa = dp.GPIOA.split(&mut clocks.ahb4);
    let gpiob = dp.GPIOB.split(&mut clocks.ahb4);
    let gpioc = dp.GPIOC.split(&mut clocks.ahb4);
    let gpiod = dp.GPIOD.split(&mut clocks.ahb4);
    let gpiog = dp.GPIOG.split(&mut clocks.ahb4);
    let gpioe = dp.GPIOE.split(&mut clocks.ahb4);

    // TODO: Place these into a pins structure that is provided to the QSPI constructor.
    let _qspi_clk = gpiob.pb2.into_alternate_af9();
    let _qspi_ncs = gpioc.pc11.into_alternate_af9();
    let _qspi_io0 = gpioe.pe7.into_alternate_af10();
    let _qspi_io1 = gpioe.pe8.into_alternate_af10();
    let _qspi_io2 = gpioe.pe9.into_alternate_af10();
    let _qspi_io3 = gpioe.pe10.into_alternate_af10();

    let mut fp_led_0 = gpiod.pd5.into_push_pull_output();
    let mut fp_led_1 = gpiod.pd6.into_push_pull_output();
    let mut fp_led_2 = gpiod.pd12.into_push_pull_output();
    let mut fp_led_3 = gpiog.pg4.into_push_pull_output();

    fp_led_0.set_low().unwrap();
    fp_led_1.set_low().unwrap();
    fp_led_2.set_low().unwrap();
    fp_led_3.set_low().unwrap();

    let qspi_interface = {
        let qspi = Qspi::new(dp.QUADSPI, &mut clocks, 10.mhz()).unwrap();
        QspiInterface{qspi}
    };

    // Instantiate the IO pins for the AD9959
    let mut reset_pin = gpioa.pa0.into_push_pull_output();
    let io_update_pin = gpiog.pg7.into_push_pull_output();

    let delay = hal::delay::Delay::new(cp.SYST, clocks.clocks);

    let i2c1 = {
        let _sda = gpiob.pb7.into_alternate_af4().set_open_drain();
        let _scl = gpiob.pb8.into_alternate_af4().set_open_drain();
        hal::i2c::I2c::i2c1(dp.I2C1, 100.khz(), &clocks)
    };

    let mut expander = mcp23017::MCP23017::new(i2c1, 0x20).unwrap();
    // ext_clk_sel=1, osc_en_n=1, att_rst_n=0, leds=0x3f
    expander.write_gpioab(0xc03f).unwrap();
    // ext_clk_sel=1, osc_en_n=1, att_rst_n=1, leds=0x3f
    expander.write_gpioab(0xe03f).unwrap();
    for i in 0..16 {
        expander.pin_mode(i, mcp23017::PinMode::OUTPUT).unwrap();
    }
    /*
    // digital_write and all_pin_mode look broken
    expander.digital_write(15, 0).unwrap(); // internal clock
    expander.digital_write(14, 0).unwrap(); // osc enable
    expander.digital_write(13, 1).unwrap(); // no att rst

    // Enable all six pounder LEDs to indicate I2C is functional.
    for i in 0..6 {
        expander.digital_write(i, 1).unwrap();
    }
    expander.all_pin_mode(mcp23017::PinMode::OUTPUT).unwrap();
    */

    let mut ad9959 = ad9959::Ad9959::new(qspi_interface, &mut reset_pin, io_update_pin, delay,
            10_000_000).unwrap();

    // Configure the system clock of the AD9959.
    ad9959.configure_system_clock(500e6).unwrap();

    // Test readback-verification of the AD9959 interface.
    if ad9959.self_test().unwrap() {
        fp_led_1.set_high().unwrap();
        fp_led_2.set_high().unwrap();
        fp_led_3.set_high().unwrap();
    }

    ad9959.set_frequency(ad9959::Channel::One, 30e6).unwrap();
    ad9959.set_frequency(ad9959::Channel::Two, 30.1e6).unwrap();
    ad9959.set_frequency(ad9959::Channel::Three, 7.8125e6).unwrap();
    ad9959.set_frequency(ad9959::Channel::Four, 7.8125e6).unwrap();

    ad9959.set_amplitude(ad9959::Channel::One, 1.).unwrap();
    ad9959.set_amplitude(ad9959::Channel::Two, 1.).unwrap();
    ad9959.set_amplitude(ad9959::Channel::Three, 1.).unwrap();
    ad9959.set_amplitude(ad9959::Channel::Four, 1.).unwrap();

    ad9959.enable_channel(ad9959::Channel::One).unwrap();
    ad9959.enable_channel(ad9959::Channel::Two).unwrap();
    ad9959.enable_channel(ad9959::Channel::Three).unwrap();
    ad9959.enable_channel(ad9959::Channel::Four).unwrap();

    clocks.rb.d2ccip1r.modify(|_, w| w.spi123sel().per());
    let mut spi = {
        let spi_mosi = gpiod.pd7.into_alternate_af5();
        let spi_miso = gpioa.pa6.into_alternate_af5();
        let spi_sck = gpiog.pg11.into_alternate_af5();
        let _spi_nss = gpiog.pg10.into_alternate_af5();

        let config = hal::spi::Config::new(hal::spi::Mode{
                polarity: hal::spi::Polarity::IdleHigh,
                phase: hal::spi::Phase::CaptureOnSecondTransition,
            })
            .frame_size(8);

        dp.SPI1.spi((spi_sck, spi_miso, spi_mosi), config, 10.mhz(), &clocks)
    };

        let mut att = [0x00u8; 1];
        spi.transfer(&mut att).unwrap();
    // latch attenuators
    expander.write_gpioab(0xef3f).unwrap();
    expander.write_gpioab(0xe03f).unwrap();
        //panic!("{}", att[0]);

    loop {
        fp_led_0.set_high().unwrap();
        // TODO: Delay?
        fp_led_0.set_low().unwrap();
    }
}
