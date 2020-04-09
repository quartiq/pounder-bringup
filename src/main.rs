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
extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
// extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

use cortex_m_rt::entry;

struct QspiInterface {
    qspi: Qspi,
}

// TODO: Refactor this to use the nb interface.
impl ad9959::Interface for QspiInterface {
    type Error = QspiError;

    fn configure_mode(&mut self, mode: ad9959::Mode) -> Result<(), QspiError> {
        match mode {
            ad9959::Mode::SingleBitTwoWire | ad9959::Mode::SingleBitThreeWire =>
                self.qspi.configure_mode(QspiMode::OneBit),
            ad9959::Mode::TwoBitSerial => self.qspi.configure_mode(QspiMode::TwoBit),
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
        let sda = gpiob.pb7.into_alternate_af4();
        let scl = gpiob.pb8.into_alternate_af4();
        hal::i2c::I2c::i2c1(dp.I2C1, (scl, sda), 100.khz(), &clocks)
    };

    let pounder_gpio_expander = mcp23017::MCP23017::new(i2c1, 0x20);

    match pounder_gpio_expander {
        Ok(mut expander) => {
            expander.digital_write(14, 0).unwrap();
            expander.digital_write(15, 0).unwrap();

            // Enable all the pounder LEDs to indicate I2C is functional.
            for i in 0..8 {
                expander.digital_write(i, 1).unwrap();
            }
        },
        Err(_) => {}
    }

    let mut ad9959 = ad9959::Ad9959::new(qspi_interface, &mut reset_pin, io_update_pin, delay,
            ad9959::Mode::FourBitSerial, 100_000_000).unwrap();

    // Configure the system clock of the AD9959.
    ad9959.configure_system_clock(500_000_000_f32).unwrap();

    // Test readback-verification of the AD9959 interface.
    if ad9959.self_test().unwrap() {
        fp_led_1.set_high().unwrap();
        fp_led_2.set_high().unwrap();
        fp_led_3.set_high().unwrap();
    }

    ad9959.set_frequency(ad9959::Channel::One, 80_000_000_f32).unwrap();
    ad9959.set_frequency(ad9959::Channel::Two, 80_001_000_f32).unwrap();
    ad9959.set_frequency(ad9959::Channel::Three, 90_000_000_f32).unwrap();
    ad9959.set_frequency(ad9959::Channel::Four, 90_001_000_f32).unwrap();

    ad9959.enable_channel(ad9959::Channel::One).unwrap();
    ad9959.enable_channel(ad9959::Channel::Two).unwrap();
    ad9959.enable_channel(ad9959::Channel::Three).unwrap();
    ad9959.enable_channel(ad9959::Channel::Four).unwrap();

    loop {
        fp_led_0.set_high().unwrap();
        // TODO: Delay?
        fp_led_0.set_low().unwrap();
    }
}
