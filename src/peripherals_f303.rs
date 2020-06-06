/*
Copyright (c) 2020, Todd Stellanova
All rights reserved.
License: BSD 3-Clause License (see LICENSE file)
*/

use stm32f3xx_hal as p_hal;

use p_hal::prelude::*;
use p_hal::stm32 as pac;

use pac::{I2C1, USART1};

use embedded_hal::digital::v2::OutputPin;


use p_hal::time::{Hertz, U32Ext};


pub type ImuI2cPortType = p_hal::i2c::I2c<
    I2C1,
    (
        p_hal::gpio::gpiob::PB8<p_hal::gpio::AF4>,
        p_hal::gpio::gpiob::PB9<p_hal::gpio::AF4>,
    ),
>;

pub type Usart1PortType = p_hal::serial::Serial<
    USART1,
    (
        p_hal::gpio::gpiob::PB6<p_hal::gpio::AF7>, //tx
        p_hal::gpio::gpiob::PB7<p_hal::gpio::AF7>, //rx
    ),
>;

pub type DelaySourceType = p_hal::delay::Delay;


pub type Usart1RxType = p_hal::serial::Rx<pac::USART1>;
// stm32f3xx_hal::serial::Rx<stm32f3::stm32f303::USART1>

pub type GpioTypeUserLed1 =  p_hal::gpio::gpioc::PC13<p_hal::gpio::Output<p_hal::gpio::PushPull>>;


pub(crate) fn setup() -> (
    ImuI2cPortType,
    GpioTypeUserLed1,
    Usart1PortType,
    DelaySourceType,
) {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let i2c_freq: Hertz = 400.khz().into();
    // Set up the system clock
    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();

    // HSI: use default internal oscillator
    //let clocks = rcc.cfgr.freeze(&mut flash.acr);
    // HSE: external crystal oscillator must be connected
    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(64.mhz()) //TODO 72 used to work?
        .pclk1(24.mhz())
        .freeze(&mut flash.acr);

    let delay_source = p_hal::delay::Delay::new(cp.SYST, clocks);

    // let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);

    // stm32f303 robotdyn:
    let mut user_led1 = gpioc
        .pc13
        .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
    user_led1.set_high().unwrap();

    let i2c_port = {
        let scl = gpiob
            .pb8
            .into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper)
            .into_af4(&mut gpiob.moder, &mut gpiob.afrh);

        let sda = gpiob
            .pb9
            .into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper)
            .into_af4(&mut gpiob.moder, &mut gpiob.afrh);

        p_hal::i2c::I2c::i2c1(dp.I2C1, (scl, sda), i2c_freq, clocks, &mut rcc.apb1)
    };

    let usart1_port = {
        let rx = gpiob.pb7.into_af7(&mut gpiob.moder, &mut gpiob.afrl);
        let tx = gpiob.pb6.into_af7(&mut gpiob.moder, &mut gpiob.afrl);
        p_hal::serial::Serial::usart1(dp.USART1, (tx,rx), 9600.bps(), clocks, &mut rcc.apb2)
    };

    (i2c_port, user_led1, usart1_port, delay_source)
}
