/*
Copyright (c) 2020, Todd Stellanova
All rights reserved.
License: BSD 3-Clause License (see LICENSE file)
*/
#![no_std]
#![no_main]


#[allow(non_upper_case_globals)]
#[no_mangle]
pub static SystemCoreClock: u32 = 64_000_000 ;

// pick a panicking behavior
use panic_rtt_core::{self, rprintln, rtt_init_print};

use cortex_m_rt as rt;
use rt::{entry};
use cortex_m::interrupt::{self, Mutex};
use lazy_static::{lazy_static};

mod peripherals_f303;
use peripherals_f303 as peripherals;
use peripherals::{DelaySourceType, GpioTypeUserLed1, Usart1RxType};

use freertos_sys::cmsis_rtos2;
use cmsis_rtos2::{osThreadId_t};
use core::cell::RefCell;
// use core::ops::DerefMut;
use core::sync::atomic::{AtomicPtr, Ordering};

use embedded_hal::digital::v2::ToggleableOutputPin;
use embedded_hal::blocking::delay::DelayMs;

// shared peripherals
// static USER_LED_1:  Mutex<RefCell< Option< GpioTypeUserLed1 >>> = Mutex::new(RefCell::new(None));
static USART1_RX:  Mutex<RefCell< Option< Usart1RxType >>> = Mutex::new(RefCell::new(None));

lazy_static! {
    static ref THANGO: AtomicPtr<DelaySourceType> = AtomicPtr::default();
    static ref USER_LED1: AtomicPtr<GpioTypeUserLed1> = AtomicPtr::default();
}

// static SHARED_GPIOTE: AtomicUsize = AtomicUsize::new(0);
//            SHARED_GPIOTE.store(0, Ordering::Relaxed);

#[no_mangle]
extern "C" fn handle_assert_failed() {
    rprintln!("handle_assert_failed");
}

#[no_mangle]
extern "C" fn start_default_task(_arg: *mut cty::c_void) {
    rprintln!("Start default task...");

    loop {
        // interrupt::free(|cs| {
        //     if let Some(ref mut led1) = USER_LED_1.borrow(cs).borrow_mut().deref_mut() {
        //         led1.toggle().unwrap();
        //     }
        // });

        unsafe {
            USER_LED1.load(Ordering::Relaxed).as_mut().unwrap().toggle().unwrap();
            THANGO.load(Ordering::Relaxed).as_mut().unwrap().delay_ms(100u8);
        }
    }

}

#[no_mangle]
extern "C" fn start_gps_listener_task(_arg: *mut cty::c_void) {

}


/// Return the default task thread ID
fn configure_tasks() -> osThreadId_t {
    // We don't pass context to the default task here, since that involves problematic
    // casting to/from C void pointers; instead, we use global static context.
    let default_thread_id = cmsis_rtos2::rtos_os_thread_new(
        Some(start_default_task),
        core::ptr::null_mut(),
        core::ptr::null(),
    );

    if default_thread_id.is_null() {
        rprintln!( " default_thread new failed!");
        return core::ptr::null_mut()
    }

    let gps_listener_thread_id = cmsis_rtos2::rtos_os_thread_new(
        Some(start_gps_listener_task),
        core::ptr::null_mut(),
        core::ptr::null(),
    );

    if gps_listener_thread_id.is_null() {
        rprintln!( "gps_listener_thread new failed!");
        return core::ptr::null_mut();
    }

    default_thread_id
}

fn start_rtos() -> osThreadId_t {
    rprintln!( "Setup RTOS...");

    let _rc = cmsis_rtos2::rtos_kernel_initialize();
    rprintln!("kernel_initialize rc: {}", _rc);

    let _tick_hz = cmsis_rtos2::rtos_kernel_get_tick_freq_hz();
    rprintln!( "tick_hz : {}", _tick_hz);

    let _sys_timer_hz = cmsis_rtos2::rtos_kernel_get_sys_timer_freq_hz();
    rprintln!("sys_timer_hz : {}", _sys_timer_hz);

    let default_task_id = configure_tasks();
    if default_task_id.is_null() {
        rprintln!("configure_tasks failed!");
        return default_task_id;
    }

    rprintln!("starting rtos...");
    let _rc = cmsis_rtos2::rtos_kernel_start();
    rprintln!( "kernel_start rc: {}", _rc);

    rprintln!("RTOS done!");

    default_task_id
}

fn setup_peripherals() {

    let (_i2c_port, mut user_led1, usart1_port, mut delay_source) = peripherals::setup();
    let ( _usart1_tx,  usart1_rx)  = usart1_port.split();

    //store shared peripherals
    interrupt::free(|cs| {
        THANGO.store(&mut delay_source, Ordering::Relaxed);
        USER_LED1.store(&mut user_led1, Ordering::Relaxed);

        // USER_LED_1.borrow(cs).replace(Some(user_led1));
        USART1_RX.borrow(cs).replace(Some(usart1_rx));
    });
}

#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("-- > MAIN --");

    setup_peripherals();
    let _default_thread_id = start_rtos();

    loop {
        cmsis_rtos2::rtos_os_thread_yield();
    }

}

// #[entry]
// fn main() -> ! {
//
//     rtt_init_print!(NoBlockTrim);
//     rprintln!("-- > MAIN --");
//
//     let (_i2c_port, mut user_led1, gps1_port, mut delay_source) = peripherals::setup();
//
//     let _ = user_led1.set_low();
//     rprintln!("ready!");
//     delay_source.delay_ms(1u8);
//
//     let ( _gps_tx,  gps_rx)  = gps1_port.split();
//     let mut ublox = ublox_core::new_serial_driver(gps_rx);
//     ublox.setup(&mut delay_source).unwrap();
//
//     rprintln!("loopstart!");
//     loop {
//         // check GNSS
//         if let Ok(msg_count) = ublox.handle_one_message() {
//             //console_print(&mut po_tx, format_args!(">>> msg_count: {} \r\n", msg_count));
//             if msg_count > 0 {
//                 if let Some(nav_pvt) = ublox.take_last_nav_pvt() {
//                     rprintln!(">>> nav_pvt lat, lon: {}, {} \r\n",
//                             nav_pvt.lat,
//                             nav_pvt.lon);
//                 }
//                 if let Some(nav_dop) = ublox.take_last_nav_dop() {
//                     rprintln!(">>> nav_dop {} \r\n", nav_dop.itow);
//                 }
//                 if let Some(mon_hw) = ublox.take_last_mon_hw() {
//                     rprintln!(">>> mon_hw jam: {} \r\n", mon_hw.jam_ind);
//                 }
//             }
//         }
//
//         let _ = user_led1.toggle();
//     }
// }

