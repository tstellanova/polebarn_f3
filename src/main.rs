/*
Copyright (c) 2020, Todd Stellanova
All rights reserved.
License: BSD 3-Clause License (see LICENSE file)
*/

// #![deny(warnings)]
#![no_main]
#![no_std]

use panic_rtt_core::{self, rprint, rprintln, rtt_init_print};

// use core::cell::RefCell;
// use cortex_m::interrupt::{self, Mutex};

use p_hal::stm32 as pac;
use stm32f303cct6_robotdyn_bsp as bsp;
use stm32f3xx_hal as p_hal;

use cortex_m_rt::{entry, exception, ExceptionFrame};

use freertos_sys::cmsis_rtos2;
use p_hal::hal::digital::v2::ToggleableOutputPin;
type MsgBufType = *const cty::c_void;

#[allow(non_upper_case_globals)]
#[no_mangle]
pub static SystemCoreClock: u32 = 64_000_000;

use core::sync::atomic::{AtomicPtr, AtomicUsize, Ordering};

use bsp::peripherals::{self, UserLed1Type};
use cmsis_rtos2::{
    osMessageQueueId_t,
    osPriority_t,
    // osPriority_t_osPriorityLow,
    osPriority_t_osPriorityNormal,
    // osPriority_t_osPriorityNormal1,
    // osPriority_t_osPriorityNormal2
    osThreadAttr_t,
};
use core::ptr::{null, null_mut};
use p_hal::prelude::*;
use ublox_core::SerialInterface;

static GLOBAL_QUEUE_HANDLE: AtomicPtr<osMessageQueueId_t> = AtomicPtr::new(core::ptr::null_mut());
static UPDATE_COUNT: AtomicUsize = AtomicUsize::new(0);
static LAST_ITOW: AtomicUsize = AtomicUsize::new(0);
static USER_LED1: AtomicPtr<UserLed1Type> = AtomicPtr::new(core::ptr::null_mut());
// static USART1_RX: Mutex<RefCell<Option<Usart1RxType>>> = Mutex::new(RefCell::new(None));
// static UBLOX_DRIVER: Mutex<RefCell<Option<UbloxDriverType>>> = Mutex::new(RefCell::new(None));
static UBX_DRIVER: AtomicPtr<UbloxDriverType> = AtomicPtr::new(core::ptr::null_mut());

// cortex-m-rt calls this for serious faults.  can set a breakpoint to debug
#[exception]
fn HardFault(_ef: &ExceptionFrame) -> ! {
    rprintln!("HardFault");
    loop {}
}

#[exception]
fn DefaultHandler(val: i16) -> ! {
    rprintln!("DefaultHandler {}", val);
    loop {}
}

/// Called when FreeRTOS assert fails
#[no_mangle]
extern "C" fn handle_assert_failed() {
    rprintln!("handle_assert_failed");
}

/// Toggle the user leds from their prior state
fn toggle_leds() {
    unsafe {
        USER_LED1
            .load(Ordering::Relaxed)
            .as_mut()
            .unwrap()
            .toggle()
            .unwrap();
    }
}

/// main body of Task 1
/// The GPS input processing task
fn task1_body() {
    let ubx = unsafe { UBX_DRIVER.load(Ordering::Relaxed).as_mut().unwrap() };

    if let Ok(msg_count) = ubx.handle_one_message() {
        if msg_count > 0 {
            if let Some(pvt) = ubx.take_last_nav_pvt() {
                let send_buf = pvt.itow.to_le_bytes();
                let mq = GLOBAL_QUEUE_HANDLE.load(Ordering::Relaxed) as osMessageQueueId_t;
                let _rc =
                    cmsis_rtos2::rtos_os_msg_queue_put(mq, send_buf.as_ptr() as MsgBufType, 1, 10);
            }
        }
    }
}

/// RTOS calls this function to start Task 1
#[no_mangle]
extern "C" fn task1_start(_arg: *mut cty::c_void) {
    loop {
        task1_body();
        cmsis_rtos2::rtos_os_thread_yield();
    }
}

/// main body of Task 2
fn task2_body() -> i32 {
    let mut recv_buf: [u8; 4] = [0; 4];
    let rc = cmsis_rtos2::rtos_os_msg_queue_get(
        GLOBAL_QUEUE_HANDLE.load(Ordering::Relaxed) as osMessageQueueId_t,
        recv_buf.as_mut_ptr() as *mut cty::c_void,
        null_mut(),
        250,
    );
    if 0 == rc {
        toggle_leds();
        UPDATE_COUNT.fetch_add(1, Ordering::SeqCst);
        let itow = u32::from_le_bytes(recv_buf);
        LAST_ITOW.store(itow as usize, Ordering::SeqCst);
    }

    rc
}

/// RTOS calls this function to run Task 2
#[no_mangle]
extern "C" fn task2_start(_arg: *mut cty::c_void) {
    loop {
        let _ = task2_body();
        // cmsis_rtos2::rtos_os_thread_yield();
        //this delay is not necessary, but it makes the LED blinking more perceptible
        cmsis_rtos2::rtos_os_delay(50);
    }
}

/// Setup all the threads that will run
pub fn setup_threads() {
    rprint!("setup_threads");

    // create a shared msg queue
    let mq = cmsis_rtos2::rtos_os_msg_queue_new(3, 4, null());
    if mq.is_null() {
        rprintln!("rtos_os_msg_queue_new failed");
        return;
    }
    GLOBAL_QUEUE_HANDLE.store(mq as *mut _, Ordering::Relaxed);

    let thread1_attr = thread_attr_with_priority(osPriority_t_osPriorityNormal);
    let thread1_id = cmsis_rtos2::rtos_os_thread_new(Some(task1_start), null_mut(), &thread1_attr);
    if thread1_id.is_null() {
        rprintln!("rtos_os_thread1_new failed!");
        return;
    }

    let thread2_attr = thread_attr_with_priority(osPriority_t_osPriorityNormal);
    let thread2_id = cmsis_rtos2::rtos_os_thread_new(Some(task2_start), null_mut(), &thread2_attr);
    if thread2_id.is_null() {
        rprintln!("rtos_os_thread2_new failed!");
        return;
    }

    rprintln!("...done");
}

/// Convenience function for creating thread attributes
fn thread_attr_with_priority(priority: osPriority_t) -> osThreadAttr_t {
    osThreadAttr_t {
        name: null(),
        attr_bits: 0,
        cb_mem: null_mut(),
        cb_size: 0,
        stack_mem: null_mut(),
        stack_size: 0,
        priority,
        tz_module: 0,
        reserved: 0,
    }
}

/// Setup peripherals
fn setup_peripherals() {
    rprint!("setup_peripherals...");

    let (mut user_led1, _delay_source, _i2c1_port, _spi1_port, _spi_csn, usart1_port) =
        peripherals::setup_peripherals();

    //set initial states of user LEDs
    user_led1.set_high().unwrap();
    //store shared peripherals
    USER_LED1.store(&mut user_led1, Ordering::Relaxed);

    let (_usart1_tx, usart1_rx) = usart1_port.split();
    let mut gps_driver = ublox_core::new_serial_driver(usart1_rx);
    UBX_DRIVER.store(&mut gps_driver, Ordering::Relaxed);

    rprintln!("done!");
}

//ublox-core

fn start_rtos() -> ! {
    rprintln!("setup rtos...");

    let _rc = cmsis_rtos2::rtos_kernel_initialize();
    let _tick_hz = cmsis_rtos2::rtos_kernel_get_tick_freq_hz();
    let _sys_timer_hz = cmsis_rtos2::rtos_kernel_get_sys_timer_freq_hz();
    rprintln!("tick_hz: {} sys_timer_hz: {} ", _tick_hz, _sys_timer_hz);

    setup_threads();

    // this should never return:
    let rc = cmsis_rtos2::rtos_kernel_start();
    rprintln!("kernel exit: {}", rc);

    unreachable!()
}

#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("-- MAIN --");

    setup_peripherals();
    //this should never return:
    start_rtos()
}

pub type Usart1RxType = p_hal::serial::Rx<pac::USART1>;
pub type UbloxDriverType = ublox_core::UbxDriver<SerialInterface<Usart1RxType>>;
