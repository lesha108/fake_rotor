use core::sync::atomic::{AtomicU32, Ordering};
use stm32f1xx_hal::{
    pac::{interrupt, TIM2, TIM3},
    timer::{CounterMs, Event},
};
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use stm32f1xx_hal::usb::UsbBusType;
use usb_device::{bus::UsbBusAllocator, prelude::*};


// Статические переменные для работы с прерываеним таймера
pub static G_TIM: Mutex<RefCell<Option<CounterMs<TIM2>>>> = Mutex::new(RefCell::new(None));
pub static SEC_COUNT_MOVE: AtomicU32 = AtomicU32::new(0);
pub static SEC_COUNT_LED: AtomicU32 = AtomicU32::new(0);

pub static G_TIM3: Mutex<RefCell<Option<CounterMs<TIM3>>>> = Mutex::new(RefCell::new(None));
pub static COUNT_AZ: AtomicU32 = AtomicU32::new(0);
pub static COUNT_EL: AtomicU32 = AtomicU32::new(0);
pub static SERIAL_W: AtomicU32 = AtomicU32::new(0);

// обаботчик прерывания таймера
#[interrupt]
fn TIM2() {
    static mut TIM: Option<CounterMs<TIM2>> = None;
    let tim = TIM.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move TIM here, leaving a None in its place
            G_TIM.borrow(cs).replace(None).unwrap()
        })
    });
    SEC_COUNT_MOVE.fetch_add(1, Ordering::Relaxed);
    SEC_COUNT_LED.fetch_add(1, Ordering::Relaxed);
    tim.clear_interrupt(Event::Update);
}

#[interrupt]
fn TIM3() {
    static mut TIM3: Option<CounterMs<TIM3>> = None;
    let tim3 = TIM3.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move TIM here, leaving a None in its place
            G_TIM3.borrow(cs).replace(None).unwrap()
        })
    });
    COUNT_AZ.fetch_add(1, Ordering::Relaxed);
    COUNT_EL.fetch_add(1, Ordering::Relaxed);
    SERIAL_W.fetch_add(1, Ordering::Relaxed);
    tim3.clear_interrupt(Event::Update);
}

// переменные и прерывания для работы USB
pub static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
pub static mut USB_SERIAL: Option<usbd_serial::SerialPort<UsbBusType>> = None;
pub static mut USB_DEVICE: Option<UsbDevice<UsbBusType>> = None;

#[interrupt]
fn USB_HP_CAN_TX() {
    usb_interrupt();
}

#[interrupt]
fn USB_LP_CAN_RX0() {
    usb_interrupt();
}

fn usb_interrupt() {
    cortex_m::interrupt::free(|_| {
        let usb_dev = unsafe { USB_DEVICE.as_mut().unwrap() };
        let serial = unsafe { USB_SERIAL.as_mut().unwrap() };
        usb_dev.poll(&mut [serial]);
    });
}
