#![no_std]
#![no_main]

mod easycom;
mod interrupts;
mod state;
mod switches;

use panic_halt as _;
//use core::convert::Infallible;
use core::fmt::Write;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::entry;
//use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use heapless::*;
use nb::block;
use stm32f1xx_hal::{
    pac,
    pac::interrupt,
    prelude::*,
    //time::MilliSeconds,
    //pwm::Channel,
    //delay::Delay,
    //i2c::{BlockingI2c, DutyCycle, Mode},
    serial::{Config, Serial},
    //watchdog::IndependentWatchdog,
    time::*,
    //adc,
    //rtc::Rtc,
    timer::{Delay, Event, Timer},
    gpio::PinState,
};

use core::cell::RefCell;
//use cortex_m::asm::{delay, wfi};
use cortex_m::interrupt::Mutex;
use stm32f1xx_hal::pac::{Interrupt, NVIC};
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};


use crate::easycom::*;
use crate::interrupts::*;
use crate::state::*;
use crate::switches::AntennaRelays;

//use cortex_m::asm;
//use cortex_m_rt::ExceptionFrame;  //  Stack frame for exception handling.
//use cortex_m_semihosting::hio;  //  For displaying messages on the debug console.
/*
static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;

static mut USB_SERIAL: Option<usbd_serial::SerialPort<UsbBusType>> = None;
static mut USB_DEVICE: Option<UsbDevice<UsbBusType>> = None;
*/
//static MY_USB_SERIAL: Mutex<RefCell<Option<usbd_serial::SerialPort<UsbBusType>>>> =
//  Mutex::new(RefCell::new(None));

// Определяем входную функцию
#[entry]
fn main() -> ! {
    //Show "Hello, world!" on the debug console, which is shown in OpenOCD
    //let mut debug_out = hio::hstdout().unwrap();
    //writeln!(debug_out, "Hello, world!").unwrap();

    // Получаем управление над аппаратными средствами
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.MHz()) // переключились на кварц
        .sysclk(48.MHz()) //
        .pclk1(24.MHz())
        .freeze(&mut flash.acr);
    // Prepare the alternate function I/O registers
    let mut afio = dp.AFIO.constrain();
    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();
    let mut gpioc = dp.GPIOC.split();

    // bluepill led
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    let mut led2 = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);

    // setup control relay pins
    let mut pin_0 = gpiob.pb12.into_open_drain_output_with_state(&mut gpiob.crh, PinState::High);
    let mut pin_1 = gpiob.pb13.into_open_drain_output_with_state(&mut gpiob.crh, PinState::High);
    let mut pin_2 = gpiob.pb14.into_open_drain_output_with_state(&mut gpiob.crh, PinState::High);

    let mut relays = AntennaRelays::new(pin_0, pin_1, pin_2);
    //relays.alloff();

    // current state of fake rotator
    let mut rotator = RotorState::new();

    // таймер функций задержки
    let mut delay = cp.SYST.delay(&clocks);

    // секундный таймер для обновления состояния реле
    let mut timer = dp.TIM2.counter_ms(&clocks);
    timer.start(1.secs()).unwrap();
    timer.listen(Event::Update);
    cortex_m::interrupt::free(|cs| *G_TIM.borrow(cs).borrow_mut() = Some(timer));
    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
    }
    SEC_COUNT_MOVE.store(0, Ordering::Relaxed);
    SEC_COUNT_LED.store(0, Ordering::Relaxed);

    // 10 ms таймер для эмуляции вращения роторов моторов
    let mut timer3 = dp.TIM3.counter_ms(&clocks);
    timer3.start(10.millis()).unwrap();
    timer3.listen(Event::Update);
    cortex_m::interrupt::free(|cs| *G_TIM3.borrow(cs).borrow_mut() = Some(timer3));
    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM3);
    }
    COUNT_AZ.store(0, Ordering::Relaxed);
    COUNT_EL.store(0, Ordering::Relaxed);

    // USART3 - commands port
    let serial3 = {
        // Configure pb10 as a push_pull output, this will be the tx pin
        let tx = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
        let rx = gpiob.pb11;
        Serial::new(
            dp.USART3,
            (tx, rx),
            &mut afio.mapr,
            Config::default().baudrate(9600.bps()),
            &clocks,
        )
    };
    // Split the serial struct into a receiving and a transmitting part
    let (mut tx, mut rx) = serial3.split();

    let mut write_tx = |to_write: &[u8]| {
        for bytes in to_write {
            block!(tx.write(*bytes)).ok();
        }
    };

        // настройка USB serial
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low();
        delay.delay_ms(100u16);
    
        let usb_dm = gpioa.pa11;
        let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);
    
        let usb = Peripheral {
            usb: dp.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        };
    
        // Unsafe to allow access to static variables
        unsafe {
            let bus = UsbBus::new(usb);
            USB_BUS = Some(bus);
            USB_SERIAL = Some(SerialPort::new(USB_BUS.as_ref().unwrap()));
            let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST")
                .device_class(USB_CLASS_CDC)
                .build();
            USB_DEVICE = Some(usb_dev);
        }
    
        unsafe {
            NVIC::unmask(Interrupt::USB_HP_CAN_TX);
            NVIC::unmask(Interrupt::USB_LP_CAN_RX0);
        }
    

    let mut led_state = false;

    // rcv buffer
    const RCV_BUF_LEN: usize = 128;
    let mut rcv_buf: Vec<u8, RCV_BUF_LEN> = Vec::new();
    let mut rcv_count_raw: usize = 0;

    const ERR_ANS: &[u8] = b"?\n";

    loop {
        //wfi();
        //delay.delay_ms(1000_u16);
        // process  input
        rcv_buf.clear();
        rcv_count_raw = 0;
        SERIAL_W.store(0, Ordering::Relaxed); // сбрасываем таймер
        loop {
            let res = rx.read();
            match res {
                Err(nb::Error::Other(_)) => {
                    break;
                }
                Err(nb::Error::WouldBlock) => {
                    let wt = SERIAL_W.load(Ordering::Relaxed);
                    if wt > 1 {
                        // wat for next char 10 ms
                        break;
                    }
                    continue;
                }
                Ok(x) => {
                    if rcv_buf.len() < RCV_BUF_LEN {
                        // защита от переполнения буфера
                        rcv_buf.push(x).unwrap();
                        SERIAL_W.store(0, Ordering::Relaxed);
                    } else {
                        break;
                    }
                }
            }
        }
        rcv_count_raw = rcv_buf.len();
        /*
        if rcv_count != 0 {
            let mut line: Vec<u8, 1000> = Vec::new();
            write!(line, "cnt={} bytes={:?}\n", rcv_count, rcv_buf).unwrap();
            write_tx(line.as_slice());
        }*/

        // Convert command to upper case
        if rcv_count_raw != 0 {
            for c in rcv_buf[0..rcv_count_raw].iter_mut() {
                if 0x61 <= *c && *c <= 0x7a {
                    *c &= !0x20;
                }
            }
        }
write_log(&rcv_buf);
        // strange short input?
        if rcv_count_raw == 1 {
            write_tx(ERR_ANS);
        }
        // at least 1 command supposed to be in buffer
        if rcv_count_raw > 1 {
            let fullslice = &rcv_buf[0..rcv_count_raw]; // make slice from actual number of chars in buffer - expect one or more commands there
                                                    // make iterator of commands - may produce empty subslices
                                                    // chars splitters are ASCII CR, LF, SPACE
            let slice_iter = fullslice.split(|num| num == &10 || num == &13 || num == &32);
            for supposed_command in slice_iter {
                //write_tx(supposed_command);
                // command string must be at least 2 chars
                if supposed_command.len() < 2 {
                    continue;
                }
                let rcv_count = supposed_command.len();

                // protocol has 2 letter commands
                let (az, el) = rotator.get_angles();
                let possible_command = EasycomCommands::try_from(&supposed_command[0..2]);
                match possible_command {
                    Ok(cmd) => {
                        match cmd {
                            EasycomCommands::VE => {
                                write_tx(EASYCOM_PROTOCOL_VERSION);
                            }
                            EasycomCommands::MR => {
                                let azf: f32 = az.into();
                                let current_az = (azf * 10.0) as u32;
                                COUNT_AZ.store(current_az, Ordering::Relaxed);
                                rotator.set_state_az(AzStates::MovingRight);
                            }
                            EasycomCommands::ML => {
                                let azf: f32 = az.into();
                                let current_az = ((MAX_AZ - azf) * 10.0) as u32;
                                COUNT_AZ.store(current_az, Ordering::Relaxed);
                                rotator.set_state_az(AzStates::MovingLeft);
                            }
                            EasycomCommands::MU => {
                                let elf: f32 = el.into();
                                let current_el = (elf * 10.0) as u32;
                                COUNT_EL.store(current_el, Ordering::Relaxed);
                                rotator.set_state_el(ElStates::MovingUp);
                            }
                            EasycomCommands::MD => {
                                let elf: f32 = el.into();
                                let current_el = ((MAX_EL - elf) * 10.0) as u32;
                                COUNT_EL.store(current_el, Ordering::Relaxed);
                                rotator.set_state_el(ElStates::MovingDown);
                            }
                            EasycomCommands::SA => {
                                rotator.stop_az();
                            }
                            EasycomCommands::SE => {
                                rotator.stop_el();
                            }
                            EasycomCommands::AZ => {
                                match rcv_count {
                                    // simple query of angle
                                    2 => {
                                        let azf: f32 = az.into();
                                        let mut line: Vec<u8, 50> = Vec::new();
                                        write!(line, "+{0:.1}\n", azf).unwrap();
                                        //write!(line, "+181.1 56.7\n").unwrap();
                                        write_tx(line.as_slice());
                                    } // angle format is X.X, XX.X, XXX.X
                                    5..=7 => {
                                        let azp: AzAngle = Default::default();
                                        match azp.from_degrees(&supposed_command[2..rcv_count]) {
                                            Ok(ang) => {
                                                rotator.set_az(ang);
                                            }
                                            Err(_) => {
                                                write_tx(ERR_ANS);
                                            }
                                        }
                                    }
                                    _ => {
                                        // requested angle not identified
                                        write_tx(ERR_ANS);
                                    }
                                }
                            }
                            EasycomCommands::EL => {
                                match rcv_count {
                                    // simple query of angle
                                    2 => {
                                        let elf: f32 = el.into();
                                        let mut line: Vec<u8, 10> = Vec::new();
                                        write!(line, "+{0:.1}\n", elf).unwrap();
                                        write_tx(line.as_slice());
                                    } // angle format is X.X, XX.X, XXX.X
                                    5..=7 => {
                                        let elp: ElAngle = Default::default();
                                        match elp.from_degrees(&supposed_command[2..rcv_count]) {
                                            Ok(ang) => {
                                                rotator.set_el(ang);
                                            }
                                            Err(_) => {
                                                write_tx(ERR_ANS);
                                            }
                                        }
                                    }
                                    _ => {
                                        // requested angle not identified
                                        write_tx(ERR_ANS);
                                    }
                                }
                            }
                            _ => {
                                write_tx(ERR_ANS); // never
                            }
                        }
                    }
                    // 2 letter command not identified
                    Err(_) => {
                        write_tx(ERR_ANS);
                    }
                }
            }
        }

        // up/down && left/right sync && auto-stop if out of range
        let (azs, els) = rotator.get_states();

        if azs != AzStates::Stopped {
            // for up preload actual ang, for down preload 360 - actual ang
            let newazd = COUNT_AZ.load(Ordering::Relaxed);
            let newazf = if azs == AzStates::MovingRight {
                newazd as f32 / 10.0
            } else {
                MAX_AZ - (newazd as f32 / 10.0)
            };
            match AzAngle::try_from(newazf) {
                Ok(ang) => {
                    rotator.set_az(ang);
                }
                Err(_) => {
                    // suppose timer went ahead of us (((
                    if azs == AzStates::MovingRight {
                        rotator.set_az(AzAngle::try_from(MAX_AZ).unwrap());
                    } else {
                        rotator.set_az(AzAngle::try_from(0.0).unwrap());
                    }
                    rotator.stop_az();
                }
            }
        }

        if els != ElStates::Stopped {
            let neweld = COUNT_EL.load(Ordering::Relaxed);
            let newelf = if els == ElStates::MovingUp {
                neweld as f32 / 10.0
            } else {
                MAX_EL - (neweld as f32 / 10.0)
            };
            match ElAngle::try_from(newelf) {
                Ok(ang) => {
                    rotator.set_el(ang);
                }
                Err(_) => {
                    if els == ElStates::MovingUp {
                        rotator.set_el(ElAngle::try_from(MAX_EL).unwrap());
                    } else {
                        rotator.set_el(ElAngle::try_from(0.0).unwrap());
                    }
                    rotator.stop_el();
                }
            }
        }

        // every 2d second sync relay state with angles
        let cnt_move = SEC_COUNT_MOVE.load(Ordering::Relaxed);
        if cnt_move > 1 {
            let (az, el) = rotator.get_angles();
            relays.process_angles(az, el);
            SEC_COUNT_MOVE.store(0, Ordering::Relaxed);
        }

        // мигаем периодически светодиодом - heartbeat
        let cnt3 = SEC_COUNT_LED.load(Ordering::Relaxed);
        if cnt3 > 1 {
            if led_state {
                led.set_high();
                led2.set_high();
                led_state = false;
            } else {
                led.set_low();
                led2.set_low();
                led_state = true;
            }
            SEC_COUNT_LED.store(0, Ordering::Relaxed); // сбрасываем таймер
        }
    }
}

/*
pub fn write_angle(a: f32) {
    let mut line: Vec<u8, 10> = Vec::new();
    write!(line, "+{0:.1}\n", a).unwrap();
    write_tx(line.as_slice());
}
*/
// глобальная функция логирования в последовательный порт USB
pub fn write_log(to_write: &[u8]) {
    cortex_m::interrupt::free(|_| {
        let serial = unsafe { USB_SERIAL.as_mut().unwrap() };
        serial.write(to_write).ok();
    });
}
