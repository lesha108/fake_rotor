use super::*;
use easycom::{AzAngle, ElAngle};
use embedded_hal::digital::v2::OutputPin;

#[derive(Debug, Clone, Copy)]
pub struct AntennaRelays<P1, P2, P3> {
    relay_0: P1,
    relay_1: P2,
    relay_2: P3,
}

impl<P1, P2, P3> AntennaRelays<P1, P2, P3>
where
    P1: OutputPin,
    P2: OutputPin,
    P3: OutputPin,
{
    pub fn new(mut pin_0: P1, mut pin_1: P2, mut pin_2: P3) -> Self {
        pin_0.set_high().ok();
        pin_1.set_high().ok();
        pin_2.set_high().ok();

        AntennaRelays {
            relay_0: pin_0, //in3 - led
            relay_1: pin_1, //in2 - r1
            relay_2: pin_2, //in1 - r2 
        }
    }

    pub fn alloff(&mut self) {
        self.relay_0.set_high().ok();
        self.relay_1.set_high().ok();
        self.relay_2.set_high().ok();
    }

    // switch relays accodingly to az & el angles
    pub fn process_angles(&mut self, aza: &AzAngle, ela: &ElAngle ) {
        let azf: f32 = aza.into();
        let elf: f32 = ela.into();
        // trunc float to int
        let azi: i32 = azf as i32;
        let eli: i32 = elf as i32;

        match eli {
            // low elevation - switch all az to Diamond X200
            0..=3 => {
                self.alloff();
            },
            // high elevation > 3
            // ALL settings here MUST be corrected for actual andennas & relay combinations
            // there must be inclusive range to form 0..360 aperture
            _ => match azi {
              /*  // antenna 2
                0..=119 => {
                    self.relay_0.set_low().ok(); // on
                    self.relay_1.set_high().ok();
                    self.relay_2.set_low().ok();    
                },
                // antenna 4
                120..=239 => {
                    self.relay_0.set_low().ok(); // on
                    self.relay_1.set_low().ok();
                    self.relay_2.set_high().ok();    
                },
                // antenna 3
                240..=360 => {
                    self.relay_0.set_low().ok(); // on
                    self.relay_1.set_high().ok();
                    self.relay_2.set_high().ok();    
                },*/
                // not possible if no error in angles ranges for antennas

                // antenna 2
                0..=80 => {
                    self.relay_0.set_low().ok(); // on
                    self.relay_1.set_low().ok();
                    self.relay_2.set_high().ok();    
                },
                321..=360 => {
                    self.relay_0.set_low().ok(); // on
                    self.relay_1.set_low().ok();
                    self.relay_2.set_high().ok();    
                },
                // antenna 3
                81..=200 => {
                    self.relay_0.set_low().ok(); // on
                    self.relay_1.set_high().ok();
                    self.relay_2.set_high().ok();    
                },
                // antenna 4
                201..=320 => {
                    self.relay_0.set_low().ok(); // on
                    self.relay_1.set_high().ok();
                    self.relay_2.set_low().ok();    
                },
                _ => {
                    self.alloff();
                }
            },
        }
    }
}
