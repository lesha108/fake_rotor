use super::*;
use easycom::*;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AzStates {
    Stopped,
    MovingLeft,
    MovingRight,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ElStates {
    Stopped,
    MovingUp,
    MovingDown,
}

pub struct RotorState {
    az_state: AzStates,
    el_state: ElStates,
    az_angle: AzAngle,
    el_angle: ElAngle,
}

impl RotorState {
    pub fn new() -> Self {
        RotorState {
            az_state: AzStates::Stopped,
            el_state: ElStates::Stopped,
            az_angle: Default::default(),
            el_angle: Default::default(),        
        }
    }

    pub fn get_angles(&self) -> (&AzAngle, &ElAngle) {
        (&self.az_angle, &self.el_angle)
    }

    pub fn get_states(&self) -> (AzStates, ElStates) {
        (self.az_state.clone(), self.el_state.clone())
    }

    pub fn stop_az(&mut self) {
        self.az_state = AzStates::Stopped;
    }

    pub fn stop_el(&mut self) {
        self.el_state = ElStates::Stopped;
    }

    pub fn set_az(&mut self, ang: AzAngle) {
        self.az_angle = ang;
    }

    pub fn set_el(&mut self, ang: ElAngle) {
        self.el_angle = ang;
    }

    pub fn set_state_az(&mut self, ns: AzStates) {
        self.az_state = ns;
    }

    pub fn set_state_el(&mut self, ns: ElStates) {
        self.el_state = ns;
    }


}


