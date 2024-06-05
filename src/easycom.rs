/* Easycom protocol implementation
 *
 * Implemented commands:
 *
 * Command      Meaning     Parameters
 * -------      -------     ----------
 *
 * ML           Move Left
 * MR           Move Right
 * MU           Move Up
 * MD           Move Down
 * SA           Stop azimuth moving
 * SE           Stop elevation moving
 *
 * VE           Request Version
 * AZ           Query azimuth
 * AZx.x        Rotate to Azimuth
 * AZxx.x       Rotate to Azimuth
 * AZxxx.x      Rotate to Azimuth
 * EL           Request Elevation
 * ELx.x        Rotate to Elevation
 * ELxx.x       Rotate to Elevation
 * ELxxx.x      Rotate to Elevation
 *
 *
 * Commands are executed upon space, carriage return, or line feed
 *
 * Reference: https://www.qsl.net/dh1ngp/onlinehelpft100/Rotator_control_with_Easycomm.htm
 *
 */

pub const EASYCOM_PROTOCOL_VERSION: &[u8] = b"VE002\n";
pub const MAX_AZ: f32 = 360.0;
pub const MAX_EL: f32 = 90.0;

pub trait AngleFormat {
    fn from_degrees(&self, val: &[u8]) -> Result<Self, ()>
    where
        Self: Sized;
}

// azimuth and elevation angles
// azimuth range 0..360
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AzAngle(f32);

impl Default for AzAngle {
    fn default() -> Self {
        AzAngle(0.0)
    }
}

impl TryFrom<f32> for AzAngle {
    type Error = ();

    fn try_from(_val: f32) -> Result<Self, Self::Error> {
        if _val >= 0.0 && _val <= MAX_AZ {
            Ok(AzAngle(_val))
        } else {
            Err(())
        }
    }
}

impl From<&AzAngle> for f32 {
    fn from(_val: &AzAngle) -> f32 {
        _val.0
    }
}

impl AngleFormat for AzAngle {
    fn from_degrees(&self, val: &[u8]) -> Result<Self, ()>
    where
        Self: Sized,
    {
        // format is AZx.x or AZxx.x or AZxxx.x
        let ang = deformat_degrees(val)?;
        <AzAngle>::try_from(ang)
    }

}

impl core::fmt::Display for AzAngle {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{0:.1}", self.0)
    }
}

fn deformat_degrees(val: &[u8]) -> Result<f32, ()> {
    // decrease by 48 ascii char to get it decimal value
    const ASCII_SHIFT: u8 = 48;
    let vl = val.len();
    if vl < 3 || vl > 5 {
        return Err(());
    }
    if val[vl - 2] != b'.' {
        return Err(());
    }
    let ang: f32 = match vl {
        3 => {
            <f32>::try_from(val[0] - ASCII_SHIFT).map_err(|_| ())?
                + <f32>::try_from(val[2] - ASCII_SHIFT).map_err(|_| ())? / 10.0
        }
        4 => {
            <f32>::try_from(val[0] - ASCII_SHIFT).map_err(|_| ())? * 10.0
                + <f32>::try_from(val[1] - ASCII_SHIFT).map_err(|_| ())?
                + <f32>::try_from(val[3] - ASCII_SHIFT).map_err(|_| ())? / 10.0
        }
        _ => {
            <f32>::try_from(val[0] - ASCII_SHIFT).map_err(|_| ())? * 100.0
                + <f32>::try_from(val[1] - ASCII_SHIFT).map_err(|_| ())? * 10.0
                + <f32>::try_from(val[2] - ASCII_SHIFT).map_err(|_| ())?
                + <f32>::try_from(val[4] - ASCII_SHIFT).map_err(|_| ())? / 10.0
        }
    };
    Ok(ang)
}

// elevation range 0..90
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ElAngle(f32);

impl Default for ElAngle {
    fn default() -> Self {
        ElAngle(0.0)
    }
}

impl TryFrom<f32> for ElAngle {
    type Error = ();

    fn try_from(_val: f32) -> Result<Self, Self::Error> {
        if _val >= 0.0 && _val <= MAX_EL {
            Ok(ElAngle(_val))
        } else {
            Err(())
        }
    }
}

impl From<&ElAngle> for f32 {
    fn from(_val: &ElAngle) -> f32 {
        _val.0
    }
}

impl AngleFormat for ElAngle {
    fn from_degrees(&self, val: &[u8]) -> Result<Self, ()>
    where
        Self: Sized,
    {
        // format is ELx.x or ELxx.x or ELxxx.x
        let ang = deformat_degrees(val)?;
        <ElAngle>::try_from(ang)
    }

}

impl core::fmt::Display for ElAngle {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{0:.1}", self.0)
    }
}

// Easycom command set
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EasycomCommands {
    ML,           // Move Left
    MR,           // Move Right
    MU,           // Move Up
    MD,           // Move Down
    SA,           // Stop azimuth moving
    SE,           // Stop elevation moving
    VE,           // Request Version
    AZ,           // Query/set azimuth
    EL,           // Request/set Elevation
}

impl TryFrom<&[u8]> for EasycomCommands {
    type Error = ();

    fn try_from(_val: &[u8]) -> Result<Self, Self::Error> {
        match _val {
            b"ML" => Ok(EasycomCommands::ML),
            b"MR" => Ok(EasycomCommands::MR),
            b"MU" => Ok(EasycomCommands::MU),
            b"MD" => Ok(EasycomCommands::MD),
            b"SA" => Ok(EasycomCommands::SA),
            b"SE" => Ok(EasycomCommands::SE),
            b"VE" => Ok(EasycomCommands::VE),
            b"AZ" => Ok(EasycomCommands::AZ),
            b"EL" => Ok(EasycomCommands::EL),
            _ => Err(()),
        }
    }
}
