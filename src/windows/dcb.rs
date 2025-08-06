use std::mem::MaybeUninit;
use windows_sys::Win32::Devices::Communication::{
    GetCommState, SetCommState, DCB, EVENPARITY, NOPARITY, ODDPARITY, ONESTOPBIT, TWOSTOPBITS,
};
use windows_sys::Win32::Foundation::HANDLE;

use crate::{DataBits, FlowControl, Parity, Result, StopBits};

/// DTR control modes
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum DtrControl {
    Disable = 0x00,
    Enable = 0x01,
    Handshake = 0x02,
}

/// RTS control modes
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum RtsControl {
    Disable = 0x00,
    Enable = 0x01,
    Handshake = 0x02,
    Toggle = 0x03,
}

/// Helper trait to manipulate DCB bitfield flags
#[allow(non_snake_case, dead_code)]
pub(crate) trait DCBBitField {
    fn set_fBinary(&mut self, value: bool);
    fn set_fParity(&mut self, value: bool);
    fn set_fOutxCtsFlow(&mut self, value: bool);
    fn set_fOutxDsrFlow(&mut self, value: bool);
    fn set_fDtrControl(&mut self, value: DtrControl);
    fn set_fDsrSensitivity(&mut self, value: bool);
    fn set_fTXContinueOnXoff(&mut self, value: bool);
    fn set_fOutX(&mut self, value: bool);
    fn set_fInX(&mut self, value: bool);
    fn set_fErrorChar(&mut self, value: bool);
    fn set_fNull(&mut self, value: bool);
    fn set_fRtsControl(&mut self, value: RtsControl);
    fn set_fAbortOnError(&mut self, value: bool);

    // Getter methods
    fn fOutxCtsFlow(&self) -> bool;
    fn fRtsControl(&self) -> RtsControl;
    fn fOutX(&self) -> bool;
    fn fInX(&self) -> bool;
}

impl DCBBitField for DCB {
    fn set_fBinary(&mut self, value: bool) {
        if value {
            self._bitfield |= 1 << 0;
        } else {
            self._bitfield &= !(1 << 0);
        }
    }

    fn set_fParity(&mut self, value: bool) {
        if value {
            self._bitfield |= 1 << 1;
        } else {
            self._bitfield &= !(1 << 1);
        }
    }

    fn set_fOutxCtsFlow(&mut self, value: bool) {
        if value {
            self._bitfield |= 1 << 2;
        } else {
            self._bitfield &= !(1 << 2);
        }
    }

    fn set_fOutxDsrFlow(&mut self, value: bool) {
        if value {
            self._bitfield |= 1 << 3;
        } else {
            self._bitfield &= !(1 << 3);
        }
    }

    fn set_fDtrControl(&mut self, value: DtrControl) {
        // Clear bits 4-5 and set new value
        self._bitfield &= !(0b11 << 4);
        self._bitfield |= ((value as u32) & 0b11) << 4;
    }

    fn set_fDsrSensitivity(&mut self, value: bool) {
        if value {
            self._bitfield |= 1 << 6;
        } else {
            self._bitfield &= !(1 << 6);
        }
    }

    fn set_fTXContinueOnXoff(&mut self, value: bool) {
        if value {
            self._bitfield |= 1 << 7;
        } else {
            self._bitfield &= !(1 << 7);
        }
    }

    fn set_fOutX(&mut self, value: bool) {
        if value {
            self._bitfield |= 1 << 8;
        } else {
            self._bitfield &= !(1 << 8);
        }
    }

    fn set_fInX(&mut self, value: bool) {
        if value {
            self._bitfield |= 1 << 9;
        } else {
            self._bitfield &= !(1 << 9);
        }
    }

    fn set_fErrorChar(&mut self, value: bool) {
        if value {
            self._bitfield |= 1 << 10;
        } else {
            self._bitfield &= !(1 << 10);
        }
    }

    fn set_fNull(&mut self, value: bool) {
        if value {
            self._bitfield |= 1 << 11;
        } else {
            self._bitfield &= !(1 << 11);
        }
    }

    fn set_fRtsControl(&mut self, value: RtsControl) {
        // Clear bits 12-13 and set new value
        self._bitfield &= !(0b11 << 12);
        self._bitfield |= ((value as u32) & 0b11) << 12;
    }

    fn set_fAbortOnError(&mut self, value: bool) {
        if value {
            self._bitfield |= 1 << 14;
        } else {
            self._bitfield &= !(1 << 14);
        }
    }

    fn fOutxCtsFlow(&self) -> bool {
        (self._bitfield & (1 << 2)) != 0
    }

    fn fRtsControl(&self) -> RtsControl {
        let bits = (self._bitfield >> 12) & 0b11;
        match bits {
            0 => RtsControl::Disable,
            1 => RtsControl::Enable,
            2 => RtsControl::Handshake,
            3 => RtsControl::Toggle,
            _ => unreachable!(),
        }
    }

    fn fOutX(&self) -> bool {
        (self._bitfield & (1 << 8)) != 0
    }

    fn fInX(&self) -> bool {
        (self._bitfield & (1 << 9)) != 0
    }
}

pub(crate) fn get_dcb(handle: HANDLE) -> Result<DCB> {
    let mut dcb: DCB = unsafe { MaybeUninit::zeroed().assume_init() };
    dcb.DCBlength = std::mem::size_of::<DCB>() as u32;

    if unsafe { GetCommState(handle, &mut dcb) } != 0 {
        Ok(dcb)
    } else {
        Err(super::error::last_os_error())
    }
}

/// Initialize the DCB struct
/// Set all values that won't be affected by `SerialPortBuilder` options.
pub(crate) fn init(dcb: &mut DCB) {
    // dcb.DCBlength
    // dcb.BaudRate
    // dcb.wReserved
    // dcb.XonLim
    // dcb.XoffLim
    // dcb.ByteSize
    // dcb.Parity
    // dcb.StopBits
    dcb.XonChar = 17;
    dcb.XoffChar = 19;
    dcb.ErrorChar = 0;
    dcb.EofChar = 26;
    // dcb.EvtChar

    // always true for communications resources
    dcb.set_fBinary(true);
    // dcb.set_fParity()
    // dcb.set_fOutxCtsFlow()
    // serialport-rs doesn't support toggling DSR: so disable fOutxDsrFlow
    dcb.set_fOutxDsrFlow(false);
    dcb.set_fDtrControl(DtrControl::Disable);
    // disable because fOutxDsrFlow is disabled as well
    dcb.set_fDsrSensitivity(false);
    // dcb.set_fTXContinueOnXoff()
    // dcb.set_fOutX()
    // dcb.set_fInX()
    dcb.set_fErrorChar(false);
    // fNull: when set to TRUE null bytes are discarded when received.
    // null bytes won't be discarded by serialport-rs
    dcb.set_fNull(false);
    // dcb.set_fRtsControl()
    // serialport-rs does not handle the fAbortOnError behaviour, so we must make sure it's not enabled
    dcb.set_fAbortOnError(false);
}

pub(crate) fn set_dcb(handle: HANDLE, mut dcb: DCB) -> Result<()> {
    if unsafe { SetCommState(handle, &mut dcb as *mut _) != 0 } {
        Ok(())
    } else {
        Err(super::error::last_os_error())
    }
}

pub(crate) fn set_baud_rate(dcb: &mut DCB, baud_rate: u32) {
    dcb.BaudRate = baud_rate;
}

pub(crate) fn set_data_bits(dcb: &mut DCB, data_bits: DataBits) {
    dcb.ByteSize = match data_bits {
        DataBits::Five => 5,
        DataBits::Six => 6,
        DataBits::Seven => 7,
        DataBits::Eight => 8,
    };
}

pub(crate) fn set_parity(dcb: &mut DCB, parity: Parity) {
    dcb.Parity = match parity {
        Parity::None => NOPARITY,
        Parity::Odd => ODDPARITY,
        Parity::Even => EVENPARITY,
    };

    dcb.set_fParity(parity != Parity::None);
}

pub(crate) fn set_stop_bits(dcb: &mut DCB, stop_bits: StopBits) {
    dcb.StopBits = match stop_bits {
        StopBits::One => ONESTOPBIT,
        StopBits::Two => TWOSTOPBITS,
    };
}

pub(crate) fn set_flow_control(dcb: &mut DCB, flow_control: FlowControl) {
    match flow_control {
        FlowControl::None => {
            dcb.set_fOutxCtsFlow(false);
            dcb.set_fRtsControl(RtsControl::Disable);
            dcb.set_fOutX(false);
            dcb.set_fInX(false);
        }
        FlowControl::Software => {
            dcb.set_fOutxCtsFlow(false);
            dcb.set_fRtsControl(RtsControl::Disable);
            dcb.set_fOutX(true);
            dcb.set_fInX(true);
        }
        FlowControl::Hardware => {
            dcb.set_fOutxCtsFlow(true);
            dcb.set_fRtsControl(RtsControl::Enable);
            dcb.set_fOutX(false);
            dcb.set_fInX(false);
        }
    }
}
