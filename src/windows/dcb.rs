use std::mem::MaybeUninit;
use winapi::shared::minwindef::*;
use winapi::um::commapi::*;
use winapi::um::winbase::*;
use winapi::um::winnt::HANDLE;

use crate::{DataBits, FlowControl, Parity, Result, StopBits};

pub(crate) fn get_dcb(handle: HANDLE) -> Result<DCB> {
    let mut dcb: DCB = unsafe { MaybeUninit::zeroed().assume_init() };
    dcb.DCBlength = std::mem::size_of::<DCB>() as u32;

    if unsafe { GetCommState(handle, &mut dcb) } != 0 {
        return Ok(dcb);
    } else {
        return Err(super::error::last_os_error());
    }
}

pub(crate) fn set_dcb(handle: HANDLE, mut dcb: DCB) -> Result<()> {
    if unsafe { SetCommState(handle, &mut dcb as *mut _) != 0 } {
        return Ok(());
    } else {
        return Err(super::error::last_os_error());
    }
}

pub(crate) fn set_baud_rate(dcb: &mut DCB, baud_rate: u32) {
    dcb.BaudRate = baud_rate as DWORD;
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
        Parity::None => NOPARITY as u8,
        Parity::Odd => ODDPARITY as u8,
        Parity::Even => EVENPARITY as u8,
    };
}

pub(crate) fn set_stop_bits(dcb: &mut DCB, stop_bits: StopBits) {
    dcb.StopBits = match stop_bits {
        StopBits::One => ONESTOPBIT as u8,
        StopBits::Two => TWOSTOPBITS as u8,
    };
}

pub(crate) fn set_flow_control(dcb: &mut DCB, flow_control: FlowControl) {
    match flow_control {
        FlowControl::None => {
            dcb.set_fOutxCtsFlow(0);
            dcb.set_fRtsControl(0);
            dcb.set_fOutX(0);
            dcb.set_fInX(0);
        }
        FlowControl::Software => {
            dcb.set_fOutxCtsFlow(0);
            dcb.set_fRtsControl(0);
            dcb.set_fOutX(1);
            dcb.set_fInX(1);
        }
        FlowControl::Hardware => {
            dcb.set_fOutxCtsFlow(1);
            dcb.set_fRtsControl(1);
            dcb.set_fOutX(0);
            dcb.set_fInX(0);
        }
    }
}
