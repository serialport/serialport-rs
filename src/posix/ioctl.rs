use std::mem;
use std::os::unix::io::RawFd;

use nix::libc;

// These are wrapped in a module because they're `pub` by default
mod raw {
    use nix::libc;
    ioctl!(bad none tiocexcl with libc::TIOCEXCL);
    ioctl!(bad none tiocnxcl with libc::TIOCNXCL);
    ioctl!(bad read tiocmget with libc::TIOCMGET; u8);
    ioctl!(bad write_ptr tiocmbic with libc::TIOCMBIC; u8);
    ioctl!(bad write_ptr tiocmbis with libc::TIOCMBIS; u8);
}

bitflags!{
    /// Flags to indicate which wires in a serial connection to use
    pub struct SerialLines: i32 {
        const DATA_SET_READY = libc::TIOCM_DSR;
        const DATA_TERMINAL_READY = libc::TIOCM_DTR;
        const REQUEST_TO_SEND = libc::TIOCM_RTS;
        const SECONDARY_TRANSMIT = libc::TIOCM_ST;
        const SECONDARY_RECEIVE = libc::TIOCM_SR;
        const CLEAR_TO_SEND = libc::TIOCM_CTS;
        const DATA_CARRIER_DETECT = libc::TIOCM_CAR;
        const RING = libc::TIOCM_RNG;
    }
}

pub fn tiocexcl(fd: RawFd) -> ::Result<()> {
    unsafe { raw::tiocexcl(fd) }
        .map(|_| ())
        .map_err(|e| e.into())
}

pub fn tiocnxcl(fd: RawFd) -> ::Result<()> {
    unsafe { raw::tiocnxcl(fd) }
        .map(|_| ())
        .map_err(|e| e.into())
}

pub fn tiocmget(fd: RawFd) -> ::Result<SerialLines> {
    let mut status = unsafe { mem::uninitialized() };
    unsafe { raw::tiocmget(fd, &mut status) }
        .map(SerialLines::from_bits_truncate)
        .map_err(|e| e.into())
}

pub fn tiocmbic(fd: RawFd, status: SerialLines) -> ::Result<()> {
    let bits = status.bits() as u8;
    unsafe { raw::tiocmbic(fd, &bits) }
        .map(|_| ())
        .map_err(|e| e.into())
}

pub fn tiocmbis(fd: RawFd, status: SerialLines) -> ::Result<()> {
    let bits = status.bits() as u8;
    unsafe { raw::tiocmbis(fd, &bits) }
        .map(|_| ())
        .map_err(|e| e.into())
}
