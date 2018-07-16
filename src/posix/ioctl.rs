use std::mem;
use std::os::unix::io::RawFd;

use nix::libc;

// These are wrapped in a module because they're `pub` by default
mod raw {
    use nix::libc;
    ioctl_none_bad!(tiocexcl, libc::TIOCEXCL);
    ioctl_none_bad!(tiocnxcl, libc::TIOCNXCL);
    ioctl_read_bad!(tiocmget, libc::TIOCMGET, libc::c_int);
    ioctl_write_ptr_bad!(tiocmbic, libc::TIOCMBIC, libc::c_int);
    ioctl_write_ptr_bad!(tiocmbis, libc::TIOCMBIS, libc::c_int);
    ioctl_read!(
        #[cfg(any(target_os = "android", all(target_os = "linux", not(any(target_env = "musl", target_arch = "powerpc", target_arch = "powerpc64")))))]
        tcgets2, b'T', 0x2A, libc::termios2);
    ioctl_write_ptr!(
        #[cfg(any(target_os = "android", all(target_os = "linux", not(any(target_env = "musl", target_arch = "powerpc", target_arch = "powerpc64")))))]
        tcsets2, b'T', 0x2B, libc::termios2);
}

bitflags!{
    /// Flags to indicate which wires in a serial connection to use
    pub struct SerialLines: libc::c_int {
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
    let x = unsafe { raw::tiocmget(fd, &mut status) };
    x.map(SerialLines::from_bits_truncate).map_err(|e| e.into())
}

pub fn tiocmbic(fd: RawFd, status: SerialLines) -> ::Result<()> {
    let bits = status.bits() as libc::c_int;
    unsafe { raw::tiocmbic(fd, &bits) }
        .map(|_| ())
        .map_err(|e| e.into())
}

pub fn tiocmbis(fd: RawFd, status: SerialLines) -> ::Result<()> {
    let bits = status.bits() as libc::c_int;
    unsafe { raw::tiocmbis(fd, &bits) }
        .map(|_| ())
        .map_err(|e| e.into())
}

#[cfg(
    any(
        target_os = "android",
        all(
            target_os = "linux",
            not(any(target_env = "musl", target_arch = "powerpc", target_arch = "powerpc64"))
        )
    )
)]
pub fn tcgets2(fd: RawFd) -> ::Result<libc::termios2> {
    let mut options = unsafe { mem::uninitialized() };
    match unsafe { raw::tcgets2(fd, &mut options) } {
        Ok(_) => Ok(options),
        Err(e) => Err(e.into()),
    }
}

#[cfg(
    any(
        target_os = "android",
        all(
            target_os = "linux",
            not(any(target_env = "musl", target_arch = "powerpc", target_arch = "powerpc64"))
        )
    )
)]
pub fn tcsets2(fd: RawFd, options: &libc::termios2) -> ::Result<()> {
    unsafe { raw::tcsets2(fd, options) }
        .map(|_| ())
        .map_err(|e| e.into())
}
