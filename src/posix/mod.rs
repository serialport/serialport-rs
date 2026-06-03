pub use self::enumerate::*;
pub use self::tty::*;

mod enumerate;
mod error;
mod flock;
mod ioctl;
mod poll;
mod termios;
mod tty;

use std::os::unix::io::{AsRawFd, FromRawFd, IntoRawFd, RawFd};

#[allow(missing_docs)]
pub trait SerialPortExt {
    fn exclusive(&self) -> bool;
    fn set_exclusive(&mut self, exclusive: bool) -> crate::Result<()>;
    fn send_break(&self, duration: BreakDuration) -> crate::Result<()>;
    fn pair() -> crate::Result<(crate::SerialPort, crate::SerialPort)>;
}

impl SerialPortExt for crate::SerialPort {
    fn exclusive(&self) -> bool {
        self.0.exclusive()
    }

    fn set_exclusive(&mut self, exclusive: bool) -> crate::Result<()> {
        self.0.set_exclusive(exclusive)
    }

    fn send_break(&self, duration: BreakDuration) -> crate::Result<()> {
        self.0.send_break(duration)
    }

    fn pair() -> crate::Result<(crate::SerialPort, crate::SerialPort)> {
        TTYPort::pair().map(|(m, s)| (crate::SerialPort(m), crate::SerialPort(s)))
    }
}

impl AsRawFd for crate::SerialPort {
    fn as_raw_fd(&self) -> RawFd {
        self.0.as_raw_fd()
    }
}

impl IntoRawFd for crate::SerialPort {
    fn into_raw_fd(self) -> RawFd {
        self.0.into_raw_fd()
    }
}

impl FromRawFd for crate::SerialPort {
    unsafe fn from_raw_fd(fd: RawFd) -> Self {
        crate::SerialPort(crate::posix::TTYPort::from_raw_fd(fd))
    }
}
