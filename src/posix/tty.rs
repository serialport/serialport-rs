#[cfg(target_os = "macos")]
use std::ffi::{CStr, CString};
#[cfg(target_os = "linux")]
use std::ffi::OsStr;
use std::{io, mem};
use std::os::unix::prelude::*;
use std::path::Path;
use std::time::Duration;

#[cfg(target_os = "macos")]
use cf::*;
#[cfg(target_os = "macos")]
use IOKit_sys::*;
use posix::ioctl;
#[cfg(target_os = "macos")]
use nix::libc::{c_char, c_void};
#[cfg(target_os = "linux")]
use libudev;
use nix::{self, unistd};
use nix::fcntl::fcntl;
use nix::sys::termios::SetArg::TCSANOW;
use nix::sys::termios::{tcgetattr, tcsetattr, Termios};

use {BaudRate, DataBits, FlowControl, Parity, SerialPort, SerialPortInfo, SerialPortSettings,
     SerialPortType, StopBits, UsbPortInfo};
use {Error, ErrorKind};

/// Convenience method for removing exclusive access from
/// a fd and closing it.
fn close(fd: RawFd) {
    // remove exclusive access
    let _ = ioctl::tiocnxcl(fd);

    // On Linux and BSD, we don't need to worry about return
    // type as EBADF means the fd was never open or is already closed
    //
    // Linux and BSD guarantee that for any other error code the
    // fd is already closed, though MacOSX does not.
    //
    // close() also should never be retried, and the error code
    // in most cases in purely informative
    let _ = unistd::close(fd);
}

/// A TTY-based serial port implementation.
///
/// The port will be closed when the value is dropped. However, this struct
/// should not be instantiated directly by using `TTYPort::open()`, instead use
/// the cross-platform `serialport::open()` or
/// `serialport::open_with_settings()`.
#[derive(Debug)]
pub struct TTYPort {
    fd: RawFd,
    timeout: Duration,
    exclusive: bool,
    port_name: Option<String>,
}

impl TTYPort {
    /// Opens a TTY device as a serial port.
    ///
    /// `path` should be the path to a TTY device, e.g., `/dev/ttyS0`.
    ///
    /// Ports are opened in exclusive mode by default. If this is undesireable
    /// behavior, use `TTYPort::set_exclusive(false)`.
    ///
    /// ## Errors
    ///
    /// * `NoDevice` if the device could not be opened. This could indicate that
    ///    the device is already in use.
    /// * `InvalidInput` if `path` is not a valid device name.
    /// * `Io` for any other error while opening or initializing the device.
    pub fn open(path: &Path, settings: &SerialPortSettings) -> ::Result<TTYPort> {

        use nix::fcntl::OFlag;
        use nix::fcntl::FcntlArg::F_SETFL;
        use nix::sys::termios::ControlFlags;
        use nix::sys::termios::{cfmakeraw, tcgetattr, tcsetattr, tcflush};
        use nix::sys::termios::SetArg::TCSANOW;
        use nix::sys::termios::FlushArg::TCIOFLUSH;

        let fd = nix::fcntl::open(path,
                                  OFlag::O_RDWR | OFlag::O_NOCTTY | OFlag::O_NONBLOCK,
                                  nix::sys::stat::Mode::empty())?;

        let mut termios = tcgetattr(fd).map_err(|e| {
            close(fd);
            e
        })?;

        // If any of these steps fail, then we should abort creation of the
        // TTYPort and ensure the file descriptor is closed.
        // So we wrap these calls in a block and check the result.
        {
            // setup TTY for binary serial port access
            // Enable reading from the port and ignore all modem control lines
            termios.control_flags.insert(ControlFlags::CREAD | ControlFlags::CLOCAL);
            // Enable raw mode with disables any implicit processing of the input or output data streams
            // This also sets no timeout period and a read will block until at least one character is
            // available.
            cfmakeraw(&mut termios);

            // write settings to TTY
            tcsetattr(fd, TCSANOW, &termios)?;

            // Read back settings from port and confirm they were applied correctly
            let actual_termios = tcgetattr(fd)?;

            if actual_termios.input_flags != termios.input_flags ||
               actual_termios.output_flags != termios.output_flags ||
               actual_termios.local_flags != termios.local_flags ||
               actual_termios.control_flags != termios.control_flags {
                return Err(Error::new(ErrorKind::Unknown, "Settings did not apply correctly"));
            };

            tcflush(fd, TCIOFLUSH)?;

            // get exclusive access to device
            ioctl::tiocexcl(fd)?;

            // clear O_NONBLOCK flag
            fcntl(fd, F_SETFL(nix::fcntl::OFlag::empty()))?;

            Ok(())

        }.map_err(|e:Error|{
            close(fd);
            e
        })?;

        let mut port = TTYPort {
            fd: fd,
            timeout: Duration::new(0, 0), // This is overwritten by the subsequent call to `set_all()`
            exclusive: true, // This is guaranteed by the above `ioctl::tiocexcl()` call
            port_name: path.to_str().map(|s| s.to_string()),
        };

        // Then we try and finish setting up the port.
        // If this fails, we also need to be sure to close the
        // file descriptor.
        if let Err(err) = port.set_all(settings) {
            close(fd);
            return Err(err);
        }

        Ok(port)
    }

    /// Returns the exclusivity of the port
    ///
    /// If a port is exclusive, then trying to open the same device path again
    /// will fail.
    pub fn exclusive(&self) -> bool {
        self.exclusive
    }

    /// Sets the exclusivity of the port
    ///
    /// If a port is exclusive, then trying to open the same device path again
    /// will fail.
    ///
    /// See the man pages for the tiocexcl and tiocnxcl ioctl's for more details.
    ///
    /// ## Errors
    ///
    /// * `Io` for any error while setting exclusivity for the port.
    pub fn set_exclusive(&mut self, exclusive: bool) -> ::Result<()> {
        let setting_result = if exclusive {
            ioctl::tiocexcl(self.fd)
        } else {
            ioctl::tiocnxcl(self.fd)
        };

        if let Err(err) = setting_result {
            Err(err)
        } else {
            self.exclusive = exclusive;
            Ok(())
        }
    }

    fn set_pin(&mut self, pin: ioctl::SerialLines, level: bool) -> ::Result<()> {
        let retval = if level {
            ioctl::tiocmbis(self.fd, pin)
        } else {
            ioctl::tiocmbic(self.fd, pin)
        };

        match retval {
            Ok(()) => Ok(()),
            Err(err) => Err(err),
        }
    }

    fn read_pin(&mut self, pin: ioctl::SerialLines) -> ::Result<bool> {
        match ioctl::tiocmget(self.fd) {
            Ok(pins) => Ok(pins.contains(pin)),
            Err(err) => Err(err),
        }
    }

    /// Create a pair of pseudo serial terminals
    ///
    /// ## Returns
    /// Two connected `TTYPort` objects: `(master, slave)`
    ///
    /// ## Errors
    /// Attempting any IO or parameter settings on the slave tty after the master
    /// tty is closed will return errors.
    ///
    /// On some platforms manipulating the master port will fail and only
    /// modifying the slave port is possible.
    ///
    /// ## Examples
    ///
    /// ```
    /// use serialport::posix::TTYPort;
    ///
    /// let (master, slave) = TTYPort::pair().unwrap();
    /// ```
    pub fn pair() -> ::Result<(Self, Self)> {

        // Open the next free pty.
        let next_pty_fd = nix::pty::posix_openpt(nix::fcntl::OFlag::O_RDWR)?;

        // Grant access to the associated slave pty
        nix::pty::grantpt(&next_pty_fd)?;

        // Unlock the slave pty
        nix::pty::unlockpt(&next_pty_fd)?;

        // Get the path of the attached slave ptty
        #[cfg(not(any(target_os = "linux",
                      target_os = "android",
                      target_os = "emscripten",
                      target_os = "fuchsia")))]
        let ptty_name = unsafe { nix::pty::ptsname(&next_pty_fd)? };

        #[cfg(any(target_os = "linux",
                  target_os = "android",
                  target_os = "emscripten",
                  target_os = "fuchsia"))]
        let ptty_name = nix::pty::ptsname_r(&next_pty_fd)?;

        // Open the slave port using default settings
        let slave_tty = TTYPort::open(Path::new(&ptty_name), &Default::default())?;

        // Manually construct the master port here because the
        // `tcgetattr()` doesn't work on Mac, Solaris, and maybe other
        // BSDs when used on the master port.
        let master_tty = TTYPort {
            fd: next_pty_fd.into_raw_fd(),
            timeout: Duration::from_millis(100),
            exclusive: true,
            port_name: None,
        };

        Ok((master_tty, slave_tty))
    }

    fn get_termios(&self) -> ::Result<Termios> {
        tcgetattr(self.fd).map_err(|e| e.into())
    }

    fn set_termios(&self, termios: &Termios) -> ::Result<()> {
        tcsetattr(self.fd, TCSANOW, &termios).map_err(|e| e.into())
    }
}

impl Drop for TTYPort {
    fn drop(&mut self) {
        close(self.fd);
    }
}

impl AsRawFd for TTYPort {
    fn as_raw_fd(&self) -> RawFd {
        self.fd
    }
}

impl IntoRawFd for TTYPort {
    fn into_raw_fd(self) -> RawFd {
        // Pull just the file descriptor out. We also prevent the destructor
        // from being run by calling `mem::forget`. If we didn't do this, the
        // port would be closed, which would make `into_raw_fd` unusable.
        let TTYPort { fd, .. } = self;
        mem::forget(self);
        fd
    }
}

impl FromRawFd for TTYPort {
    unsafe fn from_raw_fd(fd: RawFd) -> Self {
        // Try to set exclusive, as is the default setting.  Catch errors.. this method MUST
        // return a TTYPort so we'll just indicate non-exclusive on an error here.
        let exclusive = match ioctl::tiocexcl(fd) {
            Ok(_) => true,
            Err(_) => false,
        };

        // It is not trivial to get the file path corresponding to a file descriptor.
        // We'll punt and set it None here.
        TTYPort {
            fd: fd,
            timeout: Duration::from_millis(100),
            exclusive: exclusive,
            port_name: None,
        }
    }
}

impl io::Read for TTYPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if let Err(e) = super::poll::wait_read_fd(self.fd, self.timeout) {
            return Err(io::Error::from(::Error::from(e)));
        }

        match nix::unistd::read(self.fd, buf) {
            Ok(n) => Ok(n),
            Err(e) => Err(io::Error::from(::Error::from(e))),
        }
    }
}

impl io::Write for TTYPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if let Err(e) = super::poll::wait_write_fd(self.fd, self.timeout) {
            return Err(io::Error::from(::Error::from(e)));
        }

        match nix::unistd::write(self.fd, buf) {
            Ok(n) => Ok(n),
            Err(e) => Err(io::Error::from(::Error::from(e))),
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        nix::sys::termios::tcdrain(self.fd).map_err(|_| {
            io::Error::new(io::ErrorKind::Other, "flush failed")
        })
    }
}

impl SerialPort for TTYPort {
    fn port_name(&self) -> Option<String> {
        self.port_name.clone()
    }

    /// Returns a struct with all port settings
    fn settings(&self) -> SerialPortSettings {
        SerialPortSettings {
            baud_rate: self.baud_rate().expect("Couldn't retrieve baud rate"),
            data_bits: self.data_bits().expect("Couldn't retrieve data bits"),
            flow_control: self.flow_control().expect("Couldn't retrieve flow control"),
            parity: self.parity().expect("Couldn't retrieve parity"),
            stop_bits: self.stop_bits().expect("Couldn't retrieve stop bits"),
            timeout: self.timeout,
        }
    }

    fn baud_rate(&self) -> Option<BaudRate> {
        use nix::sys::termios::{cfgetospeed, cfgetispeed};
        use nix::sys::termios::BaudRate::*;

        let termios = match self.get_termios() {
            Ok(t) => t,
            Err(_) => return None,
        };
        let ospeed = cfgetospeed(&termios);
        let ispeed = cfgetispeed(&termios);

        if ospeed != ispeed {
            return None;
        }

        match (ospeed as nix::libc::speed_t).into() {
            B50 => Some(BaudRate::Baud50),
            B75 => Some(BaudRate::Baud75),
            B110 => Some(BaudRate::Baud110),
            B134 => Some(BaudRate::Baud134),
            B150 => Some(BaudRate::Baud150),
            B200 => Some(BaudRate::Baud200),
            B300 => Some(BaudRate::Baud300),
            B600 => Some(BaudRate::Baud600),
            B1200 => Some(BaudRate::Baud1200),
            B1800 => Some(BaudRate::Baud1800),
            B2400 => Some(BaudRate::Baud2400),
            B4800 => Some(BaudRate::Baud4800),
            #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
            B7200 => Some(BaudRate::Baud7200),
            B9600 => Some(BaudRate::Baud9600),
            #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
            B14400 => Some(BaudRate::Baud14400),
            B19200 => Some(BaudRate::Baud19200),
            #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
            B28800 => Some(BaudRate::Baud28800),
            B38400 => Some(BaudRate::Baud38400),
            B57600 => Some(BaudRate::Baud57600),
            #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
            B76800 => Some(BaudRate::Baud76800),
            B115200 => Some(BaudRate::Baud115200),
            B230400 => Some(BaudRate::Baud230400),
            #[cfg(any(target_os = "android", target_os = "linux", target_os = "freebsd"))]
            B460800 => Some(BaudRate::Baud460800),
            #[cfg(any(target_os = "android", target_os = "linux"))]
            B500000 => Some(BaudRate::Baud500000),
            #[cfg(any(target_os = "android", target_os = "linux"))]
            B576000 => Some(BaudRate::Baud576000),
            // FIXME: Re-enable Baud921600 once nix > 0.10.0 is released
            #[cfg(any(target_os = "android", target_os = "freebsd", target_os = "linux", target_os = "netbsd"))]
            B921600 => Some(BaudRate::BaudOther(921_600)),
            #[cfg(any(target_os = "android", target_os = "linux"))]
            B1000000 => Some(BaudRate::Baud1000000),
            #[cfg(any(target_os = "android", target_os = "linux"))]
            B1152000 => Some(BaudRate::Baud1152000),
            #[cfg(any(target_os = "android",target_os = "linux"))]
            B1500000 => Some(BaudRate::Baud1500000),
            #[cfg(any(target_os = "android", target_os = "linux"))]
            B2000000 => Some(BaudRate::Baud2000000),
            #[cfg(any(target_os = "android", target_os = "linux"))]
            B2500000 => Some(BaudRate::Baud2500000),
            #[cfg(any(target_os = "android", target_os = "linux"))]
            B3000000 => Some(BaudRate::Baud3000000),
            #[cfg(any(target_os = "android", target_os = "linux"))]
            B3500000 => Some(BaudRate::Baud3500000),
            #[cfg(any(target_os = "android", target_os = "linux"))]
            B4000000 => Some(BaudRate::Baud4000000),
            _ => None,
        }
    }

    fn data_bits(&self) -> Option<DataBits> {
        use nix::sys::termios::ControlFlags;

        let termios = match self.get_termios() {
            Ok(t) => t,
            Err(_) => return None,
        };
        match termios.control_flags & ControlFlags::CSIZE {
            ControlFlags::CS8 => Some(DataBits::Eight),
            ControlFlags::CS7 => Some(DataBits::Seven),
            ControlFlags::CS6 => Some(DataBits::Six),
            ControlFlags::CS5 => Some(DataBits::Five),

            _ => None,
        }
    }

    fn flow_control(&self) -> Option<FlowControl> {
        use nix::sys::termios::{ControlFlags, InputFlags};

        let termios = match self.get_termios() {
            Ok(t) => t,
            Err(_) => return None,
        };
        if termios.control_flags.contains(ControlFlags::CRTSCTS) {
            Some(FlowControl::Hardware)
        } else if termios.input_flags.intersects(InputFlags::IXON | InputFlags::IXOFF) {
            Some(FlowControl::Software)
        } else {
            Some(FlowControl::None)
        }
    }

    fn parity(&self) -> Option<Parity> {
        use nix::sys::termios::ControlFlags;

        let termios = match self.get_termios() {
            Ok(t) => t,
            Err(_) => return None,
        };
        if termios.control_flags.contains(ControlFlags::PARENB) {
            if termios.control_flags.contains(ControlFlags::PARODD) {
                Some(Parity::Odd)
            } else {
                Some(Parity::Even)
            }
        } else {
            Some(Parity::None)
        }
    }

    fn stop_bits(&self) -> Option<StopBits> {
        use nix::sys::termios::ControlFlags;

        let termios = match self.get_termios() {
            Ok(t) => t,
            Err(_) => return None,
        };
        if termios.control_flags.contains(ControlFlags::CSTOPB) {
            Some(StopBits::Two)
        } else {
            Some(StopBits::One)
        }
    }

    fn timeout(&self) -> Duration {
        self.timeout
    }

    fn set_all(&mut self, settings: &SerialPortSettings) -> ::Result<()> {
        self.set_baud_rate(settings.baud_rate)?;
        self.set_data_bits(settings.data_bits)?;
        self.set_flow_control(settings.flow_control)?;
        self.set_parity(settings.parity)?;
        self.set_stop_bits(settings.stop_bits)?;
        self.set_timeout(settings.timeout)?;

        Ok(())
    }

    fn set_baud_rate(&mut self, baud_rate: BaudRate) -> ::Result<()> {
        use nix::sys::termios::cfsetspeed;
        use nix::sys::termios::BaudRate::*;

        let mut termios = self.get_termios()?;
        let baud = match baud_rate {
            BaudRate::Baud50 => B50,
            BaudRate::Baud75 => B75,
            BaudRate::Baud110 => B110,
            BaudRate::Baud134 => B134,
            BaudRate::Baud150 => B150,
            BaudRate::Baud200 => B200,
            BaudRate::Baud300 => B300,
            BaudRate::Baud600 => B600,
            BaudRate::Baud1200 => B1200,
            BaudRate::Baud1800 => B1800,
            BaudRate::Baud2400 => B2400,
            BaudRate::Baud4800 => B4800,
            #[cfg(any(target_os = "freebsd", target_os = "dragonfly", target_os = "macos",
                        target_os = "netbsd", target_os = "openbsd"))]
            BaudRate::Baud7200 => B7200,
            BaudRate::Baud9600 => B9600,
            #[cfg(any(target_os = "freebsd", target_os = "dragonfly", target_os = "macos",
                        target_os = "netbsd", target_os = "openbsd"))]
            BaudRate::Baud14400 => B14400,
            BaudRate::Baud19200 => B19200,
            #[cfg(any(target_os = "freebsd", target_os = "dragonfly", target_os = "macos",
                        target_os = "netbsd", target_os = "openbsd"))]
            BaudRate::Baud28800 => B28800,
            BaudRate::Baud38400 => B38400,
            BaudRate::Baud57600 => B57600,
            #[cfg(any(target_os = "freebsd", target_os = "dragonfly", target_os = "macos",
                        target_os = "netbsd", target_os = "openbsd"))]
            BaudRate::Baud76800 => B76800,
            BaudRate::Baud115200 => B115200,
            BaudRate::Baud230400 => B230400,
            #[cfg(any(target_os = "android", target_os = "freebsd", target_os = "linux"))]
            BaudRate::Baud460800 => B460800,
            #[cfg(any(target_os = "android", target_os = "linux"))]
            BaudRate::Baud500000 => B500000,
            #[cfg(any(target_os = "android", target_os = "linux"))]
            BaudRate::Baud576000 => B576000,
            // FIXME: Re-enable Baud921600 for FreeBSD & NetBSD once nix > 0.10.0 is released
            #[cfg(any(target_os = "android", target_os = "linux"))]
            BaudRate::Baud921600 => B921600,
            #[cfg(any(target_os = "android", target_os = "linux"))]
            BaudRate::Baud1000000 => B1000000,
            #[cfg(any(target_os = "android", target_os = "linux"))]
            BaudRate::Baud1152000 => B1152000,
            #[cfg(any(target_os = "android", target_os = "linux"))]
            BaudRate::Baud1500000 => B1500000,
            #[cfg(any(target_os = "android", target_os = "linux"))]
            BaudRate::Baud2000000 => B2000000,
            #[cfg(any(target_os = "android", target_os = "linux"))]
            BaudRate::Baud2500000 => B2500000,
            #[cfg(any(target_os = "android", target_os = "linux"))]
            BaudRate::Baud3000000 => B3000000,
            #[cfg(any(target_os = "android", target_os = "linux"))]
            BaudRate::Baud3500000 => B3500000,
            #[cfg(any(target_os = "android", target_os = "linux"))]
            BaudRate::Baud4000000 => B4000000,

            BaudRate::BaudOther(_) => return Err(nix::Error::from_errno(nix::errno::Errno::EINVAL).into()),
        };

        cfsetspeed(&mut termios, baud)?;
        self.set_termios(&termios)
    }

    fn set_data_bits(&mut self, data_bits: DataBits) -> ::Result<()> {
        use nix::sys::termios::ControlFlags;

        let size = match data_bits {
            DataBits::Five => ControlFlags::CS5,
            DataBits::Six => ControlFlags::CS6,
            DataBits::Seven => ControlFlags::CS7,
            DataBits::Eight => ControlFlags::CS8,
        };

        let mut termios = self.get_termios()?;
        termios.control_flags.remove(ControlFlags::CSIZE);
        termios.control_flags.insert(size);
        self.set_termios(&termios)
    }

    fn set_flow_control(&mut self, flow_control: FlowControl) -> ::Result<()> {
        use nix::sys::termios::{ControlFlags, InputFlags};

        let mut termios = self.get_termios()?;
        match flow_control {
            FlowControl::None => {
                termios.input_flags.remove(InputFlags::IXON | InputFlags::IXOFF);
                termios.control_flags.remove(ControlFlags::CRTSCTS);
            }
            FlowControl::Software => {
                termios.input_flags.insert(InputFlags::IXON | InputFlags::IXOFF);
                termios.control_flags.remove(ControlFlags::CRTSCTS);
            }
            FlowControl::Hardware => {
                termios.input_flags.remove(InputFlags::IXON | InputFlags::IXOFF);
                termios.control_flags.insert(ControlFlags::CRTSCTS);
            }
        };
        self.set_termios(&termios)
    }

    fn set_parity(&mut self, parity: Parity) -> ::Result<()> {
        use nix::sys::termios::{ControlFlags, InputFlags};

        let mut termios = self.get_termios()?;
        match parity {
            Parity::None => {
                termios.control_flags.remove(ControlFlags::PARENB | ControlFlags::PARODD);
                termios.input_flags.remove(InputFlags::INPCK);
                termios.input_flags.insert(InputFlags::IGNPAR);
            }
            Parity::Odd => {
                termios.control_flags.insert(ControlFlags::PARENB | ControlFlags::PARODD);
                termios.input_flags.insert(InputFlags::INPCK);
                termios.input_flags.remove(InputFlags::IGNPAR);
            }
            Parity::Even => {
                termios.control_flags.remove(ControlFlags::PARODD);
                termios.control_flags.insert(ControlFlags::PARENB);
                termios.input_flags.insert(InputFlags::INPCK);
                termios.input_flags.remove(InputFlags::IGNPAR);
            }
        };
        self.set_termios(&termios)
    }

    fn set_stop_bits(&mut self, stop_bits: StopBits) -> ::Result<()> {
        use nix::sys::termios::ControlFlags;

        let mut termios = self.get_termios()?;
        match stop_bits {
            StopBits::One => termios.control_flags.remove(ControlFlags::CSTOPB),
            StopBits::Two => termios.control_flags.insert(ControlFlags::CSTOPB),
        };
        self.set_termios(&termios)
    }

    fn set_timeout(&mut self, timeout: Duration) -> ::Result<()> {
        self.timeout = timeout;
        Ok(())
    }

    fn write_request_to_send(&mut self, level: bool) -> ::Result<()> {
        self.set_pin(ioctl::REQUEST_TO_SEND, level)
    }

    fn write_data_terminal_ready(&mut self, level: bool) -> ::Result<()> {
        self.set_pin(ioctl::DATA_TERMINAL_READY, level)
    }

    fn read_clear_to_send(&mut self) -> ::Result<bool> {
        self.read_pin(ioctl::CLEAR_TO_SEND)
    }

    fn read_data_set_ready(&mut self) -> ::Result<bool> {
        self.read_pin(ioctl::DATA_SET_READY)
    }

    fn read_ring_indicator(&mut self) -> ::Result<bool> {
        self.read_pin(ioctl::RING)
    }

    fn read_carrier_detect(&mut self) -> ::Result<bool> {
        self.read_pin(ioctl::DATA_CARRIER_DETECT)
    }

    fn try_clone(&self) -> ::Result<Box<SerialPort>> {
        let fd_cloned: i32 = fcntl(self.fd, nix::fcntl::F_DUPFD(self.fd))?;
        Ok(Box::new(TTYPort {
            fd: fd_cloned,
            exclusive: self.exclusive,
            port_name: self.port_name.clone(),
            timeout: self.timeout,
        }))
    }
}

/// Retrieves the udev property value named by `key`. If the value exists, then it will be
/// converted to a String, otherwise None will be returned.
#[cfg(target_os = "linux")]
fn udev_property_as_string(d: &libudev::Device, key: &str) -> Option<String> {
    if let Some(s) = d.property_value(key).and_then(OsStr::to_str) {
        Some(s.to_string())
    } else {
        None
    }
}

/// Retrieves the udev property value named by `key`. This function assumes that the retrieved
/// string is comprised of hex digits and the integer value of this will be returned as  a u16.
/// If the property value doesn't exist or doesn't contain valid hex digits, then an error
/// will be returned.
#[cfg(target_os = "linux")]
fn udev_hex_property_as_u16(d: &libudev::Device, key: &str) -> ::Result<u16> {
    if let Some(hex_str) = d.property_value(key).and_then(OsStr::to_str) {
        if let Ok(num) = u16::from_str_radix(hex_str, 16) {
            Ok(num)
        } else {
            Err(Error::new(ErrorKind::Unknown, "value not hex string"))
        }
    } else {
        Err(Error::new(ErrorKind::Unknown, "key not found"))
    }
}

#[cfg(target_os = "linux")]
fn port_type(d: &libudev::Device) -> ::Result<SerialPortType> {
    match d.property_value("ID_BUS").and_then(OsStr::to_str) {
        Some("usb") => {
            let serial_number = udev_property_as_string(d, "ID_SERIAL_SHORT");
            Ok(SerialPortType::UsbPort(UsbPortInfo {
                vid: udev_hex_property_as_u16(d, "ID_VENDOR_ID")?,
                pid: udev_hex_property_as_u16(d, "ID_MODEL_ID")?,
                serial_number: serial_number,
                manufacturer: udev_property_as_string(d, "ID_VENDOR"),
                product: udev_property_as_string(d, "ID_MODEL"),
            }))
        }
        Some("pci") => Ok(SerialPortType::PciPort),
        _ => Ok(SerialPortType::Unknown),
    }
}

#[cfg(target_os = "linux")]
/// Scans the system for serial ports and returns a list of them.
/// The `SerialPortInfo` struct contains the name of the port
/// which can be used for opening it.
pub fn available_ports() -> ::Result<Vec<SerialPortInfo>> {
    let mut vec = Vec::new();
    if let Ok(context) = libudev::Context::new() {
        let mut enumerator = libudev::Enumerator::new(&context)?;
        enumerator.match_subsystem("tty")?;
        let devices = enumerator.scan_devices()?;
        for d in devices {
            if let Some(p) = d.parent() {
                if let Some(devnode) = d.devnode() {
                    if let Some(path) = devnode.to_str() {
                        if let Some(driver) = p.driver() {
                            if driver == "serial8250" &&
                                TTYPort::open(devnode, &Default::default()).is_err()
                            {
                                continue;
                            }
                        }
                        // Stop bubbling up port_type errors here so problematic ports are just
                        // skipped instead of causing no ports to be returned.
                        if let Ok(pt) = port_type(&d) {
                            vec.push(SerialPortInfo {
                                port_name: String::from(path),
                                port_type: pt,
                            });
                        }
                    }
                }
            }
        }
    }
    Ok(vec)
}

#[cfg(target_os = "macos")]
fn get_parent_device_by_type(
    device: io_object_t,
    parent_type: *const c_char,
) -> Option<io_registry_entry_t> {
    let parent_type = unsafe { CStr::from_ptr(parent_type) };
    use mach::kern_return::KERN_SUCCESS;
    let mut device = device;
    loop {
        let mut class_name: [c_char; 128] = unsafe { mem::uninitialized() };
        unsafe { IOObjectGetClass(device, &mut class_name[0]) };
        let name = unsafe { CStr::from_ptr(&class_name[0]) };
        if name == parent_type {
            return Some(device);
        }
        let mut parent: io_registry_entry_t = unsafe { mem::uninitialized() };
        if unsafe {
            IORegistryEntryGetParentEntry(device, kIOServiceClass(), &mut parent) != KERN_SUCCESS
        }
        {
            return None;
        }
        device = parent;
    }
}

#[cfg(target_os = "macos")]
#[allow(non_upper_case_globals)]
/// Returns a specific property of the given device as an integer.
fn get_int_property(
    device_type: io_registry_entry_t,
    property: &str,
    cf_number_type: CFNumberType,
) -> Option<u32> {
    unsafe {
        let prop_str = CString::new(property).unwrap();
        let key = CFStringCreateWithCString(
            kCFAllocatorDefault,
            prop_str.as_ptr(),
            kCFStringEncodingUTF8,
        );
        let container = IORegistryEntryCreateCFProperty(device_type, key, kCFAllocatorDefault, 0);
        if container.is_null() {
            return None;
        }
        let num = match cf_number_type {
            kCFNumberSInt16Type => {
                let mut num: u16 = 0;
                let num_ptr: *mut c_void = &mut num as *mut _ as *mut c_void;
                CFNumberGetValue(container as CFNumberRef, cf_number_type, num_ptr);
                Some(num as u32)
            }
            kCFNumberSInt32Type => {
                let mut num: u32 = 0;
                let num_ptr: *mut c_void = &mut num as *mut _ as *mut c_void;
                CFNumberGetValue(container as CFNumberRef, cf_number_type, num_ptr);
                Some(num)
            }
            _ => None,
        };
        CFRelease(container);

        num
    }
}

#[cfg(target_os = "macos")]
/// Returns a specific property of the given device as a string.
fn get_string_property(device_type: io_registry_entry_t, property: &str) -> Option<String> {
    unsafe {
        let prop_str = CString::new(property).unwrap();
        let key = CFStringCreateWithCString(
            kCFAllocatorDefault,
            prop_str.as_ptr(),
            kCFStringEncodingUTF8,
        );
        let container = IORegistryEntryCreateCFProperty(device_type, key, kCFAllocatorDefault, 0);
        if container.is_null() {
            return None;
        }

        let str_ptr = CFStringGetCStringPtr(container as CFStringRef, kCFStringEncodingMacRoman);
        if str_ptr.is_null() {
            CFRelease(container);
            return None;
        }
        let opt_str = CStr::from_ptr(str_ptr).to_str().ok().map(String::from);

        CFRelease(container);

        opt_str
    }
}

#[cfg(target_os = "macos")]
/// Determine the serial port type based on the service object (like that returned by
/// `IOIteratorNext`). Specific properties are extracted for USB devices.
fn port_type(service: io_object_t) -> SerialPortType {
    let bluetooth_device_class_name = b"IOBluetoothSerialClient\0".as_ptr() as *const c_char;
    if let Some(usb_device) = get_parent_device_by_type(service, kIOUSBDeviceClassName()) {
        SerialPortType::UsbPort(UsbPortInfo {
            vid: get_int_property(usb_device, "idVendor", kCFNumberSInt16Type)
                .unwrap_or_default() as u16,
            pid: get_int_property(usb_device, "idProduct", kCFNumberSInt16Type)
                .unwrap_or_default() as u16,
            serial_number: get_string_property(usb_device, "USB Serial Number"),
            manufacturer: get_string_property(usb_device, "USB Vendor Name"),
            product: get_string_property(usb_device, "USB Product Name"),
        })
    } else if get_parent_device_by_type(service, bluetooth_device_class_name).is_some() {
        SerialPortType::BluetoothPort
    } else {
        SerialPortType::PciPort
    }
}

#[cfg(target_os = "macos")]
/// Scans the system for serial ports and returns a list of them.
/// The `SerialPortInfo` struct contains the name of the port which can be used for opening it.
pub fn available_ports() -> ::Result<Vec<SerialPortInfo>> {
    use IOKit_sys::*;
    use cf::*;
    use mach::port::{mach_port_t, MACH_PORT_NULL};
    use mach::kern_return::KERN_SUCCESS;

    let mut vec = Vec::new();
    unsafe {

        // Create a dictionary for specifying the search terms against the IOService
        let classes_to_match = IOServiceMatching(kIOSerialBSDServiceValue());
        if classes_to_match.is_null() {
            return Err(Error::new(
                ErrorKind::Unknown,
                "IOServiceMatching returned a NULL dictionary.",
            ));
        }

        // Populate the search dictionary with a single key/value pair indicating that we're
        // searching for serial devices matching the RS232 device type.
        let key = CFStringCreateWithCString(
            kCFAllocatorDefault,
            kIOSerialBSDTypeKey(),
            kCFStringEncodingUTF8,
        );
        if key.is_null() {
            return Err(Error::new(
                ErrorKind::Unknown,
                "Failed to allocate key string.",
            ));
        }
        let value = CFStringCreateWithCString(kCFAllocatorDefault,
                                              kIOSerialBSDAllTypes(),
                                              kCFStringEncodingUTF8);
        if value.is_null() {
            return Err(Error::new(
                ErrorKind::Unknown,
                "Failed to allocate value string.",
            ));
        }
        CFDictionarySetValue(classes_to_match, key as CFTypeRef, value as CFTypeRef);

        // Get an interface to IOKit
        let mut master_port: mach_port_t = MACH_PORT_NULL;
        let mut kern_result = IOMasterPort(MACH_PORT_NULL, &mut master_port);
        if kern_result != KERN_SUCCESS {
            return Err(Error::new(
                ErrorKind::Unknown,
                format!("ERROR: {}", kern_result),
            ));
        }

        // Run the search.
        let mut matching_services: io_iterator_t = mem::uninitialized();
        kern_result = IOServiceGetMatchingServices(
            kIOMasterPortDefault,
            classes_to_match,
            &mut matching_services,
        );
        if kern_result != KERN_SUCCESS {
            return Err(Error::new(
                ErrorKind::Unknown,
                format!("ERROR: {}", kern_result),
            ));
        }

        loop {
            // Grab the next result.
            let modem_service = IOIteratorNext(matching_services);

            // Break out if we've reached the end of the iterator
            if modem_service == MACH_PORT_NULL {
                break;
            }

            // Fetch all properties of the current search result item.
            let mut props = mem::uninitialized();
            let result = IORegistryEntryCreateCFProperties(
                modem_service,
                &mut props,
                kCFAllocatorDefault,
                0,
            );
            if result == KERN_SUCCESS {
                // We only care about the IODialinDevice, which is the device path for this port.
                let key = CString::new("IODialinDevice").unwrap();
                let key_cfstring = CFStringCreateWithCString(
                    kCFAllocatorDefault,
                    key.as_ptr(),
                    kCFStringEncodingUTF8,
                );
                let value = CFDictionaryGetValue(props, key_cfstring as *const c_void);

                let type_id = CFGetTypeID(value);
                if type_id == CFStringGetTypeID() {
                    let mut buf = Vec::with_capacity(256);

                    CFStringGetCString(
                        value as CFStringRef,
                        buf.as_mut_ptr(),
                        256,
                        kCFStringEncodingUTF8,
                    );
                    let path = CStr::from_ptr(buf.as_ptr()).to_string_lossy();
                    vec.push(SerialPortInfo {
                        port_name: path.to_string(),
                        port_type: port_type(modem_service),
                    });
                } else {
                    return Err(Error::new(
                        ErrorKind::Unknown,
                        "Found invalid type for TypeID",
                    ));
                }
            } else {
                return Err(Error::new(ErrorKind::Unknown, format!("ERROR: {}", result)));
            }

            // Clean up after we're done processing htis result
            IOObjectRelease(modem_service);
        }
    }
    Ok(vec)
}

#[cfg(not(any(target_os = "linux", target_os = "macos")))]
/// Enumerating serial ports on non-Linux POSIX platforms is not yet supported
pub fn available_ports() -> ::Result<Vec<SerialPortInfo>> {
    Err(Error::new(
        ErrorKind::Unknown,
        "Not implemented for this OS",
    ))
}

/// Returns a list of baud rates officially supported by this platform. It's likely more are
/// actually supported by the hardware however.
pub fn available_baud_rates() -> Vec<u32> {
    let mut vec = vec![50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400, 4800];
    #[cfg(any(target_os = "freebsd", target_os = "dragonfly", target_os = "macos",
                target_os = "netbsd", target_os = "openbsd"))]
    vec.push(7200);
    vec.push(9600);
    #[cfg(any(target_os = "freebsd", target_os = "dragonfly", target_os = "macos",
              target_os = "netbsd", target_os = "openbsd"))]
    vec.push(14_400);
    vec.push(19_200);
    #[cfg(any(target_os = "freebsd", target_os = "dragonfly", target_os = "macos",
              target_os = "netbsd", target_os = "openbsd"))]
    vec.push(28_800);
    vec.push(38_400);
    vec.push(57_600);
    #[cfg(any(target_os = "freebsd", target_os = "dragonfly", target_os = "macos",
              target_os = "netbsd", target_os = "openbsd"))]
    vec.push(76_800);
    vec.push(115_200);
    vec.push(230_400);
    #[cfg(any(target_os = "android", target_os = "freebsd", target_os = "linux"))]
    vec.push(460_800);
    #[cfg(any(target_os = "android", target_os = "linux"))]
    vec.push(500_000);
    #[cfg(any(target_os = "android", target_os = "linux"))]
    vec.push(576_000);
    #[cfg(any(target_os = "android", target_os = "linux", target_os = "netbsd"))]
    vec.push(921_600);
    #[cfg(any(target_os = "android", target_os = "linux"))]
    vec.push(1_000_000);
    #[cfg(any(target_os = "android", target_os = "linux"))]
    vec.push(1_152_000);
    #[cfg(any(target_os = "android", target_os = "linux"))]
    vec.push(1_500_000);
    #[cfg(any(target_os = "android", target_os = "linux"))]
    vec.push(2_000_000);
    #[cfg(any(target_os = "android", target_os = "linux"))]
    vec.push(2_500_000);
    #[cfg(any(target_os = "android", target_os = "linux"))]
    vec.push(3_000_000);
    #[cfg(any(target_os = "android", target_os = "linux"))]
    vec.push(3_500_000);
    #[cfg(any(target_os = "android", target_os = "linux"))]
    vec.push(4_000_000);
    vec
}

#[test]
fn test_ttyport_into_raw_fd() {
    // `master` must be used here as Dropping it causes slave to be deleted by the OS.
    // TODO: Convert this to a statement-level attribute once
    //       https://github.com/rust-lang/rust/issues/15701 is on stable.
    // FIXME: Create a mutex across all tests for using `TTYPort::pair()` as it's not threadsafe
    #![allow(unused_variables)]
    let (master, slave) = TTYPort::pair().expect("Unable to create ptty pair");

    // First test with the master
    let master_fd = master.into_raw_fd();
    let mut termios = unsafe { mem::uninitialized() };
    let res = unsafe { nix::libc::tcgetattr(master_fd, &mut termios) };
    if res != 0 {
        close(master_fd);
        panic!("tcgetattr on the master port failed");
    }

    // And then the slave
    let slave_fd = slave.into_raw_fd();
    let res = unsafe { nix::libc::tcgetattr(slave_fd, &mut termios) };
    if res != 0 {
        close(slave_fd);
        panic!("tcgetattr on the master port failed");
    }
    close(master_fd);
    close(slave_fd);
}
