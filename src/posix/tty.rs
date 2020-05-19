use std::mem::MaybeUninit;
use std::os::unix::prelude::*;
use std::path::Path;
use std::time::Duration;
use std::{io, mem};

use nix::fcntl::fcntl;
use nix::{self, libc, unistd};

use crate::posix::ioctl::{self, SerialLines};
use crate::{
    ClearBuffer, DataBits, Error, ErrorKind, FlowControl, Parity, Result, SerialPort,
    SerialPortBuilder, StopBits,
};

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

#[cfg(any(target_os = "ios", target_os = "macos"))]
fn is_nonstandard_baud(baud: u32) -> bool {
    match baud {
        0 | 50 | 75 | 110 | 134 | 150 | 200 | 300 | 600 | 1200 | 1800 | 2400 | 4800 | 7200
        | 9600 | 14400 | 19200 | 28800 | 38400 | 57600 | 76800 | 115200 | 230400 => false,
        _ => true,
    }
}

/// A serial port implementation for POSIX TTY ports
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
    #[cfg(any(target_os = "ios", target_os = "macos"))]
    baud_rate: u32,
}

/// Specifies the duration of a transmission break
#[derive(Clone, Copy, Debug)]
pub enum BreakDuration {
    /// 0.25-0.5s
    Short,
    /// Specifies a break duration that is platform-dependent
    Arbitrary(std::num::NonZeroI32),
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
    pub fn open(builder: &SerialPortBuilder) -> Result<TTYPort> {
        use nix::fcntl::FcntlArg::F_SETFL;
        use nix::fcntl::OFlag;
        use nix::libc::{cfmakeraw, tcflush, tcgetattr, tcsetattr};

        let path = Path::new(&builder.path);
        let fd = nix::fcntl::open(
            path,
            OFlag::O_RDWR | OFlag::O_NOCTTY | OFlag::O_NONBLOCK,
            nix::sys::stat::Mode::empty(),
        )?;

        let mut termios = MaybeUninit::uninit();
        let res = unsafe { tcgetattr(fd, termios.as_mut_ptr()) };
        if let Err(e) = nix::errno::Errno::result(res) {
            close(fd);
            return Err(e.into());
        }
        let mut termios = unsafe { termios.assume_init() };

        // If any of these steps fail, then we should abort creation of the
        // TTYPort and ensure the file descriptor is closed.
        // So we wrap these calls in a block and check the result.
        {
            // setup TTY for binary serial port access
            // Enable reading from the port and ignore all modem control lines
            termios.c_cflag |= libc::CREAD | libc::CLOCAL;
            // Enable raw mode which disables any implicit processing of the input or output data streams
            // This also sets no timeout period and a read will block until at least one character is
            // available.
            unsafe { cfmakeraw(&mut termios) };

            // write settings to TTY
            unsafe { tcsetattr(fd, libc::TCSANOW, &termios) };

            // Read back settings from port and confirm they were applied correctly
            let mut actual_termios = MaybeUninit::uninit();
            unsafe { tcgetattr(fd, actual_termios.as_mut_ptr()) };
            let actual_termios = unsafe { actual_termios.assume_init() };

            if actual_termios.c_iflag != termios.c_iflag
                || actual_termios.c_oflag != termios.c_oflag
                || actual_termios.c_lflag != termios.c_lflag
                || actual_termios.c_cflag != termios.c_cflag
            {
                return Err(Error::new(
                    ErrorKind::Unknown,
                    "Settings did not apply correctly",
                ));
            };

            unsafe { tcflush(fd, libc::TCIOFLUSH) };

            // get exclusive access to device
            ioctl::tiocexcl(fd)?;

            // clear O_NONBLOCK flag
            fcntl(fd, F_SETFL(nix::fcntl::OFlag::empty()))?;

            Ok(())
        }
        .map_err(|e: Error| {
            close(fd);
            e
        })?;

        let port = TTYPort {
            fd,
            timeout: builder.timeout, // This is overwritten by the subsequent call to `set_all()`
            exclusive: true,          // This is guaranteed by the above `ioctl::tiocexcl()` call
            port_name: Some(builder.path.clone()),
            #[cfg(any(target_os = "ios", target_os = "macos"))]
            baud_rate: builder.baud_rate,
        };

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
    pub fn set_exclusive(&mut self, exclusive: bool) -> Result<()> {
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

    fn set_pin(&mut self, pin: ioctl::SerialLines, level: bool) -> Result<()> {
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

    fn read_pin(&mut self, pin: ioctl::SerialLines) -> Result<bool> {
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
    /// use serialport::TTYPort;
    ///
    /// let (master, slave) = TTYPort::pair().unwrap();
    /// ```
    pub fn pair() -> Result<(Self, Self)> {
        // Open the next free pty.
        let next_pty_fd = nix::pty::posix_openpt(nix::fcntl::OFlag::O_RDWR)?;

        // Grant access to the associated slave pty
        nix::pty::grantpt(&next_pty_fd)?;

        // Unlock the slave pty
        nix::pty::unlockpt(&next_pty_fd)?;

        // Get the path of the attached slave ptty
        #[cfg(not(any(
            target_os = "linux",
            target_os = "android",
            target_os = "emscripten",
            target_os = "fuchsia"
        )))]
        let ptty_name = unsafe { nix::pty::ptsname(&next_pty_fd)? };

        #[cfg(any(
            target_os = "linux",
            target_os = "android",
            target_os = "emscripten",
            target_os = "fuchsia"
        ))]
        let ptty_name = nix::pty::ptsname_r(&next_pty_fd)?;

        // Open the slave port
        let baud_rate = 9600;
        let builder = crate::new(ptty_name, baud_rate).timeout(Duration::from_millis(1));
        let slave_tty = TTYPort::open(&builder)?;

        // Manually construct the master port here because the
        // `tcgetattr()` doesn't work on Mac, Solaris, and maybe other
        // BSDs when used on the master port.
        let master_tty = TTYPort {
            fd: next_pty_fd.into_raw_fd(),
            timeout: Duration::from_millis(100),
            exclusive: true,
            port_name: None,
            #[cfg(any(target_os = "ios", target_os = "macos"))]
            baud_rate,
        };

        Ok((master_tty, slave_tty))
    }

    /// Sends 0-valued bits over the port for a set duration
    pub fn send_break(&self, duration: BreakDuration) -> Result<()> {
        match duration {
            BreakDuration::Short => nix::sys::termios::tcsendbreak(self.fd, 0),
            BreakDuration::Arbitrary(n) => nix::sys::termios::tcsendbreak(self.fd, n.get()),
        }
        .map_err(|e| e.into())
    }

    #[cfg(any(
        target_os = "dragonflybsd",
        target_os = "freebsd",
        target_os = "ios",
        target_os = "macos",
        target_os = "netbsd",
        target_os = "openbsd",
        all(
            target_os = "linux",
            any(
                target_env = "musl",
                target_arch = "powerpc",
                target_arch = "powerpc64"
            )
        )
    ))]
    fn get_termios(&self) -> Result<libc::termios> {
        let mut termios = MaybeUninit::uninit();
        let res = unsafe { libc::tcgetattr(self.fd, termios.as_mut_ptr()) };
        nix::errno::Errno::result(res)?;
        unsafe { Ok(termios.assume_init()) }
    }

    #[cfg(any(
        target_os = "android",
        all(
            target_os = "linux",
            not(any(
                target_env = "musl",
                target_arch = "powerpc",
                target_arch = "powerpc64"
            ))
        )
    ))]
    fn get_termios(&self) -> Result<libc::termios2> {
        ioctl::tcgets2(self.fd)
    }

    #[cfg(any(
        target_os = "dragonflybsd",
        target_os = "freebsd",
        target_os = "netbsd",
        target_os = "openbsd",
        all(
            target_os = "linux",
            any(
                target_env = "musl",
                target_arch = "powerpc",
                target_arch = "powerpc64"
            )
        )
    ))]
    fn set_termios(&self, termios: &libc::termios) -> Result<()> {
        let res = unsafe { libc::tcsetattr(self.fd, libc::TCSANOW, termios) };
        nix::errno::Errno::result(res)?;
        Ok(())
    }

    // On mac, we ignore the speed from the termios struct and instead use the one that's
    // specified in the `TTYPort`.
    #[cfg(any(target_os = "ios", target_os = "macos"))]
    fn set_termios(&self, termios: &libc::termios) -> Result<()> {
        // For non-standard baud rates, we use a dummy baud rate of 9600 when setting the termios
        // struct because the baud rate will actually be set by a subsequent call to the
        // `iossiospeed` ioctl.
        if is_nonstandard_baud(self.baud_rate) {
            let mut termios = termios.clone();
            termios.c_ispeed = 9600;
            termios.c_ospeed = 9600;

            let res = unsafe { libc::tcsetattr(self.fd, libc::TCSANOW, &termios) };
            nix::errno::Errno::result(res)?;

            ioctl::iossiospeed(self.fd, &(self.baud_rate as libc::speed_t))
        } else {
            let mut termios = termios.clone();
            termios.c_ispeed = self.baud_rate as libc::speed_t;
            termios.c_ospeed = self.baud_rate as libc::speed_t;

            let res = unsafe { libc::tcsetattr(self.fd, libc::TCSANOW, &termios) };
            nix::errno::Errno::result(res)?;
            Ok(())
        }
    }

    #[cfg(any(
        target_os = "android",
        all(
            target_os = "linux",
            not(any(
                target_env = "musl",
                target_arch = "powerpc",
                target_arch = "powerpc64"
            ))
        )
    ))]
    fn set_termios(&self, termios2: &libc::termios2) -> Result<()> {
        ioctl::tcsets2(self.fd, &termios2)
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

/// Get the baud speed for a port from its file descriptor
#[cfg(any(target_os = "ios", target_os = "macos"))]
fn get_termios_speed(fd: RawFd) -> u32 {
    let mut termios = MaybeUninit::uninit();
    let res = unsafe { libc::tcgetattr(fd, termios.as_mut_ptr()) };
    nix::errno::Errno::result(res).expect("Failed to get termios data");
    let termios = unsafe { termios.assume_init() };
    assert_eq!(termios.c_ospeed, termios.c_ispeed);
    termios.c_ospeed as u32
}

impl FromRawFd for TTYPort {
    unsafe fn from_raw_fd(fd: RawFd) -> Self {
        // Try to set exclusive, as is the default setting.  Catch errors.. this method MUST
        // return a TTYPort so we'll just indicate non-exclusive on an error here.
        let exclusive = match ioctl::tiocexcl(fd) {
            Ok(_) => true,
            Err(_) => false,
        };

        TTYPort {
            fd,
            timeout: Duration::from_millis(100),
            exclusive,
            // It is not trivial to get the file path corresponding to a file descriptor.
            // We'll punt on it and set it to `None` here.
            port_name: None,
            // It's not guaranteed that the baud rate in the `termios` struct is correct, as
            // setting an arbitrary baud rate via the `iossiospeed` ioctl overrides that value,
            // but extract that value anyways as a best-guess of the actual baud rate.
            #[cfg(any(target_os = "ios", target_os = "macos"))]
            baud_rate: get_termios_speed(fd),
        }
    }
}

impl io::Read for TTYPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if let Err(e) = super::poll::wait_read_fd(self.fd, self.timeout) {
            return Err(io::Error::from(Error::from(e)));
        }

        match nix::unistd::read(self.fd, buf) {
            Ok(n) => Ok(n),
            Err(e) => Err(io::Error::from(Error::from(e))),
        }
    }
}

impl io::Write for TTYPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if let Err(e) = super::poll::wait_write_fd(self.fd, self.timeout) {
            return Err(io::Error::from(Error::from(e)));
        }

        match nix::unistd::write(self.fd, buf) {
            Ok(n) => Ok(n),
            Err(e) => Err(io::Error::from(Error::from(e))),
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        nix::sys::termios::tcdrain(self.fd)
            .map_err(|_| io::Error::new(io::ErrorKind::Other, "flush failed"))
    }
}

impl SerialPort for TTYPort {
    fn name(&self) -> Option<String> {
        self.port_name.clone()
    }

    /// Returns the port's baud rate
    ///
    /// On some platforms this will be the actual device baud rate, which may differ from the
    /// desired baud rate.
    #[cfg(any(
        target_os = "android",
        all(
            target_os = "linux",
            not(any(
                target_env = "musl",
                target_arch = "powerpc",
                target_arch = "powerpc64"
            ))
        )
    ))]
    fn baud_rate(&self) -> Result<u32> {
        let termios2 = ioctl::tcgets2(self.fd)?;

        assert!(termios2.c_ospeed == termios2.c_ispeed);

        Ok(termios2.c_ospeed as u32)
    }

    /// Returns the port's baud rate
    ///
    /// On some platforms this will be the actual device baud rate, which may differ from the
    /// desired baud rate.
    #[cfg(any(
        target_os = "dragonflybsd",
        target_os = "freebsd",
        target_os = "netbsd",
        target_os = "openbsd"
    ))]
    fn baud_rate(&self) -> Result<u32> {
        let termios = self.get_termios()?;

        let ospeed = unsafe { libc::cfgetospeed(&termios) };
        let ispeed = unsafe { libc::cfgetispeed(&termios) };

        assert!(ospeed == ispeed);

        Ok(ospeed as u32)
    }

    /// Returns the port's baud rate
    ///
    /// On some platforms this will be the actual device baud rate, which may differ from the
    /// desired baud rate.
    #[cfg(any(target_os = "ios", target_os = "macos"))]
    fn baud_rate(&self) -> Result<u32> {
        Ok(self.baud_rate)
    }

    /// Returns the port's baud rate
    ///
    /// On some platforms this will be the actual device baud rate, which may differ from the
    /// desired baud rate.
    #[cfg(all(
        target_os = "linux",
        any(
            target_env = "musl",
            target_arch = "powerpc",
            target_arch = "powerpc64"
        )
    ))]
    fn baud_rate(&self) -> Result<u32> {
        use self::libc::{
            B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000,
            B460800, B500000, B576000, B921600,
        };
        use self::libc::{
            B110, B115200, B1200, B134, B150, B1800, B19200, B200, B230400, B2400, B300, B38400,
            B4800, B50, B57600, B600, B75, B9600,
        };

        let termios = self.get_termios()?;
        let ospeed = unsafe { libc::cfgetospeed(&termios) };
        let ispeed = unsafe { libc::cfgetispeed(&termios) };

        assert!(ospeed == ispeed);

        let res: u32 = match ospeed {
            B50 => 50,
            B75 => 75,
            B110 => 110,
            B134 => 134,
            B150 => 150,
            B200 => 200,
            B300 => 300,
            B600 => 600,
            B1200 => 1200,
            B1800 => 1800,
            B2400 => 2400,
            B4800 => 4800,
            B9600 => 9600,
            B19200 => 19_200,
            B38400 => 38_400,
            B57600 => 57_600,
            B115200 => 115_200,
            B230400 => 230_400,
            B460800 => 460_800,
            B500000 => 500_000,
            B576000 => 576_000,
            B921600 => 921_600,
            B1000000 => 1_000_000,
            B1152000 => 1_152_000,
            B1500000 => 1_500_000,
            B2000000 => 2_000_000,
            B2500000 => 2_500_000,
            B3000000 => 3_000_000,
            B3500000 => 3_500_000,
            B4000000 => 4_000_000,
            _ => unreachable!(),
        };

        Ok(res)
    }

    fn data_bits(&self) -> Result<DataBits> {
        let termios = self.get_termios()?;
        match termios.c_cflag & libc::CSIZE {
            libc::CS8 => Ok(DataBits::Eight),
            libc::CS7 => Ok(DataBits::Seven),
            libc::CS6 => Ok(DataBits::Six),
            libc::CS5 => Ok(DataBits::Five),
            _ => Err(Error::new(
                ErrorKind::Unknown,
                "Invalid data bits setting encountered",
            )),
        }
    }

    fn flow_control(&self) -> Result<FlowControl> {
        let termios = self.get_termios()?;
        if termios.c_cflag & libc::CRTSCTS == libc::CRTSCTS {
            Ok(FlowControl::Hardware)
        } else if termios.c_iflag & (libc::IXON | libc::IXOFF) == (libc::IXON | libc::IXOFF) {
            Ok(FlowControl::Software)
        } else {
            Ok(FlowControl::None)
        }
    }

    fn parity(&self) -> Result<Parity> {
        let termios = self.get_termios()?;
        if termios.c_cflag & libc::PARENB == libc::PARENB {
            if termios.c_cflag & libc::PARODD == libc::PARODD {
                Ok(Parity::Odd)
            } else {
                Ok(Parity::Even)
            }
        } else {
            Ok(Parity::None)
        }
    }

    fn stop_bits(&self) -> Result<StopBits> {
        let termios = self.get_termios()?;
        if termios.c_cflag & libc::CSTOPB == libc::CSTOPB {
            Ok(StopBits::Two)
        } else {
            Ok(StopBits::One)
        }
    }

    fn timeout(&self) -> Duration {
        self.timeout
    }

    #[cfg(any(
        target_os = "android",
        all(
            target_os = "linux",
            not(any(
                target_env = "musl",
                target_arch = "powerpc",
                target_arch = "powerpc64"
            ))
        )
    ))]
    fn set_baud_rate(&mut self, baud_rate: u32) -> Result<()> {
        let mut termios2 = ioctl::tcgets2(self.fd)?;
        termios2.c_cflag &= !nix::libc::CBAUD;
        termios2.c_cflag |= nix::libc::BOTHER;
        termios2.c_ispeed = baud_rate;
        termios2.c_ospeed = baud_rate;

        ioctl::tcsets2(self.fd, &termios2)
    }

    // BSDs use the baud rate as the constant value so there's no translation necessary
    #[cfg(any(
        target_os = "dragonflybsd",
        target_os = "freebsd",
        target_os = "netbsd",
        target_os = "openbsd"
    ))]
    fn set_baud_rate(&mut self, baud_rate: u32) -> Result<()> {
        let mut termios = self.get_termios()?;
        let res = unsafe { libc::cfsetspeed(&mut termios, baud_rate.into()) };
        nix::errno::Errno::result(res)?;
        self.set_termios(&termios)
    }

    #[cfg(all(
        target_os = "linux",
        any(
            target_env = "musl",
            target_arch = "powerpc",
            target_arch = "powerpc64"
        )
    ))]
    fn set_baud_rate(&mut self, baud_rate: u32) -> Result<()> {
        use self::libc::{
            B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000,
            B460800, B500000, B576000, B921600,
        };
        use self::libc::{
            B110, B115200, B1200, B134, B150, B1800, B19200, B200, B230400, B2400, B300, B38400,
            B4800, B50, B57600, B600, B75, B9600,
        };

        let mut termios = self.get_termios()?;

        let baud_rate = match baud_rate {
            50 => B50,
            75 => B75,
            110 => B110,
            134 => B134,
            150 => B150,
            200 => B200,
            300 => B300,
            600 => B600,
            1200 => B1200,
            1800 => B1800,
            2400 => B2400,
            4800 => B4800,
            9600 => B9600,
            19_200 => B19200,
            38_400 => B38400,
            57_600 => B57600,
            115_200 => B115200,
            230_400 => B230400,
            460_800 => B460800,
            500_000 => B500000,
            576_000 => B576000,
            921_600 => B921600,
            1_000_000 => B1000000,
            1_152_000 => B1152000,
            1_500_000 => B1500000,
            2_000_000 => B2000000,
            2_500_000 => B2500000,
            3_000_000 => B3000000,
            3_500_000 => B3500000,
            4_000_000 => B4000000,
            _ => return Err(Error::new(ErrorKind::InvalidInput, "invalid baud rate")),
        };
        let res = unsafe { libc::cfsetspeed(&mut termios, baud_rate) };
        nix::errno::Errno::result(res)?;
        self.set_termios(&termios)
    }

    // Mac OS needs special logic for setting arbitrary baud rates.
    #[cfg(any(target_os = "ios", target_os = "macos"))]
    fn set_baud_rate(&mut self, baud_rate: u32) -> Result<()> {
        self.baud_rate = baud_rate;
        let termios = self.get_termios()?;
        self.set_termios(&termios)
    }

    fn set_flow_control(&mut self, flow_control: FlowControl) -> Result<()> {
        let mut termios = self.get_termios()?;
        match flow_control {
            FlowControl::None => {
                termios.c_iflag &= !(libc::IXON | libc::IXOFF);
                termios.c_cflag &= !libc::CRTSCTS;
            }
            FlowControl::Software => {
                termios.c_iflag |= libc::IXON | libc::IXOFF;
                termios.c_cflag &= !libc::CRTSCTS;
            }
            FlowControl::Hardware => {
                termios.c_iflag &= !(libc::IXON | libc::IXOFF);
                termios.c_cflag |= libc::CRTSCTS;
            }
        };
        self.set_termios(&termios)
    }

    fn set_parity(&mut self, parity: Parity) -> Result<()> {
        let mut termios = self.get_termios()?;
        match parity {
            Parity::None => {
                termios.c_cflag &= !(libc::PARENB | libc::PARODD);
                termios.c_iflag &= !libc::INPCK;
                termios.c_iflag |= libc::IGNPAR;
            }
            Parity::Odd => {
                termios.c_cflag |= libc::PARENB | libc::PARODD;
                termios.c_iflag |= libc::INPCK;
                termios.c_iflag &= !libc::IGNPAR;
            }
            Parity::Even => {
                termios.c_cflag &= !libc::PARODD;
                termios.c_cflag |= libc::PARENB;
                termios.c_iflag |= libc::INPCK;
                termios.c_iflag &= !libc::IGNPAR;
            }
        };
        self.set_termios(&termios)
    }

    fn set_data_bits(&mut self, data_bits: DataBits) -> Result<()> {
        let size = match data_bits {
            DataBits::Five => libc::CS5,
            DataBits::Six => libc::CS6,
            DataBits::Seven => libc::CS7,
            DataBits::Eight => libc::CS8,
        };

        let mut termios = self.get_termios()?;
        termios.c_cflag &= !libc::CSIZE;
        termios.c_cflag |= size;
        self.set_termios(&termios)
    }

    fn set_stop_bits(&mut self, stop_bits: StopBits) -> Result<()> {
        let mut termios = self.get_termios()?;
        match stop_bits {
            StopBits::One => termios.c_cflag &= !libc::CSTOPB,
            StopBits::Two => termios.c_cflag |= libc::CSTOPB,
        };
        self.set_termios(&termios)
    }

    fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        self.timeout = timeout;
        Ok(())
    }

    fn write_request_to_send(&mut self, level: bool) -> Result<()> {
        self.set_pin(SerialLines::REQUEST_TO_SEND, level)
    }

    fn write_data_terminal_ready(&mut self, level: bool) -> Result<()> {
        self.set_pin(SerialLines::DATA_TERMINAL_READY, level)
    }

    fn read_clear_to_send(&mut self) -> Result<bool> {
        self.read_pin(SerialLines::CLEAR_TO_SEND)
    }

    fn read_data_set_ready(&mut self) -> Result<bool> {
        self.read_pin(SerialLines::DATA_SET_READY)
    }

    fn read_ring_indicator(&mut self) -> Result<bool> {
        self.read_pin(SerialLines::RING)
    }

    fn read_carrier_detect(&mut self) -> Result<bool> {
        self.read_pin(SerialLines::DATA_CARRIER_DETECT)
    }

    fn bytes_to_read(&self) -> Result<u32> {
        ioctl::fionread(self.fd)
    }

    fn bytes_to_write(&self) -> Result<u32> {
        ioctl::tiocoutq(self.fd)
    }

    fn clear(&self, buffer_to_clear: ClearBuffer) -> Result<()> {
        let buffer_id = match buffer_to_clear {
            ClearBuffer::Input => libc::TCIFLUSH,
            ClearBuffer::Output => libc::TCOFLUSH,
            ClearBuffer::All => libc::TCIOFLUSH,
        };

        let res = unsafe { nix::libc::tcflush(self.fd, buffer_id) };

        nix::errno::Errno::result(res)
            .map(|_| ())
            .map_err(|e| e.into())
    }

    fn try_clone(&self) -> Result<Box<dyn SerialPort>> {
        let fd_cloned: i32 = fcntl(self.fd, nix::fcntl::F_DUPFD(self.fd))?;
        Ok(Box::new(TTYPort {
            fd: fd_cloned,
            exclusive: self.exclusive,
            port_name: self.port_name.clone(),
            timeout: self.timeout,
            #[cfg(any(target_os = "ios", target_os = "macos"))]
            baud_rate: self.baud_rate,
        }))
    }
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
    let mut termios = MaybeUninit::uninit();
    let res = unsafe { nix::libc::tcgetattr(master_fd, termios.as_mut_ptr()) };
    if res != 0 {
        close(master_fd);
        panic!("tcgetattr on the master port failed");
    }

    // And then the slave
    let slave_fd = slave.into_raw_fd();
    let res = unsafe { nix::libc::tcgetattr(slave_fd, termios.as_mut_ptr()) };
    if res != 0 {
        close(slave_fd);
        panic!("tcgetattr on the master port failed");
    }
    close(master_fd);
    close(slave_fd);
}
