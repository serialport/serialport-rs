use std::mem::MaybeUninit;
use std::os::unix::prelude::*;
use std::path::Path;
use std::time::Duration;
use std::{io, mem};

use nix::fcntl::{fcntl, OFlag};
use nix::{self, libc, unistd};

use crate::posix::{BreakDuration, SerialPortExt};
use crate::sys::posix::ioctl::{self, SerialLines};
use crate::sys::posix::termios;
use crate::{
    ClearBuffer, DataBits, Error, ErrorKind, FlowControl, Parity, Result, SerialPortBuilder,
    StopBits,
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

/// A serial port implementation for POSIX TTY ports
///
/// The port will be closed when the value is dropped. However, this struct
/// should not be instantiated directly by using `SerialPort::open()`, instead use
/// the cross-platform `serialport::open()` or
/// `serialport::open_with_settings()`.
#[derive(Debug)]
pub struct SerialPort {
    fd: RawFd,
    read_timeout: Option<Duration>,
    write_timeout: Option<Duration>,
    exclusive: bool,
    port_name: Option<String>,
    #[cfg(any(target_os = "ios", target_os = "macos"))]
    baud_rate: u32,
}

impl SerialPort {
    /// Opens a TTY device as a serial port.
    ///
    /// `path` should be the path to a TTY device, e.g., `/dev/ttyS0`.
    ///
    /// Ports are opened in exclusive mode by default. If this is undesireable
    /// behavior, use `SerialPort::set_exclusive(false)`.
    ///
    /// ## Errors
    ///
    /// * `NoDevice` if the device could not be opened. This could indicate that
    ///    the device is already in use.
    /// * `InvalidInput` if `path` is not a valid device name.
    /// * `Io` for any other error while opening or initializing the device.
    pub fn open(builder: SerialPortBuilder, path: impl AsRef<Path>) -> Result<SerialPort> {
        use nix::fcntl::FcntlArg::F_SETFL;
        use nix::libc::{cfmakeraw, tcflush, tcgetattr, tcsetattr};

        let path = path.as_ref();
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
        // SerialPort and ensure the file descriptor is closed.
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

            // clear O_NONBLOCK flag
            fcntl(fd, F_SETFL(nix::fcntl::OFlag::empty()))?;

            Ok(())
        }
        .map_err(|e: Error| {
            close(fd);
            e
        })?;

        // Configure the low-level port settings
        let mut termios = termios::get_termios(fd)?;
        termios::set_parity(&mut termios, builder.parity);
        termios::set_flow_control(&mut termios, builder.flow_control);
        termios::set_data_bits(&mut termios, builder.data_bits);
        termios::set_stop_bits(&mut termios, builder.stop_bits);
        #[cfg(not(any(target_os = "ios", target_os = "macos")))]
        termios::set_baud_rate(&mut termios, builder.baud_rate);
        #[cfg(any(target_os = "ios", target_os = "macos"))]
        termios::set_termios(fd, &termios, builder.baud_rate)?;
        #[cfg(not(any(target_os = "ios", target_os = "macos")))]
        termios::set_termios(fd, &termios)?;

        // Return the final port object
        Ok(SerialPort {
            fd,
            read_timeout: builder.read_timeout,
            write_timeout: builder.write_timeout,
            exclusive: false,
            port_name: Some(path.to_string_lossy().into_owned()),
            #[cfg(any(target_os = "ios", target_os = "macos"))]
            baud_rate: builder.baud_rate,
        })
    }

    fn set_pin(&mut self, pin: ioctl::SerialLines, level: bool) -> Result<()> {
        if level {
            ioctl::tiocmbis(self.fd, pin)
        } else {
            ioctl::tiocmbic(self.fd, pin)
        }
    }

    fn read_pin(&mut self, pin: ioctl::SerialLines) -> Result<bool> {
        ioctl::tiocmget(self.fd).map(|pins| pins.contains(pin))
    }

    /// Attempts to clone the `SerialPort`. This allow you to write and read simultaneously from the
    /// same serial connection. Please note that if you want a real asynchronous serial port you
    /// should look at [mio-serial](https://crates.io/crates/mio-serial) or
    /// [tokio-serial](https://crates.io/crates/tokio-serial).
    ///
    /// Also, you must be very careful when changing the settings of a cloned `SerialPort` : since
    /// the settings are cached on a per object basis, trying to modify them from two different
    /// objects can cause some nasty behavior.
    ///
    /// # Errors
    ///
    /// This function returns an error if the serial port couldn't be cloned.
    pub fn try_clone(&self) -> Result<Self> {
        let fd_cloned: i32 = fcntl(self.fd, nix::fcntl::F_DUPFD(self.fd))?;
        Ok(SerialPort {
            fd: fd_cloned,
            exclusive: self.exclusive,
            port_name: self.port_name.clone(),
            read_timeout: self.read_timeout,
            write_timeout: self.write_timeout,
            #[cfg(any(target_os = "ios", target_os = "macos"))]
            baud_rate: self.baud_rate,
        })
    }

    pub fn name(&self) -> Option<&str> {
        self.port_name.as_ref().map(|s| &**s)
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
    pub fn baud_rate(&self) -> Result<u32> {
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
    pub fn baud_rate(&self) -> Result<u32> {
        let termios = termios::get_termios(self.fd)?;

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
    pub fn baud_rate(&self) -> Result<u32> {
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
    pub fn baud_rate(&self) -> Result<u32> {
        use self::libc::{
            B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000,
            B460800, B500000, B576000, B921600,
        };
        use self::libc::{
            B110, B115200, B1200, B134, B150, B1800, B19200, B200, B230400, B2400, B300, B38400,
            B4800, B50, B57600, B600, B75, B9600,
        };

        let termios = termios::get_termios(self.fd)?;
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

    pub fn data_bits(&self) -> Result<DataBits> {
        let termios = termios::get_termios(self.fd)?;
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

    pub fn flow_control(&self) -> Result<FlowControl> {
        let termios = termios::get_termios(self.fd)?;
        if termios.c_cflag & libc::CRTSCTS == libc::CRTSCTS {
            Ok(FlowControl::Hardware)
        } else if termios.c_iflag & (libc::IXON | libc::IXOFF) == (libc::IXON | libc::IXOFF) {
            Ok(FlowControl::Software)
        } else {
            Ok(FlowControl::None)
        }
    }

    pub fn parity(&self) -> Result<Parity> {
        let termios = termios::get_termios(self.fd)?;
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

    pub fn stop_bits(&self) -> Result<StopBits> {
        let termios = termios::get_termios(self.fd)?;
        if termios.c_cflag & libc::CSTOPB == libc::CSTOPB {
            Ok(StopBits::Two)
        } else {
            Ok(StopBits::One)
        }
    }

    pub fn read_timeout(&self) -> Option<Duration> {
        self.read_timeout
    }

    pub fn write_timeout(&self) -> Option<Duration> {
        self.write_timeout
    }

    #[cfg(any(
        target_os = "android",
        target_os = "dragonflybsd",
        target_os = "freebsd",
        target_os = "netbsd",
        target_os = "openbsd",
        target_os = "linux"
    ))]
    pub fn set_baud_rate(&mut self, baud_rate: u32) -> Result<()> {
        let mut termios = termios::get_termios(self.fd)?;
        termios::set_baud_rate(&mut termios, baud_rate);
        termios::set_termios(self.fd, &termios)
    }

    // Mac OS needs special logic for setting arbitrary baud rates.
    #[cfg(any(target_os = "ios", target_os = "macos"))]
    pub fn set_baud_rate(&mut self, baud_rate: u32) -> Result<()> {
        ioctl::iossiospeed(self.fd, &(baud_rate as libc::speed_t))?;
        self.baud_rate = baud_rate;
        Ok(())
    }

    pub fn set_flow_control(&mut self, flow_control: FlowControl) -> Result<()> {
        let mut termios = termios::get_termios(self.fd)?;
        termios::set_flow_control(&mut termios, flow_control);
        #[cfg(any(target_os = "ios", target_os = "macos"))]
        return termios::set_termios(self.fd, &termios, self.baud_rate);
        #[cfg(not(any(target_os = "ios", target_os = "macos")))]
        return termios::set_termios(self.fd, &termios);
    }

    pub fn set_parity(&mut self, parity: Parity) -> Result<()> {
        let mut termios = termios::get_termios(self.fd)?;
        termios::set_parity(&mut termios, parity);
        #[cfg(any(target_os = "ios", target_os = "macos"))]
        return termios::set_termios(self.fd, &termios, self.baud_rate);
        #[cfg(not(any(target_os = "ios", target_os = "macos")))]
        return termios::set_termios(self.fd, &termios);
    }

    pub fn set_data_bits(&mut self, data_bits: DataBits) -> Result<()> {
        let mut termios = termios::get_termios(self.fd)?;
        termios::set_data_bits(&mut termios, data_bits);
        #[cfg(any(target_os = "ios", target_os = "macos"))]
        return termios::set_termios(self.fd, &termios, self.baud_rate);
        #[cfg(not(any(target_os = "ios", target_os = "macos")))]
        return termios::set_termios(self.fd, &termios);
    }

    pub fn set_stop_bits(&mut self, stop_bits: StopBits) -> Result<()> {
        let mut termios = termios::get_termios(self.fd)?;
        termios::set_stop_bits(&mut termios, stop_bits);
        #[cfg(any(target_os = "ios", target_os = "macos"))]
        return termios::set_termios(self.fd, &termios, self.baud_rate);
        #[cfg(not(any(target_os = "ios", target_os = "macos")))]
        return termios::set_termios(self.fd, &termios);
    }

    pub fn set_read_timeout(&mut self, read_timeout: Option<Duration>) -> Result<()> {
        self.read_timeout = read_timeout;
        Ok(())
    }

    pub fn set_write_timeout(&mut self, write_timeout: Option<Duration>) -> Result<()> {
        self.write_timeout = write_timeout;
        Ok(())
    }

    pub fn write_request_to_send(&mut self, level: bool) -> Result<()> {
        self.set_pin(SerialLines::REQUEST_TO_SEND, level)
    }

    pub fn write_data_terminal_ready(&mut self, level: bool) -> Result<()> {
        self.set_pin(SerialLines::DATA_TERMINAL_READY, level)
    }

    pub fn read_clear_to_send(&mut self) -> Result<bool> {
        self.read_pin(SerialLines::CLEAR_TO_SEND)
    }

    pub fn read_data_set_ready(&mut self) -> Result<bool> {
        self.read_pin(SerialLines::DATA_SET_READY)
    }

    pub fn read_ring_indicator(&mut self) -> Result<bool> {
        self.read_pin(SerialLines::RING)
    }

    pub fn read_carrier_detect(&mut self) -> Result<bool> {
        self.read_pin(SerialLines::DATA_CARRIER_DETECT)
    }

    pub fn bytes_to_read(&self) -> Result<u32> {
        ioctl::fionread(self.fd)
    }

    pub fn bytes_to_write(&self) -> Result<u32> {
        ioctl::tiocoutq(self.fd)
    }

    pub fn clear(&self, buffer_to_clear: ClearBuffer) -> Result<()> {
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

    pub fn set_break(&self) -> Result<()> {
        ioctl::tiocsbrk(self.fd)
    }

    pub fn clear_break(&self) -> Result<()> {
        ioctl::tioccbrk(self.fd)
    }
}

impl Drop for SerialPort {
    fn drop(&mut self) {
        close(self.fd);
    }
}

impl AsRawFd for SerialPort {
    fn as_raw_fd(&self) -> RawFd {
        self.fd
    }
}

impl AsRawFd for crate::SerialPort {
    fn as_raw_fd(&self) -> RawFd {
        self.0.as_raw_fd()
    }
}

impl IntoRawFd for SerialPort {
    fn into_raw_fd(mut self) -> RawFd {
        // into_raw_fd needs to remove the file descriptor from the `SerialPort`
        // to return it, but also needs to prevent Drop from being called, since
        // that would close the file descriptor and make `into_raw_fd` unusuable.
        // However, we also want to avoid leaking the rest of the contents of the
        // struct, so we either need to take it out or be sure it doesn't need to
        // be dropped.
        let fd = self.fd;
        // Currently port_name is the only field that needs to be dropped, and we
        // can do that by taking it out of the optional before we forget the struct.
        self.port_name.take();
        mem::forget(self);
        fd
    }
}

impl IntoRawFd for crate::SerialPort {
    fn into_raw_fd(self) -> RawFd {
        // crate::SerialPort doesn't explicitly implement Drop, so we can just take
        // out the inner value.
        self.0.into_raw_fd()
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

impl FromRawFd for SerialPort {
    unsafe fn from_raw_fd(fd: RawFd) -> Self {
        SerialPort {
            fd,
            read_timeout: None,
            write_timeout: None,
            exclusive: ioctl::tiocexcl(fd).is_ok(),
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

impl FromRawFd for crate::SerialPort {
    unsafe fn from_raw_fd(fd: RawFd) -> Self {
        crate::SerialPort(SerialPort::from_raw_fd(fd))
    }
}

impl io::Read for &SerialPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if let Err(e) = super::poll::wait_read_fd(self.fd, self.read_timeout) {
            return Err(io::Error::from(Error::from(e)));
        }

        nix::unistd::read(self.fd, buf).map_err(|e| io::Error::from(Error::from(e)))
    }
}

impl io::Write for &SerialPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if let Err(e) = super::poll::wait_write_fd(self.fd, self.write_timeout) {
            return Err(io::Error::from(Error::from(e)));
        }

        nix::unistd::write(self.fd, buf).map_err(|e| io::Error::from(Error::from(e)))
    }

    fn flush(&mut self) -> io::Result<()> {
        nix::sys::termios::tcdrain(self.fd)
            .map_err(|_| io::Error::new(io::ErrorKind::Other, "flush failed"))
    }
}

impl SerialPortExt for SerialPort {
    /// Create a pair of pseudo serial terminals
    ///
    /// ## Returns
    /// Two connected `SerialPort` objects: `(master, slave)`
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
    /// use serialport::{SerialPort, posix::SerialPortExt};
    ///
    /// let (master, slave) = SerialPort::pair().unwrap();
    /// ```
    fn pair() -> Result<(Self, Self)> {
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
        #[cfg(any(target_os = "ios", target_os = "macos"))]
        let baud_rate = 9600;
        let fd = nix::fcntl::open(
            Path::new(&ptty_name),
            OFlag::O_RDWR | OFlag::O_NOCTTY | OFlag::O_NONBLOCK,
            nix::sys::stat::Mode::empty(),
        )?;

        // Set the port to a raw state. Using these ports will not work without this.
        let mut termios = MaybeUninit::uninit();
        let res = unsafe { crate::sys::posix::tty::libc::tcgetattr(fd, termios.as_mut_ptr()) };
        if let Err(e) = nix::errno::Errno::result(res) {
            close(fd);
            return Err(e.into());
        }
        let mut termios = unsafe { termios.assume_init() };
        unsafe { crate::sys::posix::tty::libc::cfmakeraw(&mut termios) };
        unsafe { crate::sys::posix::tty::libc::tcsetattr(fd, libc::TCSANOW, &termios) };

        fcntl(
            fd,
            nix::fcntl::FcntlArg::F_SETFL(nix::fcntl::OFlag::empty()),
        )?;

        let slave_tty = SerialPort {
            fd,
            read_timeout: None,
            write_timeout: None,
            exclusive: true,
            port_name: Some(ptty_name),
            #[cfg(any(target_os = "ios", target_os = "macos"))]
            baud_rate,
        };

        // Manually construct the master port here because the
        // `tcgetattr()` doesn't work on Mac, Solaris, and maybe other
        // BSDs when used on the master port.
        let master_tty = SerialPort {
            fd: next_pty_fd.into_raw_fd(),
            read_timeout: None,
            write_timeout: None,
            exclusive: true,
            port_name: None,
            #[cfg(any(target_os = "ios", target_os = "macos"))]
            baud_rate,
        };

        Ok((master_tty, slave_tty))
    }

    /// Returns the exclusivity of the port
    ///
    /// If a port is exclusive, then trying to open the same device path again
    /// will fail.
    fn exclusive(&self) -> bool {
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
    fn set_exclusive(&mut self, exclusive: bool) -> Result<()> {
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

    /// Sends 0-valued bits over the port for a set duration
    fn send_break(&self, duration: BreakDuration) -> Result<()> {
        match duration {
            BreakDuration::Short => nix::sys::termios::tcsendbreak(self.fd, 0),
            BreakDuration::Arbitrary(n) => nix::sys::termios::tcsendbreak(self.fd, n.get()),
        }
        .map_err(|e| e.into())
    }
}

#[test]
fn test_ttyport_into_raw_fd() {
    // `master` must be used here as Dropping it causes slave to be deleted by the OS.
    // TODO: Convert this to a statement-level attribute once
    //       https://github.com/rust-lang/rust/issues/15701 is on stable.
    // FIXME: Create a mutex across all tests for using `SerialPort::pair()` as it's not threadsafe
    #![allow(unused_variables)]
    let (master, slave) = SerialPort::pair().expect("Unable to create ptty pair");

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
