use std::ffi::{CString, CStr};
#[cfg(target_os = "linux")]
use std::ffi::OsStr;
use std::io;
use std::mem;
use std::os::unix::prelude::*;
use std::path::Path;
use std::time::Duration;

use ioctl;
use libc::{self, c_int, c_void, size_t};
#[cfg(target_os = "linux")]
use libudev;
use termios;

use {BaudRate, DataBits, FlowControl, Parity, SerialPort, SerialPortInfo, SerialPortSettings,
     StopBits};
use {Error, ErrorKind};


#[cfg(target_os = "linux")]
const O_NOCTTY: c_int = 0x00000100;

#[cfg(target_os = "macos")]
const O_NOCTTY: c_int = 0x00020000;

#[cfg(not(any(target_os = "linux", target_os = "macos")))]
const O_NOCTTY: c_int = 0;


/// A TTY-based serial port implementation.
///
/// The port will be closed when the value is dropped. However, this struct
/// should not be instantiated directly by using `TTYPort::open()`, instead use
/// the cross-platform `serialport::open()` or
/// `serialport::open_with_settings()`.
#[derive(Debug)]
pub struct TTYPort {
    fd: RawFd,
    termios: termios::Termios,
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
        use libc::{O_RDWR, O_NONBLOCK, F_SETFL, EINVAL};
        use termios::{CREAD, CLOCAL}; // cflags
        use termios::{cfmakeraw, tcgetattr, tcsetattr, tcflush};
        use termios::{TCSANOW, TCIOFLUSH};

        let cstr = match CString::new(path.as_os_str().as_bytes()) {
            Ok(s) => s,
            Err(_) => return Err(super::error::from_raw_os_error(EINVAL)),
        };

        let fd = unsafe { libc::open(cstr.as_ptr(), O_RDWR | O_NOCTTY | O_NONBLOCK, 0) };
        if fd < 0 {
            return Err(super::error::last_os_error());
        }

        let mut termios = match termios::Termios::from_fd(fd) {
            Ok(t) => t,
            Err(e) => return Err(super::error::from_io_error(e)),
        };

        // setup TTY for binary serial port access
        // Enable reading from the port and ignore all modem control lines
        termios.c_cflag |= CREAD | CLOCAL;
        // Enable raw mode with disables any implicit processing of the input or output data streams
        // This also sets no timeout period and a read will block until at least one character is
        // available.
        cfmakeraw(&mut termios);

        // write settings to TTY
        if let Err(err) = tcsetattr(fd, TCSANOW, &termios) {
            return Err(super::error::from_io_error(err));
        }

        // Read back settings from port and confirm they were applied correctly
        // TODO: Switch this to an all-zeroed termios struct
        let mut actual_termios = termios;
        if let Err(err) = tcgetattr(fd, &mut actual_termios) {
            return Err(super::error::from_io_error(err));
        }
        if actual_termios != termios {
            return Err(Error::new(ErrorKind::Unknown, "Settings did not apply correctly"));
        }

        if let Err(err) = tcflush(fd, TCIOFLUSH) {
            return Err(super::error::from_io_error(err));
        }

        let mut port = TTYPort {
            fd: fd,
            termios: termios,
            timeout: Duration::from_millis(100),
            exclusive: true, // This is guaranteed by the following `ioctl::tiocexcl()` call
            port_name: path.to_str().map(|s| s.to_string()),
        };

        // get exclusive access to device
        if let Err(err) = ioctl::tiocexcl(port.fd) {
            return Err(super::error::from_io_error(err));
        }

        // clear O_NONBLOCK flag
        if unsafe { libc::fcntl(port.fd, F_SETFL, 0) } < 0 {
            return Err(super::error::last_os_error());
        }

        port.set_all(settings)?;

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
            Err(super::error::from_io_error(err))
        } else {
            self.exclusive = exclusive;
            Ok(())
        }
    }

    fn write_settings(&self) -> ::Result<()> {
        use termios::{tcsetattr, tcflush};
        use termios::{TCSANOW, TCIOFLUSH};

        if let Err(err) = tcsetattr(self.fd, TCSANOW, &self.termios) {
            return Err(super::error::from_io_error(err));
        }

        if let Err(err) = tcflush(self.fd, TCIOFLUSH) {
            return Err(super::error::from_io_error(err));
        }
        Ok(())
    }

    fn set_pin(&mut self, pin: c_int, level: bool) -> ::Result<()> {
        let retval = if level {
            ioctl::tiocmbis(self.fd, pin)
        } else {
            ioctl::tiocmbic(self.fd, pin)
        };

        match retval {
            Ok(()) => Ok(()),
            Err(err) => Err(super::error::from_io_error(err)),
        }
    }

    fn read_pin(&mut self, pin: c_int) -> ::Result<bool> {
        match ioctl::tiocmget(self.fd) {
            Ok(pins) => Ok(pins & pin != 0),
            Err(err) => Err(super::error::from_io_error(err)),
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
    /// ## Examples
    ///
    /// ```
    /// use serialport::posix::TTYPort;
    ///
    /// let (master, slave) = TTYPort::pair().unwrap();
    /// ```
    pub fn pair() -> ::Result<(Self, Self)> {

        // Open the next free pty.
        let next_pty_fd = unsafe { libc::posix_openpt(libc::O_RDWR) };
        if next_pty_fd < 0 {
            return Err(super::error::last_os_error());
        }

        // Grant access to the associated slave pty
        if unsafe { libc::grantpt(next_pty_fd) } < 0 {
            unsafe { libc::close(next_pty_fd) };
            return Err(super::error::last_os_error());
        }

        // Unlock the slave pty
        if unsafe { libc::unlockpt(next_pty_fd) } < 0 {
            unsafe { libc::close(next_pty_fd) };
            return Err(super::error::last_os_error());
        }

        // Get the path of the attached slave ptty
        let ptty_name_ptr = unsafe { libc::ptsname(next_pty_fd) };
        if ptty_name_ptr.is_null() {
            unsafe { libc::close(next_pty_fd) };
            return Err(super::error::last_os_error());
        }

        let ptty_name_cstr = unsafe { CStr::from_ptr(ptty_name_ptr) };
        let ptty_name = match ptty_name_cstr.to_str() {
            Ok(s) => s,
            Err(_) => {
                unsafe { libc::close(next_pty_fd) };
                return Err(super::error::last_os_error());
            }
        };

        // Make TTYPort structs for both master and slave pty's
        let master_tty = unsafe { TTYPort::from_raw_fd(next_pty_fd) };
        let settings = master_tty.settings();
        let slave_tty = TTYPort::open(Path::new(ptty_name), &settings)?;

        Ok((master_tty, slave_tty))
    }
}

impl Drop for TTYPort {
    fn drop(&mut self) {
        #![allow(unused_must_use)]
        ioctl::tiocnxcl(self.fd);

        unsafe {
            libc::close(self.fd);
        }
    }
}

impl AsRawFd for TTYPort {
    fn as_raw_fd(&self) -> RawFd {
        self.fd
    }
}

impl IntoRawFd for TTYPort {
    fn into_raw_fd(self) -> RawFd {
        // Pull just the file descriptor out.  Let the
        // rest of the member fields drop.
        let TTYPort { fd, .. } = self;
        fd
    }
}

impl FromRawFd for TTYPort {
    unsafe fn from_raw_fd(fd: RawFd) -> Self {

        let termios = termios::Termios::from_fd(fd).expect("Unable to retrieve termios settings.");

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
            termios: termios,
            timeout: Duration::from_millis(100),
            exclusive: exclusive,
            port_name: None,
        }
    }
}

impl io::Read for TTYPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        super::poll::wait_read_fd(self.fd, self.timeout)?;

        let len = unsafe { libc::read(self.fd, buf.as_ptr() as *mut c_void, buf.len() as size_t) };

        if len >= 0 {
            Ok(len as usize)
        } else {
            Err(io::Error::last_os_error())
        }
    }
}

impl io::Write for TTYPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        super::poll::wait_write_fd(self.fd, self.timeout)?;

        let len = unsafe { libc::write(self.fd, buf.as_ptr() as *mut c_void, buf.len() as size_t) };

        if len >= 0 {
            Ok(len as usize)
        } else {
            Err(io::Error::last_os_error())
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        termios::tcdrain(self.fd)
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
        use termios::{cfgetospeed, cfgetispeed};
        use termios::{B50, B75, B110, B134, B150, B200, B300, B600, B1200, B1800, B2400, B4800,
                      B9600, B19200, B38400};
        use termios::os::target::{B57600, B115200, B230400};

        #[cfg(target_os = "linux")]
        use termios::os::linux::{B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000,
                                 B2000000, B2500000, B3000000, B3500000, B4000000};

        #[cfg(target_os = "macos")]
        use termios::os::macos::{B7200, B14400, B28800, B76800};

        #[cfg(target_os = "freebsd")]
        use termios::os::freebsd::{B7200, B14400, B28800, B76800, B460800, B921600};

        #[cfg(target_os = "openbsd")]
        use termios::os::openbsd::{B7200, B14400, B28800, B76800};

        let ospeed = cfgetospeed(&self.termios);
        let ispeed = cfgetispeed(&self.termios);

        if ospeed != ispeed {
            return None;
        }

        match ospeed {
            B50 => Some(BaudRate::BaudOther(50)),
            B75 => Some(BaudRate::BaudOther(75)),
            B110 => Some(BaudRate::Baud110),
            B134 => Some(BaudRate::BaudOther(134)),
            B150 => Some(BaudRate::BaudOther(150)),
            B200 => Some(BaudRate::BaudOther(200)),
            B300 => Some(BaudRate::Baud300),
            B600 => Some(BaudRate::Baud600),
            B1200 => Some(BaudRate::Baud1200),
            B1800 => Some(BaudRate::BaudOther(1800)),
            B2400 => Some(BaudRate::Baud2400),
            B4800 => Some(BaudRate::Baud4800),
            #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
            B7200 => Some(BaudRate::BaudOther(7200)),
            B9600 => Some(BaudRate::Baud9600),
            #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
            B14400 => Some(BaudRate::BaudOther(14400)),
            B19200 => Some(BaudRate::Baud19200),
            #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
            B28800 => Some(BaudRate::BaudOther(28800)),
            B38400 => Some(BaudRate::Baud38400),
            B57600 => Some(BaudRate::Baud57600),
            #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
            B76800 => Some(BaudRate::BaudOther(76800)),
            B115200 => Some(BaudRate::Baud115200),
            B230400 => Some(BaudRate::BaudOther(230400)),
            #[cfg(any(target_os = "linux", target_os = "freebsd"))]
            B460800 => Some(BaudRate::BaudOther(460800)),
            #[cfg(target_os = "linux")]
            B500000 => Some(BaudRate::BaudOther(500000)),
            #[cfg(target_os = "linux")]
            B576000 => Some(BaudRate::BaudOther(576000)),
            #[cfg(any(target_os = "linux", target_os = "freebsd"))]
            B921600 => Some(BaudRate::BaudOther(921600)),
            #[cfg(target_os = "linux")]
            B1000000 => Some(BaudRate::BaudOther(1000000)),
            #[cfg(target_os = "linux")]
            B1152000 => Some(BaudRate::BaudOther(1152000)),
            #[cfg(target_os = "linux")]
            B1500000 => Some(BaudRate::BaudOther(1500000)),
            #[cfg(target_os = "linux")]
            B2000000 => Some(BaudRate::BaudOther(2000000)),
            #[cfg(target_os = "linux")]
            B2500000 => Some(BaudRate::BaudOther(2500000)),
            #[cfg(target_os = "linux")]
            B3000000 => Some(BaudRate::BaudOther(3000000)),
            #[cfg(target_os = "linux")]
            B3500000 => Some(BaudRate::BaudOther(3500000)),
            #[cfg(target_os = "linux")]
            B4000000 => Some(BaudRate::BaudOther(4000000)),

            _ => None,
        }
    }

    fn data_bits(&self) -> Option<DataBits> {
        use termios::{CSIZE, CS5, CS6, CS7, CS8};

        match self.termios.c_cflag & CSIZE {
            CS8 => Some(DataBits::Eight),
            CS7 => Some(DataBits::Seven),
            CS6 => Some(DataBits::Six),
            CS5 => Some(DataBits::Five),

            _ => None,
        }
    }

    fn flow_control(&self) -> Option<FlowControl> {
        use termios::{IXON, IXOFF};
        use termios::os::target::CRTSCTS;

        if self.termios.c_cflag & CRTSCTS != 0 {
            Some(FlowControl::Hardware)
        } else if self.termios.c_iflag & (IXON | IXOFF) != 0 {
            Some(FlowControl::Software)
        } else {
            Some(FlowControl::None)
        }
    }

    fn parity(&self) -> Option<Parity> {
        use termios::{PARENB, PARODD};

        if self.termios.c_cflag & PARENB != 0 {
            if self.termios.c_cflag & PARODD != 0 {
                Some(Parity::Odd)
            } else {
                Some(Parity::Even)
            }
        } else {
            Some(Parity::None)
        }
    }

    fn stop_bits(&self) -> Option<StopBits> {
        use termios::CSTOPB;

        if self.termios.c_cflag & CSTOPB != 0 {
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
        use libc::EINVAL;
        use termios::cfsetspeed;
        use termios::{B50, B75, B110, B134, B150, B200, B300, B600, B1200, B1800, B2400, B4800,
                      B9600, B19200, B38400};
        use termios::os::target::{B57600, B115200, B230400};

        #[cfg(target_os = "linux")]
        use termios::os::linux::{B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000,
                                 B2000000, B2500000, B3000000, B3500000, B4000000};

        #[cfg(target_os = "macos")]
        use termios::os::macos::{B7200, B14400, B28800, B76800};

        #[cfg(target_os = "freebsd")]
        use termios::os::freebsd::{B7200, B14400, B28800, B76800, B460800, B921600};

        #[cfg(target_os = "openbsd")]
        use termios::os::openbsd::{B7200, B14400, B28800, B76800};

        let baud = match baud_rate {
            BaudRate::BaudOther(50) => B50,
            BaudRate::BaudOther(75) => B75,
            BaudRate::Baud110 => B110,
            BaudRate::BaudOther(134) => B134,
            BaudRate::BaudOther(150) => B150,
            BaudRate::BaudOther(200) => B200,
            BaudRate::Baud300 => B300,
            BaudRate::Baud600 => B600,
            BaudRate::Baud1200 => B1200,
            BaudRate::BaudOther(1800) => B1800,
            BaudRate::Baud2400 => B2400,
            BaudRate::Baud4800 => B4800,
            #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
            BaudRate::BaudOther(7200) => B7200,
            BaudRate::Baud9600 => B9600,
            #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
            BaudRate::BaudOther(14400) => B14400,
            BaudRate::Baud19200 => B19200,
            #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
            BaudRate::BaudOther(28800) => B28800,
            BaudRate::Baud38400 => B38400,
            BaudRate::Baud57600 => B57600,
            #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
            BaudRate::BaudOther(76800) => B76800,
            BaudRate::Baud115200 => B115200,
            BaudRate::BaudOther(230400) => B230400,
            #[cfg(any(target_os = "linux", target_os = "freebsd"))]
            BaudRate::BaudOther(460800) => B460800,
            #[cfg(target_os = "linux")]
            BaudRate::BaudOther(500000) => B500000,
            #[cfg(target_os = "linux")]
            BaudRate::BaudOther(576000) => B576000,
            #[cfg(any(target_os = "linux", target_os = "freebsd"))]
            BaudRate::BaudOther(921600) => B921600,
            #[cfg(target_os = "linux")]
            BaudRate::BaudOther(1000000) => B1000000,
            #[cfg(target_os = "linux")]
            BaudRate::BaudOther(1152000) => B1152000,
            #[cfg(target_os = "linux")]
            BaudRate::BaudOther(1500000) => B1500000,
            #[cfg(target_os = "linux")]
            BaudRate::BaudOther(2000000) => B2000000,
            #[cfg(target_os = "linux")]
            BaudRate::BaudOther(2500000) => B2500000,
            #[cfg(target_os = "linux")]
            BaudRate::BaudOther(3000000) => B3000000,
            #[cfg(target_os = "linux")]
            BaudRate::BaudOther(3500000) => B3500000,
            #[cfg(target_os = "linux")]
            BaudRate::BaudOther(4000000) => B4000000,

            BaudRate::BaudOther(_) => return Err(super::error::from_raw_os_error(EINVAL)),
        };

        cfsetspeed(&mut self.termios, baud)?;
        self.write_settings()
    }

    fn set_data_bits(&mut self, data_bits: DataBits) -> ::Result<()> {
        use termios::{CSIZE, CS5, CS6, CS7, CS8};

        let size = match data_bits {
            DataBits::Five => CS5,
            DataBits::Six => CS6,
            DataBits::Seven => CS7,
            DataBits::Eight => CS8,
        };

        self.termios.c_cflag &= !CSIZE;
        self.termios.c_cflag |= size;
        self.write_settings()
    }

    fn set_flow_control(&mut self, flow_control: FlowControl) -> ::Result<()> {
        use termios::{IXON, IXOFF};
        use termios::os::target::CRTSCTS;

        match flow_control {
            FlowControl::None => {
                self.termios.c_iflag &= !(IXON | IXOFF);
                self.termios.c_cflag &= !CRTSCTS;
            }
            FlowControl::Software => {
                self.termios.c_iflag |= IXON | IXOFF;
                self.termios.c_cflag &= !CRTSCTS;
            }
            FlowControl::Hardware => {
                self.termios.c_iflag &= !(IXON | IXOFF);
                self.termios.c_cflag |= CRTSCTS;
            }
        };
        self.write_settings()
    }

    fn set_parity(&mut self, parity: Parity) -> ::Result<()> {
        use termios::{PARENB, PARODD, INPCK, IGNPAR};

        match parity {
            Parity::None => {
                self.termios.c_cflag &= !(PARENB | PARODD);
                self.termios.c_iflag &= !INPCK;
                self.termios.c_iflag |= IGNPAR;
            }
            Parity::Odd => {
                self.termios.c_cflag |= PARENB | PARODD;
                self.termios.c_iflag |= INPCK;
                self.termios.c_iflag &= !IGNPAR;
            }
            Parity::Even => {
                self.termios.c_cflag &= !PARODD;
                self.termios.c_cflag |= PARENB;
                self.termios.c_iflag |= INPCK;
                self.termios.c_iflag &= !IGNPAR;
            }
        };
        self.write_settings()
    }

    fn set_stop_bits(&mut self, stop_bits: StopBits) -> ::Result<()> {
        use termios::CSTOPB;

        match stop_bits {
            StopBits::Two => self.termios.c_cflag &= !CSTOPB,
            StopBits::One => self.termios.c_cflag |= CSTOPB,
        };
        self.write_settings()
    }

    fn set_timeout(&mut self, timeout: Duration) -> ::Result<()> {
        self.timeout = timeout;
        Ok(())
    }

    fn write_request_to_send(&mut self, level: bool) -> ::Result<()> {
        self.set_pin(ioctl::TIOCM_RTS, level)
    }

    fn write_data_terminal_ready(&mut self, level: bool) -> ::Result<()> {
        self.set_pin(ioctl::TIOCM_DTR, level)
    }

    fn read_clear_to_send(&mut self) -> ::Result<bool> {
        self.read_pin(ioctl::TIOCM_CTS)
    }

    fn read_data_set_ready(&mut self) -> ::Result<bool> {
        self.read_pin(ioctl::TIOCM_DSR)
    }

    fn read_ring_indicator(&mut self) -> ::Result<bool> {
        self.read_pin(ioctl::TIOCM_RI)
    }

    fn read_carrier_detect(&mut self) -> ::Result<bool> {
        self.read_pin(ioctl::TIOCM_CD)
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
fn port_type(d: &libudev::Device) -> ::Result<::SerialPortType> {
    match d.property_value("ID_BUS").and_then(OsStr::to_str) {
        Some("usb") => {
            Ok(::SerialPortType::UsbPort(::UsbPortInfo {
                vid: udev_hex_property_as_u16(d, "ID_VENDOR_ID")?,
                pid: udev_hex_property_as_u16(d, "ID_MODEL_ID")?,
                serial_number: udev_property_as_string(d, "ID_SERIAL_SHORT"),
                manufacturer: udev_property_as_string(d, "ID_VENDOR"),
                product: udev_property_as_string(d, "ID_MODEL"),
            }))
        }
        Some("pci") => Ok(::SerialPortType::PciPort),
        _ => Ok(::SerialPortType::Unknown),
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
                               TTYPort::open(devnode, &Default::default()).is_err() {
                                continue;
                            }
                        }
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
/// Scans the system for serial ports and returns a list of them.
/// The `SerialPortInfo` struct contains the name of the port which can be used for opening it.
pub fn available_ports() -> ::Result<Vec<SerialPortInfo>> {
    use IOKit_sys::*;
    use cf::*;
    use mach::port::{mach_port_t, MACH_PORT_NULL};
    use mach::kern_return::KERN_SUCCESS;

    let mut vec = Vec::new();
    unsafe {
        let mut master_port: mach_port_t = MACH_PORT_NULL;

        let classes_to_match = IOServiceMatching(kIOSerialBSDServiceValue());
        if classes_to_match.is_null() {
            panic!("IOServiceMatching returned a NULL dictionary.");
        }

        // build key
        let key = CFStringCreateWithCString(kCFAllocatorDefault,
                                            kIOSerialBSDTypeKey(),
                                            kCFStringEncodingUTF8);
        if key.is_null() {
            panic!("failed to allocate key string");
        }

        // build value
        let val = CFStringCreateWithCString(kCFAllocatorDefault,
                                            kIOSerialBSDRS232Type(),
                                            kCFStringEncodingUTF8);
        if val.is_null() {
            panic!("failed to allocate value string");
        }

        // set value in dictionary
        CFDictionarySetValue(classes_to_match, key as CFTypeRef, val as CFTypeRef);

        let mut kern_result = IOMasterPort(MACH_PORT_NULL, &mut master_port);
        if kern_result != KERN_SUCCESS {
            panic!("ERROR: {}", kern_result);
        }

        let mut matching_services: io_iterator_t = mem::uninitialized();

        kern_result = IOServiceGetMatchingServices(kIOMasterPortDefault,
                                                   classes_to_match,
                                                   &mut matching_services);
        if kern_result != KERN_SUCCESS {
            panic!("ERROR: {}", kern_result);
        }

        loop {
            let modem_service = IOIteratorNext(matching_services);

            if modem_service == MACH_PORT_NULL {
                break;
            }

            let mut props = mem::uninitialized();

            let result = IORegistryEntryCreateCFProperties(modem_service,
                                                           &mut props,
                                                           kCFAllocatorDefault,
                                                           0);
            if result == KERN_SUCCESS {
                let key = CString::new("IODialinDevice").unwrap();
                let key_cfstring = CFStringCreateWithCString(kCFAllocatorDefault,
                                                             key.as_ptr(),
                                                             kCFStringEncodingUTF8);
                let value = CFDictionaryGetValue(props, key_cfstring as *const c_void);

                let type_id = CFGetTypeID(value);
                if type_id == CFStringGetTypeID() {
                    let mut buf = Vec::<libc::c_char>::with_capacity(256);

                    CFStringGetCString(value as CFStringRef,
                                       buf.as_mut_ptr(),
                                       256,
                                       kCFStringEncodingUTF8);
                    let path = CStr::from_ptr(buf.as_ptr()).to_string_lossy();
                    vec.push(SerialPortInfo {
                        port_name: path.to_string(),
                        port_type: ::SerialPortType::Unknown,
                    });
                }
            }

            IOObjectRelease(modem_service);
        }
    }
    Ok(vec)
}

#[cfg(not(any(target_os = "linux", target_os = "macos")))]
/// Enumerating serial ports on non-Linux POSIX platforms is not yet supported
pub fn available_ports() -> ::Result<Vec<SerialPortInfo>> {
    Err(Error::new(ErrorKind::Unknown, "Not implemented for this OS"))
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
    vec.push(14400);
    vec.push(19200);
    #[cfg(any(target_os = "freebsd", target_os = "dragonfly", target_os = "macos",
              target_os = "netbsd", target_os = "openbsd"))]
    vec.push(28800);
    vec.push(38400);
    vec.push(57600);
    #[cfg(any(target_os = "freebsd", target_os = "dragonfly", target_os = "macos",
              target_os = "netbsd", target_os = "openbsd"))]
    vec.push(76800);
    vec.push(115200);
    vec.push(230400);
    #[cfg(any(target_os = "android", target_os = "freebsd", target_os = "linux"))]
    vec.push(460800);
    #[cfg(any(target_os = "android", target_os = "linux"))]
    vec.push(500000);
    #[cfg(any(target_os = "android", target_os = "linux"))]
    vec.push(576000);
    #[cfg(any(target_os = "android", target_os = "linux", target_os = "netbsd"))]
    vec.push(921600);
    #[cfg(any(target_os = "android", target_os = "linux"))]
    vec.push(1000000);
    #[cfg(any(target_os = "android", target_os = "linux"))]
    vec.push(1152000);
    #[cfg(any(target_os = "android", target_os = "linux"))]
    vec.push(1500000);
    #[cfg(any(target_os = "android", target_os = "linux"))]
    vec.push(2000000);
    #[cfg(any(target_os = "android", target_os = "linux"))]
    vec.push(2500000);
    #[cfg(any(target_os = "android", target_os = "linux"))]
    vec.push(3000000);
    #[cfg(any(target_os = "android", target_os = "linux"))]
    vec.push(3500000);
    #[cfg(any(target_os = "android", target_os = "linux"))]
    vec.push(4000000);
    vec
}
