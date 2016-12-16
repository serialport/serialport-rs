extern crate libc;
#[cfg(target_os = "linux")]
extern crate libudev;
extern crate termios;
extern crate ioctl_rs as ioctl;

use std::ffi::CString;
use std::io;
use std::path::Path;
use std::time::Duration;

use std::os::unix::prelude::*;

use self::libc::{c_int,c_void,size_t};

use ::{DataBits, FlowControl, Parity, SerialPort, SerialPortInfo, StopBits};


#[cfg(target_os = "linux")]
const O_NOCTTY: c_int = 0x00000100;

#[cfg(target_os = "macos")]
const O_NOCTTY: c_int = 0x00020000;

#[cfg(not(any(target_os = "linux", target_os = "macos")))]
const O_NOCTTY: c_int = 0;


/// A TTY-based serial port implementation.
///
/// The port will be closed when the value is dropped.
#[derive(Debug)]
pub struct TTYPort {
    fd: RawFd,
    termios: termios::Termios,
    timeout: Duration
}

impl TTYPort {
    /// Opens a TTY device as a serial port.
    ///
    /// `path` should be the path to a TTY device, e.g., `/dev/ttyS0`.
    ///
    /// ```no_run
    /// use std::path::Path;
    ///
    /// serial::posix::TTYPort::open(Path::new("/dev/ttyS0")).unwrap();
    /// ```
    ///
    /// ## Errors
    ///
    /// * `NoDevice` if the device could not be opened. This could indicate that the device is
    ///   already in use.
    /// * `InvalidInput` if `port` is not a valid device name.
    /// * `Io` for any other error while opening or initializing the device.
    pub fn open(path: &Path) -> ::Result<Box<SerialPort>> {
        use self::libc::{O_RDWR,O_NONBLOCK,F_SETFL,EINVAL};
        use self::termios::{CREAD,CLOCAL}; // cflags
        use self::termios::{ICANON,ECHO,ECHOE,ECHOK,ECHONL,ISIG,IEXTEN}; // lflags
        use self::termios::{OPOST}; // oflags
        use self::termios::{INLCR,IGNCR,ICRNL,IGNBRK}; // iflags
        use self::termios::{VMIN,VTIME}; // c_cc indexes
        use self::termios::{tcsetattr,tcflush};
        use self::termios::{TCSANOW,TCIOFLUSH};

        let cstr = match CString::new(path.as_os_str().as_bytes()) {
            Ok(s) => s,
            Err(_) => return Err(super::error::from_raw_os_error(EINVAL))
        };

        let fd = unsafe { libc::open(cstr.as_ptr(), O_RDWR | O_NOCTTY | O_NONBLOCK, 0) };
        if fd < 0 {
            return Err(super::error::last_os_error());
        }

        let mut termios = match termios::Termios::from_fd(fd) {
            Ok(t) => t,
            Err(e) => return Err(super::error::from_io_error(e))
        };

        // setup TTY for binary serial port access
        termios.c_cflag |= CREAD | CLOCAL;
        termios.c_lflag &= !(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN);
        termios.c_oflag &= !OPOST;
        termios.c_iflag &= !(INLCR | IGNCR | ICRNL | IGNBRK);

        termios.c_cc[VMIN] = 0;
        termios.c_cc[VTIME] = 0;

        // write settings to TTY
        if let Err(err) = tcsetattr(fd, TCSANOW, &termios) {
            return Err(super::error::from_io_error(err));
        }

        if let Err(err) = tcflush(fd, TCIOFLUSH) {
            return Err(super::error::from_io_error(err));
        }

        let port = TTYPort {
            fd: fd,
            termios: termios,
            timeout: Duration::from_millis(100)
        };

        // get exclusive access to device
        if let Err(err) = ioctl::tiocexcl(port.fd) {
            return Err(super::error::from_io_error(err))
        }

        // clear O_NONBLOCK flag
        if unsafe { libc::fcntl(port.fd, F_SETFL, 0) } < 0 {
            return Err(super::error::last_os_error());
        }

        Ok(Box::new(port))
    }

    fn set_pin(&mut self, pin: c_int, level: bool) -> ::Result<()> {
        let retval = if level {
            ioctl::tiocmbis(self.fd, pin)
        }
        else {
            ioctl::tiocmbic(self.fd, pin)
        };

        match retval {
            Ok(()) => Ok(()),
            Err(err) => Err(super::error::from_io_error(err))
        }
    }

    fn read_pin(&mut self, pin: c_int) -> ::Result<bool> {
        match ioctl::tiocmget(self.fd) {
            Ok(pins) => Ok(pins & pin != 0),
            Err(err) => Err(super::error::from_io_error(err))
        }
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

impl io::Read for TTYPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        try!(super::poll::wait_read_fd(self.fd, self.timeout));

        let len = unsafe { libc::read(self.fd, buf.as_ptr() as *mut c_void, buf.len() as size_t) };

        if len >= 0 {
            Ok(len as usize)
        }
        else {
            Err(io::Error::last_os_error())
        }
    }
}

impl io::Write for TTYPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        try!(super::poll::wait_write_fd(self.fd, self.timeout));

        let len = unsafe { libc::write(self.fd, buf.as_ptr() as *mut c_void, buf.len() as size_t) };

        if len >= 0 {
            Ok(len as usize)
        }
        else {
            Err(io::Error::last_os_error())
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        termios::tcdrain(self.fd)
    }
}

impl SerialPort for TTYPort {

    fn baud_rate(&self) -> Option<::BaudRate> {
        use self::termios::{cfgetospeed,cfgetispeed};
        use self::termios::{B50,B75,B110,B134,B150,B200,B300,B600,B1200,B1800,B2400,B4800,B9600,B19200,B38400};
        use self::termios::os::target::{B57600,B115200,B230400};

        #[cfg(target_os = "linux")]
        use self::termios::os::linux::{B460800,B500000,B576000,B921600,B1000000,B1152000,B1500000,B2000000,B2500000,B3000000,B3500000,B4000000};

        #[cfg(target_os = "macos")]
        use self::termios::os::macos::{B7200,B14400,B28800,B76800};

        #[cfg(target_os = "freebsd")]
        use self::termios::os::freebsd::{B7200,B14400,B28800,B76800,B460800,B921600};

        #[cfg(target_os = "openbsd")]
        use self::termios::os::openbsd::{B7200,B14400,B28800,B76800};

        let ospeed = cfgetospeed(&self.termios);
        let ispeed = cfgetispeed(&self.termios);

        if ospeed != ispeed {
            return None
        }

        match ospeed {
            B50      => Some(::BaudOther(50)),
            B75      => Some(::BaudOther(75)),
            B110     => Some(::Baud110),
            B134     => Some(::BaudOther(134)),
            B150     => Some(::BaudOther(150)),
            B200     => Some(::BaudOther(200)),
            B300     => Some(::Baud300),
            B600     => Some(::Baud600),
            B1200    => Some(::Baud1200),
            B1800    => Some(::BaudOther(1800)),
            B2400    => Some(::Baud2400),
            B4800    => Some(::Baud4800),
            #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
            B7200    => Some(::BaudOther(7200)),
            B9600    => Some(::Baud9600),
            #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
            B14400   => Some(::BaudOther(14400)),
            B19200   => Some(::Baud19200),
            #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
            B28800   => Some(::BaudOther(28800)),
            B38400   => Some(::Baud38400),
            B57600   => Some(::Baud57600),
            #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
            B76800   => Some(::BaudOther(76800)),
            B115200  => Some(::Baud115200),
            B230400  => Some(::BaudOther(230400)),
            #[cfg(any(target_os = "linux", target_os = "freebsd"))]
            B460800  => Some(::BaudOther(460800)),
            #[cfg(target_os = "linux")]
            B500000  => Some(::BaudOther(500000)),
            #[cfg(target_os = "linux")]
            B576000  => Some(::BaudOther(576000)),
            #[cfg(any(target_os = "linux", target_os = "freebsd"))]
            B921600  => Some(::BaudOther(921600)),
            #[cfg(target_os = "linux")]
            B1000000 => Some(::BaudOther(1000000)),
            #[cfg(target_os = "linux")]
            B1152000 => Some(::BaudOther(1152000)),
            #[cfg(target_os = "linux")]
            B1500000 => Some(::BaudOther(1500000)),
            #[cfg(target_os = "linux")]
            B2000000 => Some(::BaudOther(2000000)),
            #[cfg(target_os = "linux")]
            B2500000 => Some(::BaudOther(2500000)),
            #[cfg(target_os = "linux")]
            B3000000 => Some(::BaudOther(3000000)),
            #[cfg(target_os = "linux")]
            B3500000 => Some(::BaudOther(3500000)),
            #[cfg(target_os = "linux")]
            B4000000 => Some(::BaudOther(4000000)),

            _ => None
        }
    }

    fn data_bits(&self) -> Option<DataBits> {
        use self::termios::{CSIZE,CS5,CS6,CS7,CS8};

        match self.termios.c_cflag & CSIZE {
            CS8 => Some(DataBits::Eight),
            CS7 => Some(DataBits::Seven),
            CS6 => Some(DataBits::Six),
            CS5 => Some(DataBits::Five),

            _ => None
        }
    }

    fn flow_control(&self) -> Option<FlowControl> {
        use self::termios::{IXON,IXOFF};
        use self::termios::os::target::{CRTSCTS};

        if self.termios.c_cflag & CRTSCTS != 0 {
            Some(FlowControl::Hardware)
        }
        else if self.termios.c_iflag & (IXON | IXOFF) != 0 {
            Some(FlowControl::Software)
        }
        else {
            Some(FlowControl::None)
        }
    }

    fn parity(&self) -> Option<Parity> {
        use self::termios::{PARENB,PARODD};

        if self.termios.c_cflag & PARENB != 0 {
            if self.termios.c_cflag & PARODD != 0 {
                Some(Parity::Odd)
            }
            else {
                Some(Parity::Even)
            }
        }
        else {
            Some(Parity::None)
        }
    }

    fn stop_bits(&self) -> Option<StopBits> {
        use self::termios::{CSTOPB};

        if self.termios.c_cflag & CSTOPB != 0 {
            Some(StopBits::Two)
        }
        else {
            Some(StopBits::One)
        }
    }

    fn timeout(&self) -> Duration {
        self.timeout
    }

    fn set_baud_rate(&mut self, baud_rate: ::BaudRate) -> ::Result<()> {
        use self::libc::{EINVAL};
        use self::termios::cfsetspeed;
        use self::termios::{B50,B75,B110,B134,B150,B200,B300,B600,B1200,B1800,B2400,B4800,B9600,B19200,B38400};
        use self::termios::os::target::{B57600,B115200,B230400};

        #[cfg(target_os = "linux")]
        use self::termios::os::linux::{B460800,B500000,B576000,B921600,B1000000,B1152000,B1500000,B2000000,B2500000,B3000000,B3500000,B4000000};

        #[cfg(target_os = "macos")]
        use self::termios::os::macos::{B7200,B14400,B28800,B76800};

        #[cfg(target_os = "freebsd")]
        use self::termios::os::freebsd::{B7200,B14400,B28800,B76800,B460800,B921600};

        #[cfg(target_os = "openbsd")]
        use self::termios::os::openbsd::{B7200,B14400,B28800,B76800};

        let baud = match baud_rate {
            ::BaudOther(50)      => B50,
            ::BaudOther(75)      => B75,
            ::Baud110            => B110,
            ::BaudOther(134)     => B134,
            ::BaudOther(150)     => B150,
            ::BaudOther(200)     => B200,
            ::Baud300            => B300,
            ::Baud600            => B600,
            ::Baud1200           => B1200,
            ::BaudOther(1800)    => B1800,
            ::Baud2400           => B2400,
            ::Baud4800           => B4800,
            #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
            ::BaudOther(7200)    => B7200,
            ::Baud9600           => B9600,
            #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
            ::BaudOther(14400)   => B14400,
            ::Baud19200          => B19200,
            #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
            ::BaudOther(28800)   => B28800,
            ::Baud38400          => B38400,
            ::Baud57600          => B57600,
            #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
            ::BaudOther(76800)   => B76800,
            ::Baud115200         => B115200,
            ::BaudOther(230400)  => B230400,
            #[cfg(any(target_os = "linux", target_os = "freebsd"))]
            ::BaudOther(460800)  => B460800,
            #[cfg(target_os = "linux")]
            ::BaudOther(500000)  => B500000,
            #[cfg(target_os = "linux")]
            ::BaudOther(576000)  => B576000,
            #[cfg(any(target_os = "linux", target_os = "freebsd"))]
            ::BaudOther(921600)  => B921600,
            #[cfg(target_os = "linux")]
            ::BaudOther(1000000) => B1000000,
            #[cfg(target_os = "linux")]
            ::BaudOther(1152000) => B1152000,
            #[cfg(target_os = "linux")]
            ::BaudOther(1500000) => B1500000,
            #[cfg(target_os = "linux")]
            ::BaudOther(2000000) => B2000000,
            #[cfg(target_os = "linux")]
            ::BaudOther(2500000) => B2500000,
            #[cfg(target_os = "linux")]
            ::BaudOther(3000000) => B3000000,
            #[cfg(target_os = "linux")]
            ::BaudOther(3500000) => B3500000,
            #[cfg(target_os = "linux")]
            ::BaudOther(4000000) => B4000000,

            ::BaudOther(_) => return Err(super::error::from_raw_os_error(EINVAL))
        };

        match cfsetspeed(&mut self.termios, baud) {
            Ok(()) => Ok(()),
            Err(err) => Err(super::error::from_io_error(err))
        }
    }

    fn set_data_bits(&mut self, data_bits: DataBits) -> ::Result<()>  {
        use self::termios::{CSIZE,CS5,CS6,CS7,CS8};

        let size = match data_bits {
            DataBits::Five => CS5,
            DataBits::Six => CS6,
            DataBits::Seven => CS7,
            DataBits::Eight => CS8
        };

        self.termios.c_cflag &= !CSIZE;
        self.termios.c_cflag |= size;
        Ok(())
    }

    fn set_flow_control(&mut self, flow_control: FlowControl) -> ::Result<()>  {
        use self::termios::{IXON,IXOFF};
        use self::termios::os::target::{CRTSCTS};

        match flow_control {
            FlowControl::None => {
                self.termios.c_iflag &= !(IXON | IXOFF);
                self.termios.c_cflag &= !CRTSCTS;
            },
            FlowControl::Software => {
                self.termios.c_iflag |= IXON | IXOFF;
                self.termios.c_cflag &= !CRTSCTS;
            },
            FlowControl::Hardware => {
                self.termios.c_iflag &= !(IXON | IXOFF);
                self.termios.c_cflag |= CRTSCTS;
            }
        };
        Ok(())
    }

    fn set_parity(&mut self, parity: Parity) -> ::Result<()>  {
        use self::termios::{PARENB,PARODD,INPCK,IGNPAR};

        match parity {
            Parity::None => {
                self.termios.c_cflag &= !(PARENB | PARODD);
                self.termios.c_iflag &= !INPCK;
                self.termios.c_iflag |= IGNPAR;
            },
            Parity::Odd => {
                self.termios.c_cflag |= PARENB | PARODD;
                self.termios.c_iflag |= INPCK;
                self.termios.c_iflag &= !IGNPAR;
            },
            Parity::Even => {
                self.termios.c_cflag &= !PARODD;
                self.termios.c_cflag |= PARENB;
                self.termios.c_iflag |= INPCK;
                self.termios.c_iflag &= !IGNPAR;
            }
        };
        Ok(())
    }

    fn set_stop_bits(&mut self, stop_bits: StopBits) -> ::Result<()>  {
        use self::termios::{CSTOPB};

        match stop_bits {
            StopBits::Two => self.termios.c_cflag &= !CSTOPB,
            StopBits::One => self.termios.c_cflag |= CSTOPB
        };
        Ok(())
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

    fn read_carrier_data(&mut self) -> ::Result<bool> {
        self.read_pin(ioctl::TIOCM_CD)
    }
}

#[cfg(target_os = "linux")]
pub fn available_ports() -> ::Result<Vec<SerialPortInfo>> {
    let mut vec = Vec::new();
    if let Ok(context) = libudev::Context::new() {
        let mut enumerator = try!(libudev::Enumerator::new(&context));
        try!(enumerator.match_subsystem("tty"));
        let devices = try!(enumerator.scan_devices());
        for d in devices {
            if let Some(p) = d.parent() {
                if let Some(devnode) = d.devnode() {
                    if let Some(path) = devnode.to_str() {
                        if let Some(driver) = p.driver() {
                            if driver == "serial8250" && TTYPort::open(devnode).is_err() {
                                continue;
                            }
                        }
                        vec.push(SerialPortInfo { port_name: String::from(path) });
                    }
                }
            }
        }
    }
    Ok(vec)
}

pub fn available_baud_rates() -> Vec<u32> {
    let mut vec = vec![50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400, 4800];
    #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
    vec.push(7200);
    vec.push(9600);
    #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
    vec.push(14400);
    vec.push(19200);
    #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
    vec.push(28800);
    vec.push(38400);
    vec.push(57600);
    #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "openbsd"))]
    vec.push(76800);
    vec.push(115200);
    vec.push(230400);
    #[cfg(any(target_os = "linux", target_os = "freebsd"))]
    vec.push(460800);
    #[cfg(target_os = "linux")]
    vec.push(500000);
    #[cfg(target_os = "linux")]
    vec.push(576000);
    #[cfg(any(target_os = "linux", target_os = "freebsd"))]
    vec.push(921600);
    #[cfg(target_os = "linux")]
    vec.push(1000000);
    #[cfg(target_os = "linux")]
    vec.push(1152000);
    #[cfg(target_os = "linux")]
    vec.push(1500000);
    #[cfg(target_os = "linux")]
    vec.push(2000000);
    #[cfg(target_os = "linux")]
    vec.push(2500000);
    #[cfg(target_os = "linux")]
    vec.push(3000000);
    #[cfg(target_os = "linux")]
    vec.push(3500000);
    #[cfg(target_os = "linux")]
    vec.push(4000000);
    vec
}
