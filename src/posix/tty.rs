use std::os::unix::prelude::*;
use std::path::Path;
use std::time::{Duration, Instant};
use std::{io, mem};

use nix::fcntl::{fcntl, OFlag};
use nix::{self, libc, unistd};

use crate::posix::ioctl::{self, SerialLines};
use crate::posix::termios;
use crate::{
    ClearBuffer, DataBits, Error, ErrorKind, FlowControl, Parity, ReadMode, Result, SerialPort,
    SerialPortBuilder, StopBits, WriteMode,
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
/// should not be instantiated directly by using `TTYPort::open()`, instead use
/// the cross-platform `serialport::open()` or
/// `serialport::open_with_settings()`.
#[derive(Debug)]
pub struct TTYPort {
    fd: RawFd,
    read_mode: ReadMode,
    write_mode: WriteMode,
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
    /// `path` should be the path to a TTY device, e.g. `/dev/ttyS0` or `/dev/ttyUSB0`
    ///
    /// ## Errors
    ///
    /// * `NoDevice` if the device could not be opened. This could indicate that
    ///    the device is already in use.
    /// * `InvalidInput` if `path` is not a valid device name.
    /// * `Io` for any other error while opening or initializing the device.
    pub fn open(builder: &SerialPortBuilder) -> Result<TTYPort> {
        let path = Path::new(&builder.path);
        let fd = nix::fcntl::open(
            path,
            OFlag::O_RDWR | OFlag::O_NOCTTY | OFlag::O_NONBLOCK,
            nix::sys::stat::Mode::empty(),
        )?;

        // Configure the low-level port settings
        // If any of these steps fail, then we should abort creation of the
        // TTYPort and ensure the file descriptor is closed.
        // So we wrap these calls in a block and check the result.
        // {
        //     let mut termios = termios::get_termios(fd)?;
            //termios::init_termios(&mut termios); // Seems to still have newline processing enabled
        //     termios::set_parity(&mut termios, builder.parity);
        //     termios::set_flow_control(&mut termios, builder.flow_control);
        //     termios::set_data_bits(&mut termios, builder.data_bits);
        //     termios::set_stop_bits(&mut termios, builder.stop_bits);
        //     #[cfg(not(any(target_os = "ios", target_os = "macos")))]
        //     termios::set_baud_rate(&mut termios, builder.baud_rate);
        //     #[cfg(any(target_os = "ios", target_os = "macos"))]
        //     termios::set_termios(fd, &termios, builder.baud_rate)?;
        //     #[cfg(not(any(target_os = "ios", target_os = "macos")))]
        //     termios::set_termios(fd, &termios)?;

            // Clear both input and output buffers in case there is any left over detritus, for
            // example if a port was disconnected.
        //     unsafe { nix::libc::tcflush(fd, libc::TCIOFLUSH) };

        //     Ok(())
        // }
        // .map_err(|e: Error| {
        //     close(fd);
        //     e
        // })?;


    let mut termios = std::mem::MaybeUninit::uninit();
    let res = unsafe { libc::tcgetattr(fd, termios.as_mut_ptr()) };
    nix::errno::Errno::result(res)?;
    let mut termios = unsafe { termios.assume_init() };

    // termios.c_cflag |= libc::CLOCAL | libc::CREAD;
    unsafe { nix::libc::cfmakeraw(&mut termios) };
    termios.c_cc[libc::VMIN] = 1;
    termios.c_cc[libc::VTIME] = 0;
    unsafe { nix::libc::tcsetattr(fd, libc::TCSANOW, &termios) };

        // Return the final port object
        Ok(TTYPort {
            fd,
            read_mode: builder.read_mode,
            write_mode: builder.write_mode,
            exclusive: false,
            port_name: Some(builder.path.clone()),
            #[cfg(any(target_os = "ios", target_os = "macos"))]
            baud_rate: builder.baud_rate,
        })
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
        if level {
            ioctl::tiocmbis(self.fd, pin)
        } else {
            ioctl::tiocmbic(self.fd, pin)
        }
    }

    fn read_pin(&mut self, pin: ioctl::SerialLines) -> Result<bool> {
        ioctl::tiocmget(self.fd).map(|pins| pins.contains(pin))
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

        let baud_rate = 9600; // Baud rate doesn't matter here, so set it to something well supported
        let slave_tty = crate::new(ptty_name, baud_rate).open_native().unwrap();

        // Manually construct the master port here because the
        // `tcgetattr()` doesn't work on Mac, Solaris, and maybe other
        // BSDs when used on the master port.
        let master_tty = TTYPort {
            fd: next_pty_fd.into_raw_fd(),
            read_mode: ReadMode::Immediate,
            write_mode: WriteMode::Immediate,
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

    /// Attempts to clone the `SerialPort`. This allow you to write and read simultaneously from the
    /// same serial connection. Please note that if you want a real asynchronous serial port you
    /// should look at [mio-serial](https://crates.io/crates/mio-serial) or
    /// [tokio-serial](https://crates.io/crates/tokio-serial).
    ///
    /// Also, you must be very careful when changing the settings of a cloned `SerialPort` : since
    /// the settings are cached on a per object basis, trying to modify them from two different
    /// objects can cause some nasty behavior.
    ///
    /// This is the same as `SerialPort::try_clone()` but returns the concrete type instead.
    ///
    /// # Errors
    ///
    /// This function returns an error if the serial port couldn't be cloned.
    pub fn try_clone_native(&self) -> Result<TTYPort> {
        let fd_cloned: i32 = fcntl(self.fd, nix::fcntl::F_DUPFD(self.fd))?;
        Ok(TTYPort {
            fd: fd_cloned,
            exclusive: self.exclusive,
            port_name: self.port_name.clone(),
            read_mode: self.read_mode,
            write_mode: self.write_mode,
            #[cfg(any(target_os = "ios", target_os = "macos"))]
            baud_rate: self.baud_rate,
        })
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
        TTYPort {
            fd,
            read_mode: ReadMode::Immediate,
            write_mode: WriteMode::Immediate,
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

impl io::Read for TTYPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let mut chars_left_to_read = buf.len();
        let timeout = match self.read_mode().unwrap() { // Can't fail on POSIX
            ReadMode::Timeout(t) => Duration::from_millis(t.get() as u64),
            ReadMode::Immediate => Duration::default(), // FIXME: Replace with `Duration::ZERO` once it stabilizes
            ReadMode::Blocking => Duration::new(u64::MAX, 1_000_000_000 - 1), // FIXME: Replace with `Duration::MAX` once it stabilizes
        };
        let start_instant = Instant::now();

        //println!("Starting loop!");
        while chars_left_to_read > 0 {
            //println!("chars_left_to_read: {}, start_instant.elapsed: {}", chars_left_to_read, start_instant.elapsed().as_millis());
            use nix::errno::Errno::{EIO, EPIPE};
            use nix::poll::{PollFd, PollFlags};

            // For poll's timeouts:
            //   * 0 => return immediately, even if fd isn't ready,
            //   * negative timeout => infinite timeout,
            //   * X => timeout is >= X ms
            let poll_timeout = match self.read_mode().unwrap() { // Can't fail on POSIX
                ReadMode::Blocking => -1 as nix::libc::c_int,
                _ => timeout.as_millis() as nix::libc::c_int,
            };
            let poll_events = PollFlags::POLLIN;
            let mut fd = PollFd::new(self.fd, poll_events);
            //println!("poll()");
            let wait = nix::poll::poll(std::slice::from_mut(&mut fd), poll_timeout)
                .map_err(|e| io::Error::from(crate::Error::from(e)))?;

            // All errors generated by poll are already caught by the nix wrapper around libc, so
            // we only need to check if there's at least 1 event. If there isn't, it means `poll`
            // timed out without any new input, so we can just return.
            if wait != 1 {
                //println!("Timing out!");
                break;
            }

            // Check the result of poll() by looking at the revents field
            match fd.revents() {
                Some(e) if e == poll_events => (),
                // If there was a hangout or invalid request
                Some(e) if e.contains(PollFlags::POLLHUP) || e.contains(PollFlags::POLLNVAL) => {
                    return Err(io::Error::new(io::ErrorKind::BrokenPipe, EPIPE.desc()))
                }
                Some(_) | None => return Err(io::Error::new(io::ErrorKind::Other, EIO.desc())),
            };

            let bytes_read = buf.len() - chars_left_to_read;
            //println!("read(&mut buf[{}...])", &bytes_read);
            let current_read = nix::unistd::read(self.fd, &mut buf[bytes_read..])
                .map_err(|e| io::Error::from(Error::from(e)))?;
            //println!("Read {} chars", &current_read);
            chars_left_to_read -= current_read;

            if start_instant.elapsed() >= timeout {
                break;
            }
        }
        // println!("Made it! ({} chars left)", buf.len() - chars_left_to_read);

        Ok(buf.len() - chars_left_to_read)
    }
}

impl io::Write for TTYPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let mut chars_left_to_write = buf.len();
        let timeout = match self.write_mode().unwrap() { // Can't fail on POSIX
            WriteMode::Timeout(t) => Duration::from_millis(t.get() as u64),
            WriteMode::Immediate => Duration::default(), // FIXME: Replace with `Duration::ZERO` once it stabilizes
            WriteMode::Blocking => Duration::new(u64::MAX, 1_000_000_000 - 1), // FIXME: Replace with `Duration::MAX` once it stabilizes
        };
        let start_instant = Instant::now();

        println!("Starting loop!");
        let elapsed = start_instant.elapsed();
        println!("chars_left_to_write: {}, start_instant.elapsed: {}, timeout: {}", chars_left_to_write, elapsed.as_nanos(), timeout.as_nanos());
        while chars_left_to_write > 0 {
            use nix::errno::Errno::{EIO, EPIPE};
            use nix::poll::{PollFd, PollFlags};
            println!("!!");
            // For poll's timeouts:
            //   * 0 => return immediately, even if fd isn't ready,
            //   * negative timeout => infinite timeout,
            //   * X => timeout is >= X ms
            let poll_timeout = match self.write_mode().unwrap() { // Can't fail on POSIX
                WriteMode::Blocking => -1 as nix::libc::c_int,
                _ => timeout.as_millis() as nix::libc::c_int,
            };
            let poll_events = PollFlags::POLLOUT;
            let mut fd = PollFd::new(self.fd, poll_events);
            println!("poll()");
            let wait = nix::poll::poll(std::slice::from_mut(&mut fd), poll_timeout)
                .map_err(|e| io::Error::from(crate::Error::from(e)))?;

            // All errors generated by poll are already caught by the nix wrapper around libc, so
            // we only need to check if there's at least 1 event. If there isn't, it means `poll`
            // timed out without any new input, so we can just return.
            if wait != 1 {
                println!("Timing out!");
                break;
            }

            // Check the result of poll() by looking at the revents field
            match fd.revents() {
                Some(e) if e == poll_events => (),
                // If there was a hangout or invalid request
                Some(e) if e.contains(PollFlags::POLLHUP) || e.contains(PollFlags::POLLNVAL) => {
                    return Err(io::Error::new(io::ErrorKind::BrokenPipe, EPIPE.desc()))
                }
                Some(_) | None => return Err(io::Error::new(io::ErrorKind::Other, EIO.desc())),
            };

            let bytes_written = buf.len() - chars_left_to_write;
            println!("write(&mut buf[{}...])", &bytes_written);
            let current_write = nix::unistd::write(self.fd, &buf[bytes_written..])
                .map_err(|e| io::Error::from(Error::from(e)))?;
            println!("Wrote {} chars", &current_write);
            chars_left_to_write -= current_write;

            if start_instant.elapsed() >= timeout {
                break;
            }
        }
        println!("Made it! ({} chars written)", buf.len() - chars_left_to_write);

        Ok(buf.len() - chars_left_to_write)
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

    fn data_bits(&self) -> Result<DataBits> {
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

    fn flow_control(&self) -> Result<FlowControl> {
        let termios = termios::get_termios(self.fd)?;
        if termios.c_cflag & libc::CRTSCTS == libc::CRTSCTS {
            Ok(FlowControl::Hardware)
        } else if termios.c_iflag & (libc::IXON | libc::IXOFF) == (libc::IXON | libc::IXOFF) {
            Ok(FlowControl::Software)
        } else {
            Ok(FlowControl::None)
        }
    }

    fn parity(&self) -> Result<Parity> {
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

    fn stop_bits(&self) -> Result<StopBits> {
        let termios = termios::get_termios(self.fd)?;
        if termios.c_cflag & libc::CSTOPB == libc::CSTOPB {
            Ok(StopBits::Two)
        } else {
            Ok(StopBits::One)
        }
    }

    fn read_mode(&self) -> Result<ReadMode> {
        Ok(self.read_mode)
    }

    fn write_mode(&self) -> Result<WriteMode> {
        Ok(self.write_mode)
    }

    #[cfg(any(
        target_os = "android",
        target_os = "dragonflybsd",
        target_os = "freebsd",
        target_os = "netbsd",
        target_os = "openbsd",
        target_os = "linux"
    ))]
    fn set_baud_rate(&mut self, baud_rate: u32) -> Result<()> {
        let mut termios = termios::get_termios(self.fd)?;
        termios::set_baud_rate(&mut termios, baud_rate);
        termios::set_termios(self.fd, &termios)
    }

    // Mac OS needs special logic for setting arbitrary baud rates.
    #[cfg(any(target_os = "ios", target_os = "macos"))]
    fn set_baud_rate(&mut self, baud_rate: u32) -> Result<()> {
        ioctl::iossiospeed(self.fd, &(baud_rate as libc::speed_t))?;
        self.baud_rate = baud_rate;
        Ok(())
    }

    fn set_flow_control(&mut self, flow_control: FlowControl) -> Result<()> {
        let mut termios = termios::get_termios(self.fd)?;
        termios::set_flow_control(&mut termios, flow_control);
        #[cfg(any(target_os = "ios", target_os = "macos"))]
        return termios::set_termios(self.fd, &termios, self.baud_rate);
        #[cfg(not(any(target_os = "ios", target_os = "macos")))]
        return termios::set_termios(self.fd, &termios);
    }

    fn set_parity(&mut self, parity: Parity) -> Result<()> {
        let mut termios = termios::get_termios(self.fd)?;
        termios::set_parity(&mut termios, parity);
        #[cfg(any(target_os = "ios", target_os = "macos"))]
        return termios::set_termios(self.fd, &termios, self.baud_rate);
        #[cfg(not(any(target_os = "ios", target_os = "macos")))]
        return termios::set_termios(self.fd, &termios);
    }

    fn set_data_bits(&mut self, data_bits: DataBits) -> Result<()> {
        let mut termios = termios::get_termios(self.fd)?;
        termios::set_data_bits(&mut termios, data_bits);
        #[cfg(any(target_os = "ios", target_os = "macos"))]
        return termios::set_termios(self.fd, &termios, self.baud_rate);
        #[cfg(not(any(target_os = "ios", target_os = "macos")))]
        return termios::set_termios(self.fd, &termios);
    }

    fn set_stop_bits(&mut self, stop_bits: StopBits) -> Result<()> {
        let mut termios = termios::get_termios(self.fd)?;
        termios::set_stop_bits(&mut termios, stop_bits);
        #[cfg(any(target_os = "ios", target_os = "macos"))]
        return termios::set_termios(self.fd, &termios, self.baud_rate);
        #[cfg(not(any(target_os = "ios", target_os = "macos")))]
        return termios::set_termios(self.fd, &termios);
    }

    fn set_read_mode(&mut self, read_mode: ReadMode) -> Result<()> {
        self.read_mode = read_mode;
        Ok(())
    }

    fn set_write_mode(&mut self, write_mode: WriteMode) -> Result<()> {
        self.write_mode = write_mode;
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
        match self.try_clone_native() {
            Ok(p) => Ok(Box::new(p)),
            Err(e) => Err(e),
        }
    }

    fn set_break(&self) -> Result<()> {
        ioctl::tiocsbrk(self.fd)
    }

    fn clear_break(&self) -> Result<()> {
        ioctl::tioccbrk(self.fd)
    }
}

#[test]
fn test_ttyport_into_raw_fd() {
    // `master` must be used here as Dropping it causes slave to be deleted by the OS.
    // TODO: Convert this to a statement-level attribute once
    //       https://github.com/rust-lang/rust/issues/15701 is on stable.
    // FIXME: Create a mutex across all tests for using `TTYPort::pair()` as it's not threadsafe
    #![allow(unused_variables)]
    use std::mem::MaybeUninit;
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
