//! Provides unix-only extensions to the SerialPort type.

use crate::Result;

/// Specifies the duration of a transmission break.
#[derive(Clone, Copy, Debug)]
pub enum BreakDuration {
    /// 0.25-0.5s
    Short,
    /// Specifies a break duration that is platform-dependent
    Arbitrary(std::num::NonZeroI32),
}

/// Posix-only extensions to the SerialPort type.
pub trait SerialPortExt {
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
    /// use serialport::SerialPort;
    ///
    /// let (master, slave) = SerialPort::pair().unwrap();
    /// ```
    fn pair() -> Result<(Self, Self)> where Self: Sized;

    /// Returns the exclusivity of the port
    ///
    /// If a port is exclusive, then trying to open the same device path again
    /// will fail.
    fn exclusive(&self) -> bool;

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
    fn set_exclusive(&mut self, exclusive: bool) -> Result<()>;

    /// Sends 0-valued bits over the port for a set duration
    fn send_break(&self, duration: BreakDuration) -> Result<()>;
}