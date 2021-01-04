use std::convert::TryFrom;
use std::mem::MaybeUninit;
use std::num::NonZeroU16;
use std::os::windows::prelude::*;
use std::time::Duration;
use std::{io, ptr};

use winapi::shared::minwindef::*;
use winapi::um::commapi::*;
use winapi::um::fileapi::*;
use winapi::um::handleapi::*;
use winapi::um::processthreadsapi::GetCurrentProcess;
use winapi::um::winbase::*;
use winapi::um::winnt::{
    DUPLICATE_SAME_ACCESS, FILE_ATTRIBUTE_NORMAL, GENERIC_READ, GENERIC_WRITE, HANDLE, MAXDWORD,
};

use crate::windows::dcb;
use crate::{
    ClearBuffer, DataBits, Error, ErrorKind, FlowControl, Parity, ReadMode, Result, SerialPort,
    SerialPortBuilder, StopBits, WriteMode,
};

/// A serial port implementation for Windows COM ports
///
/// The port will be closed when the value is dropped. However, this struct
/// should not be instantiated directly by using `COMPort::open()`, instead use
/// the cross-platform `serialport::open()` or
/// `serialport::open_with_settings()`.
#[derive(Debug)]
pub struct COMPort {
    handle: HANDLE,
    timeout: Duration,
    port_name: Option<String>,
}

unsafe impl Send for COMPort {}

impl COMPort {
    /// Opens a COM port as a serial device.
    ///
    /// `port` should be the name of a COM port, e.g., `COM1`.
    ///
    /// If the COM port handle needs to be opened with special flags, use
    /// `from_raw_handle` method to create the `COMPort`. Note that you should
    /// set the different settings before using the serial port using `set_all`.
    ///
    /// ## Errors
    ///
    /// * `NoDevice` if the device could not be opened. This could indicate that
    ///    the device is already in use.
    /// * `InvalidInput` if `port` is not a valid device name.
    /// * `Io` for any other I/O error while opening or initializing the device.
    pub fn open(builder: &SerialPortBuilder) -> Result<COMPort> {
        let mut name = Vec::<u16>::with_capacity(4 + builder.path.len() + 1);

        name.extend(r"\\.\".encode_utf16());
        name.extend(builder.path.encode_utf16());
        name.push(0);

        let handle = unsafe {
            CreateFileW(
                name.as_ptr(),
                GENERIC_READ | GENERIC_WRITE,
                0,
                ptr::null_mut(),
                OPEN_EXISTING,
                FILE_ATTRIBUTE_NORMAL,
                0 as HANDLE,
            )
        };

        if handle == INVALID_HANDLE_VALUE {
            return Err(super::error::last_os_error());
        }

        let mut dcb = dcb::get_dcb(handle)?;
        dcb::set_baud_rate(&mut dcb, builder.baud_rate);
        dcb::set_data_bits(&mut dcb, builder.data_bits);
        dcb::set_parity(&mut dcb, builder.parity);
        dcb::set_stop_bits(&mut dcb, builder.stop_bits);
        dcb::set_flow_control(&mut dcb, builder.flow_control);
        dcb::set_dcb(handle, dcb)?;

        let mut com = COMPort::open_from_raw_handle(handle as RawHandle);
        com.set_read_mode(builder.read_mode)?;
        com.set_write_mode(builder.write_mode)?;
        com.port_name = Some(builder.path.clone());
        Ok(com)
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
    pub fn try_clone_native(&self) -> Result<COMPort> {
        let process_handle: HANDLE = unsafe { GetCurrentProcess() };
        let mut cloned_handle: HANDLE = INVALID_HANDLE_VALUE;
        unsafe {
            DuplicateHandle(
                process_handle,
                self.handle,
                process_handle,
                &mut cloned_handle,
                0,
                TRUE,
                DUPLICATE_SAME_ACCESS,
            );
        }
        if cloned_handle == INVALID_HANDLE_VALUE {
            Err(super::error::last_os_error())
        } else {
            Ok(COMPort {
                handle: cloned_handle,
                port_name: self.port_name.clone(),
                timeout: self.timeout,
            })
        }
    }

    fn escape_comm_function(&mut self, function: DWORD) -> Result<()> {
        match unsafe { EscapeCommFunction(self.handle, function) } {
            0 => Err(super::error::last_os_error()),
            _ => Ok(()),
        }
    }

    fn read_pin(&mut self, pin: DWORD) -> Result<bool> {
        let mut status: DWORD = 0;

        match unsafe { GetCommModemStatus(self.handle, &mut status) } {
            0 => Err(super::error::last_os_error()),
            _ => Ok(status & pin != 0),
        }
    }

    fn open_from_raw_handle(handle: RawHandle) -> Self {
        // It is not trivial to get the file path corresponding to a handle.
        // We'll punt and set it `None` here.
        COMPort {
            handle: handle as HANDLE,
            timeout: Duration::from_millis(100),
            port_name: None,
        }
    }
}

impl Drop for COMPort {
    fn drop(&mut self) {
        unsafe {
            CloseHandle(self.handle);
        }
    }
}

impl AsRawHandle for COMPort {
    fn as_raw_handle(&self) -> RawHandle {
        self.handle as RawHandle
    }
}

impl FromRawHandle for COMPort {
    unsafe fn from_raw_handle(handle: RawHandle) -> Self {
        COMPort::open_from_raw_handle(handle)
    }
}

impl io::Read for COMPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let mut len: DWORD = 0;

        match unsafe {
            ReadFile(
                self.handle,
                buf.as_mut_ptr() as LPVOID,
                buf.len() as DWORD,
                &mut len,
                ptr::null_mut(),
            )
        } {
            0 => Err(io::Error::last_os_error()),
            _ => {
                if len != 0 {
                    Ok(len as usize)
                } else {
                    Err(io::Error::new(
                        io::ErrorKind::TimedOut,
                        "Operation timed out",
                    ))
                }
            }
        }
    }
}

impl io::Write for COMPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let mut len: DWORD = 0;

        match unsafe {
            WriteFile(
                self.handle,
                buf.as_ptr() as LPVOID,
                buf.len() as DWORD,
                &mut len,
                ptr::null_mut(),
            )
        } {
            0 => Err(io::Error::last_os_error()),
            _ => Ok(len as usize),
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        match unsafe { FlushFileBuffers(self.handle) } {
            0 => Err(io::Error::last_os_error()),
            _ => Ok(()),
        }
    }
}

impl SerialPort for COMPort {
    fn name(&self) -> Option<String> {
        self.port_name.clone()
    }

    fn write_request_to_send(&mut self, level: bool) -> Result<()> {
        if level {
            self.escape_comm_function(SETRTS)
        } else {
            self.escape_comm_function(CLRRTS)
        }
    }

    fn write_data_terminal_ready(&mut self, level: bool) -> Result<()> {
        if level {
            self.escape_comm_function(SETDTR)
        } else {
            self.escape_comm_function(CLRDTR)
        }
    }

    fn read_clear_to_send(&mut self) -> Result<bool> {
        self.read_pin(MS_CTS_ON)
    }

    fn read_data_set_ready(&mut self) -> Result<bool> {
        self.read_pin(MS_DSR_ON)
    }

    fn read_ring_indicator(&mut self) -> Result<bool> {
        self.read_pin(MS_RING_ON)
    }

    fn read_carrier_detect(&mut self) -> Result<bool> {
        self.read_pin(MS_RLSD_ON)
    }

    fn baud_rate(&self) -> Result<u32> {
        let dcb = dcb::get_dcb(self.handle)?;
        Ok(dcb.BaudRate as u32)
    }

    fn data_bits(&self) -> Result<DataBits> {
        let dcb = dcb::get_dcb(self.handle)?;
        match dcb.ByteSize {
            5 => Ok(DataBits::Five),
            6 => Ok(DataBits::Six),
            7 => Ok(DataBits::Seven),
            8 => Ok(DataBits::Eight),
            _ => Err(Error::new(
                ErrorKind::Unknown,
                "Invalid data bits setting encountered",
            )),
        }
    }

    fn parity(&self) -> Result<Parity> {
        let dcb = dcb::get_dcb(self.handle)?;
        match dcb.Parity {
            ODDPARITY => Ok(Parity::Odd),
            EVENPARITY => Ok(Parity::Even),
            NOPARITY => Ok(Parity::None),
            _ => Err(Error::new(
                ErrorKind::Unknown,
                "Invalid parity bits setting encountered",
            )),
        }
    }

    fn stop_bits(&self) -> Result<StopBits> {
        let dcb = dcb::get_dcb(self.handle)?;
        match dcb.StopBits {
            TWOSTOPBITS => Ok(StopBits::Two),
            ONESTOPBIT => Ok(StopBits::One),
            _ => Err(Error::new(
                ErrorKind::Unknown,
                "Invalid stop bits setting encountered",
            )),
        }
    }

    fn flow_control(&self) -> Result<FlowControl> {
        let dcb = dcb::get_dcb(self.handle)?;
        if dcb.fOutxCtsFlow() != 0 || dcb.fRtsControl() != 0 {
            Ok(FlowControl::Hardware)
        } else if dcb.fOutX() != 0 || dcb.fInX() != 0 {
            Ok(FlowControl::Software)
        } else {
            Ok(FlowControl::None)
        }
    }

    fn read_mode(&self) -> Result<ReadMode> {
        let mut timeouts = MaybeUninit::uninit();
        if unsafe { GetCommTimeouts(self.handle, timeouts.as_mut_ptr()) == 0 } {
            return Err(super::error::last_os_error());
        }
        let timeouts = unsafe { timeouts.assume_init() };

        match timeouts {
            COMMTIMEOUTS {ReadIntervalTimeout: MAXDWORD, ReadTotalTimeoutMultiplier:        0, ReadTotalTimeoutConstant: 0, .. } => Ok(ReadMode::Immediate),
            COMMTIMEOUTS {ReadIntervalTimeout:        0, ReadTotalTimeoutMultiplier:        0, ReadTotalTimeoutConstant: 0, .. } => Ok(ReadMode::Blocking),
            COMMTIMEOUTS {ReadIntervalTimeout: MAXDWORD, ReadTotalTimeoutMultiplier: MAXDWORD, ReadTotalTimeoutConstant: t, .. } => {
                let timeout = u16::try_from(t).map_err(|_| Error::new(ErrorKind::InvalidInput, "Value too big for a u16"))?;
                let timeout = NonZeroU16::new(timeout);
                if timeout.is_none() {
                    return Err(Error::new(ErrorKind::InvalidInput, "Value can't be zero"));
                }
                Ok(ReadMode::Timeout(timeout.unwrap()))
            },
            _ => return Err(Error::new(ErrorKind::Unknown, "Unknown timeout configuration"))
        }
    }

    fn write_mode(&self) -> Result<WriteMode> {
        let mut timeouts = MaybeUninit::uninit();
        if unsafe { GetCommTimeouts(self.handle, timeouts.as_mut_ptr()) == 0 } {
            return Err(super::error::last_os_error());
        }
        let timeouts = unsafe { timeouts.assume_init() };

        match timeouts {
            COMMTIMEOUTS {WriteTotalTimeoutMultiplier:        0, WriteTotalTimeoutConstant: MAXDWORD, .. } => Ok(WriteMode::Immediate),
            COMMTIMEOUTS {WriteTotalTimeoutMultiplier:        0, WriteTotalTimeoutConstant: 0, .. } => Ok(WriteMode::Blocking),
            COMMTIMEOUTS {WriteTotalTimeoutMultiplier: MAXDWORD, WriteTotalTimeoutConstant: t, .. } => {
                let timeout = u16::try_from(t).map_err(|_| Error::new(ErrorKind::InvalidInput, "Value too big for a u16"))?;
                let timeout = NonZeroU16::new(timeout);
                if timeout.is_none() {
                    return Err(Error::new(ErrorKind::InvalidInput, "Value can't be zero"));
                }
                Ok(WriteMode::Timeout(timeout.unwrap()))
            }
            _ => return Err(Error::new(ErrorKind::Unknown, "Unknown timeout configuration"))
        }
    }

    fn set_baud_rate(&mut self, baud_rate: u32) -> Result<()> {
        let mut dcb = dcb::get_dcb(self.handle)?;
        dcb::set_baud_rate(&mut dcb, baud_rate);
        dcb::set_dcb(self.handle, dcb)
    }

    fn set_data_bits(&mut self, data_bits: DataBits) -> Result<()> {
        let mut dcb = dcb::get_dcb(self.handle)?;
        dcb::set_data_bits(&mut dcb, data_bits);
        dcb::set_dcb(self.handle, dcb)
    }

    fn set_parity(&mut self, parity: Parity) -> Result<()> {
        let mut dcb = dcb::get_dcb(self.handle)?;
        dcb::set_parity(&mut dcb, parity);
        dcb::set_dcb(self.handle, dcb)
    }

    fn set_stop_bits(&mut self, stop_bits: StopBits) -> Result<()> {
        let mut dcb = dcb::get_dcb(self.handle)?;
        dcb::set_stop_bits(&mut dcb, stop_bits);
        dcb::set_dcb(self.handle, dcb)
    }

    fn set_flow_control(&mut self, flow_control: FlowControl) -> Result<()> {
        let mut dcb = dcb::get_dcb(self.handle)?;
        dcb::set_flow_control(&mut dcb, flow_control);
        dcb::set_dcb(self.handle, dcb)
    }

    fn set_read_mode(&mut self, read_mode: ReadMode) -> Result<()> {
        let mut timeouts = MaybeUninit::uninit();
        if unsafe { GetCommTimeouts(self.handle, timeouts.as_mut_ptr()) == 0 } {
            return Err(super::error::last_os_error());
        }
        let mut timeouts = unsafe { timeouts.assume_init() };

        match read_mode {
            ReadMode::Immediate => {
                timeouts.ReadIntervalTimeout = MAXDWORD;
                timeouts.ReadTotalTimeoutMultiplier = 0;
                timeouts.ReadTotalTimeoutConstant = 0;
            },
            ReadMode::Blocking => {
                timeouts.ReadIntervalTimeout = 0;
                timeouts.ReadTotalTimeoutMultiplier = 0;
                timeouts.ReadTotalTimeoutConstant = 0;
            },
            ReadMode::Timeout(t) => {
                timeouts.ReadIntervalTimeout = MAXDWORD;
                timeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
                timeouts.ReadTotalTimeoutConstant = t.get() as DWORD;
            },
        };

        if unsafe { SetCommTimeouts(self.handle, &mut timeouts) } == 0 {
            return Err(super::error::last_os_error());
        }

        Ok(())
    }

    fn set_write_mode(&mut self, write_mode: WriteMode) -> Result<()> {
        let mut timeouts = MaybeUninit::uninit();
        if unsafe { GetCommTimeouts(self.handle, timeouts.as_mut_ptr()) == 0 } {
            return Err(super::error::last_os_error());
        }
        let mut timeouts = unsafe { timeouts.assume_init() };

        match write_mode {
            WriteMode::Immediate => {
                timeouts.WriteTotalTimeoutMultiplier = 0;
                timeouts.WriteTotalTimeoutConstant = MAXDWORD;
            },
            WriteMode::Blocking => {
                timeouts.WriteTotalTimeoutMultiplier = 0;
                timeouts.WriteTotalTimeoutConstant = 0;
            },
            WriteMode::Timeout(t) => {
                timeouts.WriteTotalTimeoutMultiplier = MAXDWORD;
                timeouts.WriteTotalTimeoutConstant = t.get() as DWORD;
            },
        };

        if unsafe { SetCommTimeouts(self.handle, &mut timeouts) } == 0 {
            return Err(super::error::last_os_error());
        }

        Ok(())
    }

    fn bytes_to_read(&self) -> Result<u32> {
        let mut errors: DWORD = 0;
        let mut comstat = MaybeUninit::uninit();

        if unsafe { ClearCommError(self.handle, &mut errors, comstat.as_mut_ptr()) != 0 } {
            unsafe { Ok(comstat.assume_init().cbInQue) }
        } else {
            Err(super::error::last_os_error())
        }
    }

    fn bytes_to_write(&self) -> Result<u32> {
        let mut errors: DWORD = 0;
        let mut comstat = MaybeUninit::uninit();

        if unsafe { ClearCommError(self.handle, &mut errors, comstat.as_mut_ptr()) != 0 } {
            unsafe { Ok(comstat.assume_init().cbOutQue) }
        } else {
            Err(super::error::last_os_error())
        }
    }

    fn clear(&self, buffer_to_clear: ClearBuffer) -> Result<()> {
        let buffer_flags = match buffer_to_clear {
            ClearBuffer::Input => PURGE_RXABORT | PURGE_RXCLEAR,
            ClearBuffer::Output => PURGE_TXABORT | PURGE_TXCLEAR,
            ClearBuffer::All => PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR,
        };

        if unsafe { PurgeComm(self.handle, buffer_flags) != 0 } {
            Ok(())
        } else {
            Err(super::error::last_os_error())
        }
    }

    fn try_clone(&self) -> Result<Box<dyn SerialPort>> {
        match self.try_clone_native() {
            Ok(p) => Ok(Box::new(p)),
            Err(e) => Err(e),
        }
    }

    fn set_break(&self) -> Result<()> {
        if unsafe { SetCommBreak(self.handle) != 0 } {
            Ok(())
        } else {
            Err(super::error::last_os_error())
        }
    }

    fn clear_break(&self) -> Result<()> {
        if unsafe { ClearCommBreak(self.handle) != 0 } {
            Ok(())
        } else {
            Err(super::error::last_os_error())
        }
    }
}
