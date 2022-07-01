use std::mem::MaybeUninit;
use std::os::windows::prelude::*;
use std::time::Duration;
use std::{io, ptr};

use winapi::shared::minwindef::*;
use winapi::shared::ntdef::NULL;
use winapi::shared::winerror::{ERROR_IO_PENDING, ERROR_OPERATION_ABORTED};
use winapi::um::commapi::*;
use winapi::um::errhandlingapi::GetLastError;
use winapi::um::fileapi::*;
use winapi::um::handleapi::*;
use winapi::um::ioapiset::GetOverlappedResult;
use winapi::um::minwinbase::OVERLAPPED;
use winapi::um::processthreadsapi::GetCurrentProcess;
use winapi::um::synchapi::CreateEventW;
use winapi::um::winbase::*;
use winapi::um::winnt::{
    DUPLICATE_SAME_ACCESS, FILE_ATTRIBUTE_NORMAL, GENERIC_READ, GENERIC_WRITE, HANDLE,
};

use crate::windows::dcb;
use crate::{
    ClearBuffer, DataBits, Error, ErrorKind, FlowControl, Parity, Result, SerialPort,
    SerialPortBuilder, StopBits,
};

const fn duration_to_win_timeout(time: Duration) -> DWORD {
    time.as_secs()
        .saturating_mul(1000)
        .saturating_add((time.subsec_nanos() as u64).saturating_div(1_000_000)) as DWORD
}

struct OverlappedHandle(pub HANDLE);

impl OverlappedHandle {
    #[inline]
    fn new() -> io::Result<Self> {
        match unsafe { CreateEventW(ptr::null_mut(), TRUE, FALSE, ptr::null_mut()) } {
            NULL => Err(io::Error::last_os_error()),
            handle => Ok(Self(handle)),
        }
    }

    #[inline]
    fn close(self) {
        //drop
    }

    #[inline]
    fn create_overlapped(&self) -> OVERLAPPED {
        OVERLAPPED {
            Internal: 0,
            InternalHigh: 0,
            u: unsafe { MaybeUninit::zeroed().assume_init() },
            hEvent: self.0,
        }
    }
}

impl Drop for OverlappedHandle {
    #[inline(always)]
    fn drop(&mut self) {
        unsafe {
            CloseHandle(self.0);
        }
    }
}

/// A serial port implementation for Windows COM ports
///
/// The port will be closed when the value is dropped. However, this struct
/// should not be instantiated directly by using `COMPort::open()`, instead use
/// the cross-platform `serialport::open()` or
/// `serialport::open_with_settings()`.
///
/// Port is created using `CreateFileW` syscall with following set of flags: `FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED`
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
                FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
                0 as HANDLE,
            )
        };

        if handle == INVALID_HANDLE_VALUE {
            return Err(super::error::last_os_error());
        }

        // create the COMPort here so the handle is getting closed
        // if one of the calls to `get_dcb()` or `set_dcb()` fails
        let mut com = COMPort::open_from_raw_handle(handle as RawHandle);

        let mut dcb = dcb::get_dcb(handle)?;
        dcb::init(&mut dcb);
        dcb::set_baud_rate(&mut dcb, builder.baud_rate);
        dcb::set_data_bits(&mut dcb, builder.data_bits);
        dcb::set_parity(&mut dcb, builder.parity);
        dcb::set_stop_bits(&mut dcb, builder.stop_bits);
        dcb::set_flow_control(&mut dcb, builder.flow_control);
        dcb::set_dcb(handle, dcb)?;

        com.set_timeout(builder.timeout)?;
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
        // duplicate communications device handle
        let mut duplicate_handle = INVALID_HANDLE_VALUE;
        let process = unsafe { GetCurrentProcess() };
        let res = unsafe {
            DuplicateHandle(
                process,
                self.handle,
                process,
                &mut duplicate_handle,
                0,
                FALSE,
                DUPLICATE_SAME_ACCESS,
            )
        };

        match res {
            0 => Err(super::error::last_os_error()),
            _ => Ok(COMPort {
                handle: duplicate_handle,
                port_name: self.port_name.clone(),
                timeout: self.timeout,
            }),
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

    ///Sets COM port timeouts.
    ///
    ///Comparing to `SerialPort::set_timeout` which only sets `read` timeout, this function allows
    ///to specify all available timeouts.
    ///
    ///- `data` - This timeout specifies how long to wait for next byte, since arrival of at least
    ///one byte. Once timeout expires, read returns with available data.
    ///`SerialPort::set_timeout` uses 0 which means no timeout.
    ///- `read` - Specifies overall timeout for `read` as `SerialPort::set_timeout`
    ///- `write` - Specifies overall timeout for `write` operations.
    pub fn set_timeouts(&mut self, data: Duration, read: Duration, write: Duration) -> Result<()> {
        let mut timeouts = COMMTIMEOUTS {
            ReadIntervalTimeout: duration_to_win_timeout(data),
            ReadTotalTimeoutMultiplier: 0,
            ReadTotalTimeoutConstant: duration_to_win_timeout(read),
            WriteTotalTimeoutMultiplier: 0,
            WriteTotalTimeoutConstant: duration_to_win_timeout(write),
        };

        if unsafe { SetCommTimeouts(self.handle, &mut timeouts) } == 0 {
            return Err(super::error::last_os_error());
        }

        self.timeout = read;
        Ok(())
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
        let mut read_size = buf.len();

        if self.timeout.as_secs() == 0 && self.timeout.subsec_nanos() == 0 {
            //If zero timeout then make sure we can read, before proceeding
            //Note that zero timeout will make read operation to wait until at least
            //1 byte becomes available.
            let bytes_to_read = self.bytes_to_read()? as usize;
            if bytes_to_read < read_size {
                read_size = bytes_to_read;
            }
            if read_size == 0 {
                return Ok(0);
            }
        }

        let evt_handle = OverlappedHandle::new()?;
        let mut overlapped = evt_handle.create_overlapped();
        let mut len: DWORD = 0;
        let read_result = unsafe {
            ReadFile(
                self.handle,
                buf.as_mut_ptr() as LPVOID,
                read_size as DWORD,
                &mut len,
                &mut overlapped,
            )
        };
        let last_error = unsafe { GetLastError() };
        if read_result == 0
            && last_error != ERROR_IO_PENDING
            && last_error != ERROR_OPERATION_ABORTED
        {
            return Err(io::Error::last_os_error());
        }
        let overlapped_result =
            unsafe { GetOverlappedResult(self.handle, &mut overlapped, &mut len, TRUE) };
        evt_handle.close();
        let last_error = unsafe { GetLastError() };
        if overlapped_result == 0 && last_error != ERROR_OPERATION_ABORTED {
            return Err(io::Error::last_os_error());
        }
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

impl io::Write for COMPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let evt_handle = OverlappedHandle::new()?;
        let mut overlapped = evt_handle.create_overlapped();
        let mut len: DWORD = 0;
        let write_result = unsafe {
            WriteFile(
                self.handle,
                buf.as_ptr() as LPVOID,
                buf.len() as DWORD,
                &mut len,
                &mut overlapped,
            )
        };
        let last_error = unsafe { GetLastError() };
        if write_result == 0
            && last_error != ERROR_IO_PENDING
            && last_error != ERROR_OPERATION_ABORTED
        {
            return Err(io::Error::last_os_error());
        }
        let overlapped_result =
            unsafe { GetOverlappedResult(self.handle, &mut overlapped, &mut len, TRUE) };
        evt_handle.close();

        let last_error = unsafe { GetLastError() };
        if overlapped_result == 0 && last_error != ERROR_OPERATION_ABORTED {
            return Err(io::Error::last_os_error());
        }
        Ok(len as usize)
    }

    fn flush(&mut self) -> io::Result<()> {
        match unsafe { FlushFileBuffers(self.handle) } {
            0 => Err(io::Error::last_os_error()),
            _ => Ok(()),
        }
    }
}

impl SerialPort for COMPort {
    #[inline]
    fn name(&self) -> Option<String> {
        self.port_name.clone()
    }

    #[inline]
    fn timeout(&self) -> Duration {
        self.timeout
    }

    #[inline]
    fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        self.set_timeouts(Duration::from_secs(0), timeout, Duration::from_secs(0))
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
