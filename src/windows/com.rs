extern crate libc;
extern crate winreg;

use std::ffi::OsStr;
use std::io;
use std::mem;
use std::os::windows::prelude::*;
use std::ptr;
use std::time::Duration;

use self::winreg::RegKey;
use self::winreg::types::FromRegValue;
use self::winreg::enums::*;
use self::libc::c_void;

use super::ffi::*;
use ::{BaudRate, DataBits, FlowControl, Parity, SerialPort, SerialPortInfo, StopBits};


/// A serial port implementation for Windows COM ports.
///
/// The port will be closed when the value is dropped.
pub struct COMPort {
    handle: HANDLE,
    inner: DCB,
    timeout: Duration
}

unsafe impl Send for COMPort {}

impl COMPort {
    /// Opens a COM port as a serial device.
    ///
    /// `port` should be the name of a COM port, e.g., `COM1`.
    ///
    /// ```no_run
    /// serial::windows::COMPort::open("COM1").unwrap();
    /// ```
    ///
    /// ## Errors
    ///
    /// * `NoDevice` if the device could not be opened. This could indicate that the device is
    ///   already in use.
    /// * `InvalidInput` if `port` is not a valid device name.
    /// * `Io` for any other I/O error while opening or initializing the device.
    pub fn open<T: AsRef<OsStr> + ?Sized>(port: &T) -> ::Result<Box<SerialPort>> {
        let mut name = Vec::<u16>::new();

        name.extend(OsStr::new("\\\\.\\").encode_wide());
        name.extend(port.as_ref().encode_wide());
        name.push(0);

        let handle = unsafe {
            CreateFileW(name.as_ptr(), GENERIC_READ | GENERIC_WRITE, 0, ptr::null_mut(), OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0 as HANDLE)
        };

        if handle != INVALID_HANDLE_VALUE {

            let timeout = Duration::from_millis(100);

            let mut dcb = DCB::new();

            match unsafe { GetCommState(handle, &mut dcb) } {
                0 => return Err(super::error::last_os_error()),
                _ => ()

            }

            let mut port = COMPort {
                handle: handle,
                inner: dcb,
                timeout: timeout
            };

            try!(port.set_timeout(timeout));
            Ok(Box::new(port))
        }
        else {
            Err(super::error::last_os_error())
        }
    }

    fn escape_comm_function(&mut self, function: DWORD) -> ::Result<()> {
        match unsafe { EscapeCommFunction(self.handle, function) } {
            0 => Err(super::error::last_os_error()),
            _ => Ok(())
        }
    }

    fn read_pin(&mut self, pin: DWORD) -> ::Result<bool> {
        let mut status: DWORD = unsafe { mem::uninitialized() };

        match unsafe { GetCommModemStatus(self.handle, &mut status) } {
            0 => Err(super::error::last_os_error()),
            _ => Ok(status & pin != 0)
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
        unsafe {
            mem::transmute(self.handle)
        }
    }
}

impl io::Read for COMPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let mut len: DWORD = 0;

        match unsafe { ReadFile(self.handle, buf.as_mut_ptr() as *mut c_void, buf.len() as DWORD, &mut len, ptr::null_mut()) } {
            0 => Err(io::Error::last_os_error()),
            _ => {
                if len != 0 {
                    Ok(len as usize)
                }
                else {
                    Err(io::Error::new(io::ErrorKind::TimedOut, "Operation timed out"))
                }
            }
        }
    }
}

impl io::Write for COMPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let mut len: DWORD = 0;

        match unsafe { WriteFile(self.handle, buf.as_ptr() as *mut c_void, buf.len() as DWORD, &mut len, ptr::null_mut()) } {
            0 => Err(io::Error::last_os_error()),
            _ => Ok(len as usize)
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        match unsafe { FlushFileBuffers(self.handle) } {
            0 => Err(io::Error::last_os_error()),
            _ => Ok(())
        }
    }
}

impl SerialPort for COMPort {

    fn timeout(&self) -> Duration {
        self.timeout
    }

    fn set_timeout(&mut self, timeout: Duration) -> ::Result<()> {
        let milliseconds = timeout.as_secs() * 1000 + timeout.subsec_nanos() as u64 / 1_000_000;

        let timeouts = COMMTIMEOUTS {
            ReadIntervalTimeout: 0,
            ReadTotalTimeoutMultiplier: 0,
            ReadTotalTimeoutConstant: milliseconds as DWORD,
            WriteTotalTimeoutMultiplier: 0,
            WriteTotalTimeoutConstant: 0
        };

        if unsafe { SetCommTimeouts(self.handle, &timeouts) } == 0 {
            return Err(super::error::last_os_error());
        }

        self.timeout = timeout;
        Ok(())
    }

    fn write_request_to_send(&mut self, level: bool) -> ::Result<()> {
        if level {
            self.escape_comm_function(SETRTS)
        }
        else {
            self.escape_comm_function(CLRRTS)
        }
    }

    fn write_data_terminal_ready(&mut self, level: bool) -> ::Result<()> {
        if level {
            self.escape_comm_function(SETDTR)
        }
        else {
            self.escape_comm_function(CLRDTR)
        }
    }

    fn read_clear_to_send(&mut self) -> ::Result<bool> {
        self.read_pin(MS_CTS_ON)
    }

    fn read_data_set_ready(&mut self) -> ::Result<bool> {
        self.read_pin(MS_DSR_ON)
    }

    fn read_ring_indicator(&mut self) -> ::Result<bool> {
        self.read_pin(MS_RING_ON)
    }

    fn read_carrier_detect(&mut self) -> ::Result<bool> {
        self.read_pin(MS_RLSD_ON)
    }

    fn baud_rate(&self) -> Option<::BaudRate> {
        match self.inner.BaudRate {
            CBR_110    => Some(::Baud110),
            CBR_300    => Some(::Baud300),
            CBR_600    => Some(::Baud600),
            CBR_1200   => Some(::Baud1200),
            CBR_2400   => Some(::Baud2400),
            CBR_4800   => Some(::Baud4800),
            CBR_9600   => Some(::Baud9600),
            CBR_14400  => Some(::BaudOther(14400)),
            CBR_19200  => Some(::Baud19200),
            CBR_38400  => Some(::Baud38400),
            CBR_56000  => Some(::BaudOther(56000)),
            CBR_57600  => Some(::Baud57600),
            CBR_115200 => Some(::Baud115200),
            CBR_128000 => Some(::BaudOther(128000)),
            CBR_256000 => Some(::BaudOther(256000)),
            n          => Some(::BaudOther(n as usize))
        }
    }

    fn data_bits(&self) -> Option<DataBits> {
        match self.inner.ByteSize {
            5 => Some(DataBits::Five),
            6 => Some(DataBits::Six),
            7 => Some(DataBits::Seven),
            8 => Some(DataBits::Eight),
            _ => None
        }
    }

    fn parity(&self) -> Option<Parity> {
        match self.inner.Parity {
            ODDPARITY  => Some(Parity::Odd),
            EVENPARITY => Some(Parity::Even),
            NOPARITY   => Some(Parity::None),
            _          => None
        }
    }

    fn stop_bits(&self) -> Option<StopBits> {
        match self.inner.StopBits {
            TWOSTOPBITS => Some(StopBits::Two),
            ONESTOPBIT  => Some(StopBits::One),
            _           => None
        }
    }

    fn flow_control(&self) -> Option<FlowControl> {
        if self.inner.fBits & (fOutxCtsFlow | fRtsControl) != 0 {
            Some(FlowControl::Hardware)
        }
        else if self.inner.fBits & (fOutX | fInX) != 0 {
            Some(FlowControl::Software)
        }
        else {
            Some(FlowControl::None)
        }
    }

    fn set_baud_rate(&mut self, baud_rate: BaudRate) -> ::Result<()> {
        self.inner.BaudRate = match baud_rate {
            ::Baud110      => CBR_110,
            ::Baud300      => CBR_300,
            ::Baud600      => CBR_600,
            ::Baud1200     => CBR_1200,
            ::Baud2400     => CBR_2400,
            ::Baud4800     => CBR_4800,
            ::Baud9600     => CBR_9600,
            ::Baud19200    => CBR_19200,
            ::Baud38400    => CBR_38400,
            ::Baud57600    => CBR_57600,
            ::Baud115200   => CBR_115200,
            ::BaudOther(n) => n as DWORD
        };

        Ok(())
    }

    fn set_data_bits(&mut self, data_bits: DataBits) -> ::Result<()>  {
        self.inner.ByteSize = match data_bits {
            DataBits::Five => 5,
            DataBits::Six => 6,
            DataBits::Seven => 7,
            DataBits::Eight => 8
        };
        Ok(())
    }

    fn set_parity(&mut self, parity: Parity) -> ::Result<()>  {
        self.inner.Parity = match parity {
            Parity::None => NOPARITY,
            Parity::Odd  => ODDPARITY,
            Parity::Even => EVENPARITY
        };
        Ok(())
    }

    fn set_stop_bits(&mut self, stop_bits: StopBits) -> ::Result<()>  {
        self.inner.StopBits = match stop_bits {
            StopBits::One => ONESTOPBIT,
            StopBits::Two => TWOSTOPBITS
        };
        Ok(())
    }

    fn set_flow_control(&mut self, flow_control: FlowControl) -> ::Result<()>  {
        match flow_control {
            FlowControl::None => {
                self.inner.fBits &= !(fOutxCtsFlow | fRtsControl);
                self.inner.fBits &= !(fOutX | fInX);
            },
            FlowControl::Software => {
                self.inner.fBits &= !(fOutxCtsFlow | fRtsControl);
                self.inner.fBits |= fOutX | fInX;
            },
            FlowControl::Hardware => {
                self.inner.fBits |= fOutxCtsFlow | fRtsControl;
                self.inner.fBits &= !(fOutX | fInX);
            }
        }
        Ok(())
    }
}

pub fn available_ports() -> ::Result<Vec<SerialPortInfo>> {
    let mut vec = Vec::new();
    let system = try!(RegKey::predef(HKEY_LOCAL_MACHINE)
        .open_subkey_with_flags("HARDWARE\\DEVICEMAP\\SERIALCOMM", KEY_READ));
    for reg_val in system.enum_values() {
        if let Ok((_, value)) = reg_val {
            if let Ok(val_str) = FromRegValue::from_reg_value(&value) {
                vec.push(SerialPortInfo { port_name: val_str });
            }
        }
    }
    Ok(vec)
}

pub fn available_baud_rates() -> Vec<u32> {
    vec![110u32, 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200]
}
