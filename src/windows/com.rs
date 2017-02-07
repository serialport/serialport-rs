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
use ::{BaudRate, DataBits, FlowControl, Parity, SerialPort, SerialPortInfo, SerialPortSettings,
       StopBits};


/// A serial port implementation for Windows COM ports.
///
/// The port will be closed when the value is dropped. However, this struct
/// should not be instantiated directly by using `COMPort::open()`, instead use
/// the cross-platform `serialport::open()` or
/// `serialport::open_with_settings()`.
pub struct COMPort {
    handle: HANDLE,
    inner: DCB,
    timeout: Duration,
    port_name: Option<String>,
}

unsafe impl Send for COMPort {}

impl COMPort {
    /// Opens a COM port as a serial device.
    ///
    /// `port` should be the name of a COM port, e.g., `COM1`.
    ///
    /// ## Errors
    ///
    /// * `NoDevice` if the device could not be opened. This could indicate that
    ///    the device is already in use.
    /// * `InvalidInput` if `port` is not a valid device name.
    /// * `Io` for any other I/O error while opening or initializing the device.
    pub fn open<T: AsRef<OsStr> + ?Sized>(port: &T,
                                          settings: &SerialPortSettings)
                                          -> ::Result<COMPort> {
        let mut name = Vec::<u16>::new();

        name.extend(OsStr::new("\\\\.\\").encode_wide());
        name.extend(port.as_ref().encode_wide());
        name.push(0);

        let handle = unsafe {
            CreateFileW(name.as_ptr(),
                        GENERIC_READ | GENERIC_WRITE,
                        0,
                        ptr::null_mut(),
                        OPEN_EXISTING,
                        FILE_ATTRIBUTE_NORMAL,
                        0 as HANDLE)
        };

        if handle != INVALID_HANDLE_VALUE {

            let timeout = Duration::from_millis(100);

            let mut dcb = DCB::new();

            match unsafe { GetCommState(handle, &mut dcb) } {
                0 => return Err(super::error::last_os_error()),
                _ => (),

            }

            let mut port = COMPort {
                handle: handle,
                inner: dcb,
                timeout: timeout,
                port_name: port.as_ref().to_str().map(|s| s.to_string()),
            };

            port.set_all(settings)?;

            Ok(port)
        } else {
            Err(super::error::last_os_error())
        }
    }

    fn write_settings(&mut self) -> ::Result<()> {
        match unsafe { SetCommState(self.handle, &self.inner) } {
            0 => Err(super::error::last_os_error()),
            _ => Ok(()),
        }
    }

    fn escape_comm_function(&mut self, function: DWORD) -> ::Result<()> {
        match unsafe { EscapeCommFunction(self.handle, function) } {
            0 => Err(super::error::last_os_error()),
            _ => Ok(()),
        }
    }

    fn read_pin(&mut self, pin: DWORD) -> ::Result<bool> {
        let mut status: DWORD = unsafe { mem::uninitialized() };

        match unsafe { GetCommModemStatus(self.handle, &mut status) } {
            0 => Err(super::error::last_os_error()),
            _ => Ok(status & pin != 0),
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
        unsafe { mem::transmute(self.handle) }
    }
}

impl io::Read for COMPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let mut len: DWORD = 0;

        match unsafe {
            ReadFile(self.handle,
                     buf.as_mut_ptr() as *mut c_void,
                     buf.len() as DWORD,
                     &mut len,
                     ptr::null_mut())
        } {
            0 => Err(io::Error::last_os_error()),
            _ => {
                if len != 0 {
                    Ok(len as usize)
                } else {
                    Err(io::Error::new(io::ErrorKind::TimedOut, "Operation timed out"))
                }
            }
        }
    }
}

impl io::Write for COMPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let mut len: DWORD = 0;

        match unsafe {
            WriteFile(self.handle,
                      buf.as_ptr() as *mut c_void,
                      buf.len() as DWORD,
                      &mut len,
                      ptr::null_mut())
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
    fn port_name(&self) -> Option<String> {
        self.port_name.clone()
    }

    fn timeout(&self) -> Duration {
        self.timeout
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

    fn set_timeout(&mut self, timeout: Duration) -> ::Result<()> {
        let milliseconds = timeout.as_secs() * 1000 + timeout.subsec_nanos() as u64 / 1_000_000;

        let timeouts = COMMTIMEOUTS {
            ReadIntervalTimeout: 0,
            ReadTotalTimeoutMultiplier: 0,
            ReadTotalTimeoutConstant: milliseconds as DWORD,
            WriteTotalTimeoutMultiplier: 0,
            WriteTotalTimeoutConstant: 0,
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
        } else {
            self.escape_comm_function(CLRRTS)
        }
    }

    fn write_data_terminal_ready(&mut self, level: bool) -> ::Result<()> {
        if level {
            self.escape_comm_function(SETDTR)
        } else {
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

    fn baud_rate(&self) -> Option<BaudRate> {
        match self.inner.BaudRate {
            CBR_110 => Some(BaudRate::Baud110),
            CBR_300 => Some(BaudRate::Baud300),
            CBR_600 => Some(BaudRate::Baud600),
            CBR_1200 => Some(BaudRate::Baud1200),
            CBR_2400 => Some(BaudRate::Baud2400),
            CBR_4800 => Some(BaudRate::Baud4800),
            CBR_9600 => Some(BaudRate::Baud9600),
            CBR_14400 => Some(BaudRate::BaudOther(14400)),
            CBR_19200 => Some(BaudRate::Baud19200),
            CBR_38400 => Some(BaudRate::Baud38400),
            CBR_56000 => Some(BaudRate::BaudOther(56000)),
            CBR_57600 => Some(BaudRate::Baud57600),
            CBR_115200 => Some(BaudRate::Baud115200),
            CBR_128000 => Some(BaudRate::BaudOther(128000)),
            CBR_256000 => Some(BaudRate::BaudOther(256000)),
            n => Some(BaudRate::BaudOther(n as usize)),
        }
    }

    fn data_bits(&self) -> Option<DataBits> {
        match self.inner.ByteSize {
            5 => Some(DataBits::Five),
            6 => Some(DataBits::Six),
            7 => Some(DataBits::Seven),
            8 => Some(DataBits::Eight),
            _ => None,
        }
    }

    fn parity(&self) -> Option<Parity> {
        match self.inner.Parity {
            ODDPARITY => Some(Parity::Odd),
            EVENPARITY => Some(Parity::Even),
            NOPARITY => Some(Parity::None),
            _ => None,
        }
    }

    fn stop_bits(&self) -> Option<StopBits> {
        match self.inner.StopBits {
            TWOSTOPBITS => Some(StopBits::Two),
            ONESTOPBIT => Some(StopBits::One),
            _ => None,
        }
    }

    fn flow_control(&self) -> Option<FlowControl> {
        if self.inner.fBits & (fOutxCtsFlow | fRtsControl) != 0 {
            Some(FlowControl::Hardware)
        } else if self.inner.fBits & (fOutX | fInX) != 0 {
            Some(FlowControl::Software)
        } else {
            Some(FlowControl::None)
        }
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
        self.inner.BaudRate = match baud_rate {
            BaudRate::Baud110 => CBR_110,
            BaudRate::Baud300 => CBR_300,
            BaudRate::Baud600 => CBR_600,
            BaudRate::Baud1200 => CBR_1200,
            BaudRate::Baud2400 => CBR_2400,
            BaudRate::Baud4800 => CBR_4800,
            BaudRate::Baud9600 => CBR_9600,
            BaudRate::Baud19200 => CBR_19200,
            BaudRate::Baud38400 => CBR_38400,
            BaudRate::Baud57600 => CBR_57600,
            BaudRate::Baud115200 => CBR_115200,
            BaudRate::BaudOther(n) => n as DWORD,
        };

        self.write_settings()
    }

    fn set_data_bits(&mut self, data_bits: DataBits) -> ::Result<()> {
        self.inner.ByteSize = match data_bits {
            DataBits::Five => 5,
            DataBits::Six => 6,
            DataBits::Seven => 7,
            DataBits::Eight => 8,
        };

        self.write_settings()
    }

    fn set_parity(&mut self, parity: Parity) -> ::Result<()> {
        self.inner.Parity = match parity {
            Parity::None => NOPARITY,
            Parity::Odd => ODDPARITY,
            Parity::Even => EVENPARITY,
        };

        self.write_settings()
    }

    fn set_stop_bits(&mut self, stop_bits: StopBits) -> ::Result<()> {
        self.inner.StopBits = match stop_bits {
            StopBits::One => ONESTOPBIT,
            StopBits::Two => TWOSTOPBITS,
        };

        self.write_settings()
    }

    fn set_flow_control(&mut self, flow_control: FlowControl) -> ::Result<()> {
        match flow_control {
            FlowControl::None => {
                self.inner.fBits &= !(fOutxCtsFlow | fRtsControl);
                self.inner.fBits &= !(fOutX | fInX);
            }
            FlowControl::Software => {
                self.inner.fBits &= !(fOutxCtsFlow | fRtsControl);
                self.inner.fBits |= fOutX | fInX;
            }
            FlowControl::Hardware => {
                self.inner.fBits |= fOutxCtsFlow | fRtsControl;
                self.inner.fBits &= !(fOutX | fInX);
            }
        }

        self.write_settings()
    }
}

pub fn available_ports() -> ::Result<Vec<SerialPortInfo>> {
    let mut vec = Vec::new();
    let system = RegKey::predef(HKEY_LOCAL_MACHINE)
        .open_subkey_with_flags("HARDWARE\\DEVICEMAP\\SERIALCOMM", KEY_READ)?;
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
