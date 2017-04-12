use regex::Regex;

use std::ffi::{CStr, CString, OsStr};
use std::io;
use std::mem;
use std::os::windows::prelude::*;
use std::ptr;
use std::time::Duration;

use advapi32::*;
use kernel32::*;
use setupapi::*;
use winapi::*;

use super::ffi::*;
use {BaudRate, DataBits, FlowControl, Parity, SerialPort, SerialPortInfo, SerialPortSettings,
     StopBits};
use {Error, ErrorKind};

// GUID_NULL is defined in winapi 0.3, but we're still using 0.2.8, so we declare it here.
// This can be removed once we move to winapi 0.3
const GUID_NULL: GUID = GUID {
    Data1: 0,
    Data2: 0,
    Data3: 0,
    Data4: [0; 8],
};

/// A serial port implementation for Windows COM ports.
///
/// The port will be closed when the value is dropped. However, this struct
/// should not be instantiated directly by using `COMPort::open()`, instead use
/// the cross-platform `serialport::open()` or
/// `serialport::open_with_settings()`.
#[derive(Debug)]
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

            let mut dcb: DCB = unsafe { mem::uninitialized() };

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
        // Remove mut from &mut self.inner once the follow is resolved:
        // https://github.com/retep998/winapi-rs/issues/383
        match unsafe { SetCommState(self.handle, &mut self.inner) } {
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
                           buf.as_mut_ptr() as LPVOID,
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
                            buf.as_ptr() as LPVOID,
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

        let mut timeouts = COMMTIMEOUTS {
            ReadIntervalTimeout: 0,
            ReadTotalTimeoutMultiplier: 0,
            ReadTotalTimeoutConstant: milliseconds as DWORD,
            WriteTotalTimeoutMultiplier: 0,
            WriteTotalTimeoutConstant: 0,
        };

        // Remove mut from &mut self.inner once the follow is resolved:
        // https://github.com/retep998/winapi-rs/issues/383
        if unsafe { SetCommTimeouts(self.handle, &mut timeouts) } == 0 {
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
            CBR_14400 => Some(BaudRate::Baud14400),
            CBR_19200 => Some(BaudRate::Baud19200),
            CBR_38400 => Some(BaudRate::Baud38400),
            CBR_57600 => Some(BaudRate::Baud57600),
            CBR_115200 => Some(BaudRate::Baud115200),
            CBR_128000 => Some(BaudRate::Baud128000),
            CBR_256000 => Some(BaudRate::Baud256000),
            n => Some(BaudRate::BaudOther(n as u32)),
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
        let parity: u32 = self.inner.Parity as u32;
        match parity {
            ODDPARITY => Some(Parity::Odd),
            EVENPARITY => Some(Parity::Even),
            NOPARITY => Some(Parity::None),
            _ => None,
        }
    }

    fn stop_bits(&self) -> Option<StopBits> {
        let stop_bits: u32 = self.inner.StopBits as u32;
        match stop_bits {
            TWOSTOPBITS => Some(StopBits::Two),
            ONESTOPBIT => Some(StopBits::One),
            _ => None,
        }
    }

    fn flow_control(&self) -> Option<FlowControl> {
        if self.inner.fOutxCtsFlow() != 0 || self.inner.fRtsControl() != 0 {
            Some(FlowControl::Hardware)
        } else if self.inner.fOutX() != 0 || self.inner.fInX() != 0 {
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
            BaudRate::Baud14400 => CBR_14400,
            BaudRate::Baud19200 => CBR_19200,
            BaudRate::Baud38400 => CBR_38400,
            BaudRate::Baud57600 => CBR_57600,
            BaudRate::Baud115200 => CBR_115200,
            BaudRate::Baud128000 => CBR_128000,
            BaudRate::Baud256000 => CBR_256000,
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
            Parity::None => NOPARITY as u8,
            Parity::Odd => ODDPARITY as u8,
            Parity::Even => EVENPARITY as u8,
        };

        self.write_settings()
    }

    fn set_stop_bits(&mut self, stop_bits: StopBits) -> ::Result<()> {
        self.inner.StopBits = match stop_bits {
            StopBits::One => ONESTOPBIT as u8,
            StopBits::Two => TWOSTOPBITS as u8,
        };

        self.write_settings()
    }

    fn set_flow_control(&mut self, flow_control: FlowControl) -> ::Result<()> {
        match flow_control {
            FlowControl::None => {
                self.inner.set_fOutxCtsFlow(0);
                self.inner.set_fRtsControl(0);
                self.inner.set_fOutX(0);
                self.inner.set_fInX(0);
            }
            FlowControl::Software => {
                self.inner.set_fOutxCtsFlow(0);
                self.inner.set_fRtsControl(0);
                self.inner.set_fOutX(1);
                self.inner.set_fInX(1);
            }
            FlowControl::Hardware => {
                self.inner.set_fOutxCtsFlow(1);
                self.inner.set_fRtsControl(1);
                self.inner.set_fOutX(0);
                self.inner.set_fInX(0);
            }
        }

        self.write_settings()
    }
}

// According to the MSDN docs, we should use SetupDiGetClassDevs, SetupDiEnumDeviceInfo
// and SetupDiGetDeviceInstanceId in order to enumerate devices.
// https://msdn.microsoft.com/en-us/windows/hardware/drivers/install/enumerating-installed-devices
//
// SetupDiGetClassDevs returns the devices associated with a particular class of devices.
// We want the list of devices which shows up in the Device Manager as "Ports (COM & LPT)"
// which is otherwise known as the "Ports" class.
//
// get_pots_guids returns all of the classes (guids) associated with the name "Ports".
fn get_ports_guids() -> ::Result<Vec<GUID>> {
    // Note; unwrap can't fail, since "Ports" is valid UTF-8.
    let ports_class_name = CString::new("Ports").unwrap();

    // Size vector to hold 1 result (which is the most common result).
    let mut num_guids: DWORD = 0;
    let mut guids: Vec<GUID> = Vec::new();
    guids.push(GUID_NULL); // Placeholder for first result

    // Find out how many GUIDs are associated with "Ports". Initially we assume
    // that there is only 1. num_guids will tell us how many there actually are.
    let res = unsafe {
        SetupDiClassGuidsFromNameA(ports_class_name.as_ptr(),
                                   guids.as_mut_ptr(),
                                   guids.len() as DWORD,
                                   &mut num_guids)
    };
    if res == FALSE {
        return Err(Error::new(ErrorKind::Unknown,
                              "Unable to determine number of Ports GUIDs"));
    }
    if num_guids == 0 {
        // We got a successful result of no GUIDs, so pop the placeholder that
        // we created before.
        guids.pop();
    }

    if num_guids as usize > guids.len() {
        // It turns out we needed more that one slot. num_guids will contain the number of slots
        // that we actually need, so go ahead and expand the vector to the correct size.
        while guids.len() < num_guids as usize {
            guids.push(GUID_NULL);
        }
        let res = unsafe {
            SetupDiClassGuidsFromNameA(ports_class_name.as_ptr(),
                                       guids.as_mut_ptr(),
                                       guids.len() as DWORD,
                                       &mut num_guids)
        };
        if res == FALSE {
            return Err(Error::new(ErrorKind::Unknown, "Unable to retrieve Ports GUIDs"));
        }
    }
    Ok(guids)
}

struct PortDevices {
    /// Handle to a device information set.
    hdi: HDEVINFO,

    /// Index used by iterator.
    dev_idx: DWORD,
}

impl PortDevices {
    // Creates PortDevices object which represents the set of devices associated with a particular
    // Ports class (given by `guid`).
    pub fn new(guid: &GUID) -> Self {
        PortDevices {
            hdi: unsafe { SetupDiGetClassDevsA(guid, ptr::null(), ptr::null_mut(), DIGCF_PRESENT) },
            dev_idx: 0,
        }
    }
}

impl Iterator for PortDevices {
    type Item = PortDevice;

    /// Iterator which returns a PortDevice from the set of PortDevices associated with a
    /// particular PortDevices class (guid).
    fn next(&mut self) -> Option<PortDevice> {
        let mut port_dev = PortDevice {
            hdi: self.hdi,
            devinfo_data: SP_DEVINFO_DATA {
                cbSize: mem::size_of::<SP_DEVINFO_DATA>() as DWORD,
                ClassGuid: GUID_NULL,
                DevInst: 0,
                Reserved: 0,
            },
        };
        let res =
            unsafe { SetupDiEnumDeviceInfo(self.hdi, self.dev_idx, &mut port_dev.devinfo_data) };
        if res == FALSE {
            None
        } else {
            self.dev_idx += 1;
            Some(port_dev)
        }
    }
}

impl Drop for PortDevices {
    fn drop(&mut self) {
        // Release the PortDevices object allocated in the constructor.
        unsafe {
            SetupDiDestroyDeviceInfoList(self.hdi);
        }
    }
}

struct PortDevice {
    /// Handle to a device information set.
    hdi: HDEVINFO,

    /// Information associated with this device.
    pub devinfo_data: SP_DEVINFO_DATA,
}

impl PortDevice {
    // Retrieves the device instance id string associated with this device. Some examples of
    // instance id strings are:
    //  MicroPython Board:  USB\VID_F055&PID_9802\385435603432
    //  FTDI USB Adapter:   FTDIBUS\VID_0403+PID_6001+A702TB52A\0000
    //  Black Magic Probe (Composite device with 2 UARTS):
    //      GDB Port:       USB\VID_1D50&PID_6018&MI_00\6&A694CA9&0&0000
    //      UART Port:      USB\VID_1D50&PID_6018&MI_02\6&A694CA9&0&0002
    fn instance_id(&mut self) -> Option<String> {
        let mut result_buf = [0i8; MAX_PATH];
        let res = unsafe {
            SetupDiGetDeviceInstanceIdA(self.hdi,
                                        &mut self.devinfo_data,
                                        result_buf.as_mut_ptr(),
                                        (result_buf.len() - 1) as DWORD,
                                        ptr::null_mut())
        };
        if res == FALSE {
            // Try to retrieve hardware id property.
            self.property(SPDRP_HARDWAREID)
        } else {
            let end_of_buffer = result_buf.len() - 1;
            result_buf[end_of_buffer] = 0;
            Some(unsafe { CStr::from_ptr(result_buf.as_ptr()).to_string_lossy().into_owned() })
        }
    }

    // Retrieves the port name (i.e. COM6) associated with this device.
    pub fn port_name(&mut self) -> String {
        let hkey = unsafe {
            SetupDiOpenDevRegKey(self.hdi,
                                 &mut self.devinfo_data,
                                 DICS_FLAG_GLOBAL,
                                 0,
                                 DIREG_DEV,
                                 KEY_READ)
        };
        let mut port_name_buffer = [0u8; MAX_PATH];
        let mut port_name_len = port_name_buffer.len() as DWORD;
        unsafe {
            RegQueryValueExA(hkey,
                             CString::new("PortName").unwrap().as_ptr(),
                             ptr::null_mut(),
                             ptr::null_mut(),
                             port_name_buffer.as_mut_ptr(),
                             &mut port_name_len)
        };
        unsafe { RegCloseKey(hkey) };
        String::from_utf8_lossy(&port_name_buffer[0..port_name_len as usize]).into_owned()
    }

    // Determines the port_type for this device, and if it's a USB port populate the various fields.
    pub fn port_type(&mut self) -> ::SerialPortType {
        if let Some(hardware_id) = self.instance_id() {
            // Some examples of what the hardware_id looks like:
            //  MicroPython pyboard:    USB\VID_F055&PID_9802\385435603432
            //  BlackMagic GDB Server:  USB\VID_1D50&PID_6018&MI_00\6&A694CA9&0&0000
            //  BlackMagic UART port:   USB\VID_1D50&PID_6018&MI_02\6&A694CA9&0&0002
            //  FTDI Serial Adapter:    FTDIBUS\VID_0403+PID_6001+A702TB52A\0000

            let re = Regex::new(concat!(r"VID_(?P<vid>[[:xdigit:]]{4})",
                                        r"[&+]PID_(?P<pid>[[:xdigit:]]{4})",
                                        r"([\\+](?P<serial>\w+))?"))
                    .unwrap();
            if let Some(caps) = re.captures(&hardware_id) {
                if let Ok(vid) = u16::from_str_radix(&caps[1], 16) {
                    if let Ok(pid) = u16::from_str_radix(&caps[2], 16) {
                        return ::SerialPortType::UsbPort(::UsbPortInfo {
                            vid: vid,
                            pid: pid,
                            serial_number: caps.get(4).map(|m| m.as_str().to_string()),
                            manufacturer: self.property(SPDRP_MFG),
                            product: self.property(SPDRP_FRIENDLYNAME),
                        });
                    }
                }
            }
        }
        ::SerialPortType::Unknown
    }

    // Retrieves a device property and returns it, if it exists. Returns None if the property
    // doesn't exist.
    fn property(&mut self, property_id: DWORD) -> Option<String> {
        let mut result_buf: [CHAR; MAX_PATH] = [0; MAX_PATH];
        let res = unsafe {
            SetupDiGetDeviceRegistryPropertyA(self.hdi,
                                              &mut self.devinfo_data,
                                              property_id,
                                              ptr::null_mut(),
                                              result_buf.as_mut_ptr() as PBYTE,
                                              (result_buf.len() - 1) as DWORD,
                                              ptr::null_mut())
        };
        if res == FALSE {
            if unsafe { GetLastError() } != ERROR_INSUFFICIENT_BUFFER {
                return None;
            }
        }
        let end_of_buffer = result_buf.len() - 1;
        result_buf[end_of_buffer] = 0;
        Some(unsafe { CStr::from_ptr(result_buf.as_ptr()).to_string_lossy().into_owned() })
    }
}

/// List available serial ports on the system.
pub fn available_ports() -> ::Result<Vec<SerialPortInfo>> {
    let mut ports = Vec::new();
    for guid in get_ports_guids()? {
        let port_devices = PortDevices::new(&guid);
        for mut port_device in port_devices {
            let port_name = port_device.port_name();

            // This technique also returns parallel ports, so we filter these out.
            if port_name.starts_with("LPT") {
                continue;
            }

            ports.push(::SerialPortInfo {
                           port_name: port_name,
                           port_type: port_device.port_type(),
                       });
        }
    }
    Ok(ports)
}

/// Return a list of offically-supported baud rates. It is likely that the hardware supports
/// more baud rates than this (many support arbitrary baud rates).
pub fn available_baud_rates() -> Vec<u32> {
    vec![110, 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 38400, 57600, 115200, 128000, 256000]
}
