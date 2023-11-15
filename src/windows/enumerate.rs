use std::ffi::{CStr, CString};
use std::{mem, ptr};

use regex::Regex;
use winapi::shared::guiddef::*;
use winapi::shared::minwindef::*;
use winapi::shared::winerror::*;
use winapi::um::cguid::GUID_NULL;
use winapi::um::errhandlingapi::GetLastError;
use winapi::um::setupapi::*;
use winapi::um::winnt::KEY_READ;
use winapi::um::winreg::*;

use crate::{Error, ErrorKind, Result, SerialPortInfo, SerialPortType, UsbPortInfo};

// According to the MSDN docs, we should use SetupDiGetClassDevs, SetupDiEnumDeviceInfo
// and SetupDiGetDeviceInstanceId in order to enumerate devices.
// https://msdn.microsoft.com/en-us/windows/hardware/drivers/install/enumerating-installed-devices
//
// SetupDiGetClassDevs returns the devices associated with a particular class of devices.
// We want the list of devices which shows up in the Device Manager as "Ports (COM & LPT)"
// which is otherwise known as the "Ports" class.
//
// get_pots_guids returns all of the classes (guids) associated with the name "Ports".
fn get_ports_guids() -> Result<Vec<GUID>> {
    // Note; unwrap can't fail, since "Ports" is valid UTF-8.
    let ports_class_name = CString::new("Ports").unwrap();

    // Size vector to hold 1 result (which is the most common result).
    let mut num_guids: DWORD = 0;
    let mut guids: Vec<GUID> = Vec::new();
    guids.push(GUID_NULL); // Placeholder for first result

    // Find out how many GUIDs are associated with "Ports". Initially we assume
    // that there is only 1. num_guids will tell us how many there actually are.
    let res = unsafe {
        SetupDiClassGuidsFromNameA(
            ports_class_name.as_ptr(),
            guids.as_mut_ptr(),
            guids.len() as DWORD,
            &mut num_guids,
        )
    };
    if res == FALSE {
        return Err(Error::new(
            ErrorKind::Unknown,
            "Unable to determine number of Ports GUIDs",
        ));
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
            SetupDiClassGuidsFromNameA(
                ports_class_name.as_ptr(),
                guids.as_mut_ptr(),
                guids.len() as DWORD,
                &mut num_guids,
            )
        };
        if res == FALSE {
            return Err(Error::new(
                ErrorKind::Unknown,
                "Unable to retrieve Ports GUIDs",
            ));
        }
    }
    Ok(guids)
}

/// Windows usb port information can be determined by the port's HWID string.
///
/// This function parses the HWID string using regex, and returns the USB port
/// information if the hardware ID can be parsed correctly. The manufacturer
/// and product names cannot be determined from the HWID string, so those are
/// set as None.
///
/// Some HWID examples are:
///   - MicroPython pyboard:    USB\VID_F055&PID_9802\385435603432
///   - BlackMagic GDB Server:  USB\VID_1D50&PID_6018&MI_00\6&A694CA9&0&0000
///   - BlackMagic UART port:   USB\VID_1D50&PID_6018&MI_02\6&A694CA9&0&0002
///   - FTDI Serial Adapter:    FTDIBUS\VID_0403+PID_6001+A702TB52A\0000
fn parse_usb_port_info(hardware_id: &str) -> Option<UsbPortInfo> {
    let re = Regex::new(concat!(
        r"VID_(?P<vid>[[:xdigit:]]{4})",
        r"[&+]PID_(?P<pid>[[:xdigit:]]{4})",
        r"(?:[&+]MI_(?P<iid>[[:xdigit:]]{2})){0,1}",
        r"([\\+](?P<serial>\w+))?"
    ))
    .unwrap();

    let caps = re.captures(hardware_id)?;

    Some(UsbPortInfo {
        vid: u16::from_str_radix(&caps[1], 16).ok()?,
        pid: u16::from_str_radix(&caps[2], 16).ok()?,
        serial_number: caps.name("serial").map(|m| m.as_str().to_string()),
        manufacturer: None,
        product: None,
        #[cfg(feature = "usbportinfo-interface")]
        interface: caps
            .name("iid")
            .and_then(|m| u8::from_str_radix(m.as_str(), 16).ok()),
    })
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
            SetupDiGetDeviceInstanceIdA(
                self.hdi,
                &mut self.devinfo_data,
                result_buf.as_mut_ptr(),
                (result_buf.len() - 1) as DWORD,
                ptr::null_mut(),
            )
        };
        if res == FALSE {
            // Try to retrieve hardware id property.
            self.property(SPDRP_HARDWAREID)
        } else {
            let end_of_buffer = result_buf.len() - 1;
            result_buf[end_of_buffer] = 0;
            Some(unsafe {
                CStr::from_ptr(result_buf.as_ptr())
                    .to_string_lossy()
                    .into_owned()
            })
        }
    }

    // Retrieves the port name (i.e. COM6) associated with this device.
    pub fn name(&mut self) -> String {
        let hkey = unsafe {
            SetupDiOpenDevRegKey(
                self.hdi,
                &mut self.devinfo_data,
                DICS_FLAG_GLOBAL,
                0,
                DIREG_DEV,
                KEY_READ,
            )
        };

        let mut port_name_buffer = [0u16; MAX_PATH];
        let mut port_name_len = port_name_buffer.len() as DWORD;
        let value_name: Vec<u16> = "PortName".encode_utf16().chain(Some(0)).collect();

        unsafe {
            RegQueryValueExW(
                hkey,
                value_name.as_ptr(),
                ptr::null_mut(),
                ptr::null_mut(),
                port_name_buffer.as_mut_ptr() as *mut u8,
                &mut port_name_len,
            )
        };
        unsafe { RegCloseKey(hkey) };

        let port_name = &port_name_buffer[0..port_name_len as usize];

        String::from_utf16_lossy(port_name)
            .trim_end_matches(0 as char)
            .to_string()
    }

    // Determines the port_type for this device, and if it's a USB port populate the various fields.
    pub fn port_type(&mut self) -> SerialPortType {
        self.instance_id()
            .and_then(|s| parse_usb_port_info(&s))
            .map(|mut info| {
                info.manufacturer = self.property(SPDRP_MFG);
                info.product = self.property(SPDRP_FRIENDLYNAME);
                SerialPortType::UsbPort(info)
            })
            .unwrap_or(SerialPortType::Unknown)
    }

    // Retrieves a device property and returns it, if it exists. Returns None if the property
    // doesn't exist.
    fn property(&mut self, property_id: DWORD) -> Option<String> {
        let mut property_buf = [0u16; MAX_PATH];

        let res = unsafe {
            SetupDiGetDeviceRegistryPropertyW(
                self.hdi,
                &mut self.devinfo_data,
                property_id,
                ptr::null_mut(),
                property_buf.as_mut_ptr() as PBYTE,
                property_buf.len() as DWORD,
                ptr::null_mut(),
            )
        };

        if res == FALSE {
            if unsafe { GetLastError() } != ERROR_INSUFFICIENT_BUFFER {
                return None;
            }
        }

        // Using the unicode version of 'SetupDiGetDeviceRegistryProperty' seems to report the
        // entire mfg registry string. This typically includes some driver information that we should discard.
        // Example string: 'FTDI5.inf,%ftdi%;FTDI'
        String::from_utf16_lossy(&property_buf)
            .trim_end_matches(0 as char)
            .split(';')
            .last()
            .map(str::to_string)
    }
}

/// List available serial ports on the system.
pub fn available_ports() -> Result<Vec<SerialPortInfo>> {
    let mut ports = Vec::new();
    for guid in get_ports_guids()? {
        let port_devices = PortDevices::new(&guid);
        for mut port_device in port_devices {
            let port_name = port_device.name();

            debug_assert!(
                port_name.as_bytes().last().map_or(true, |c| *c != b'\0'),
                "port_name has a trailing nul: {:?}",
                port_name
            );

            // This technique also returns parallel ports, so we filter these out.
            if port_name.starts_with("LPT") {
                continue;
            }

            ports.push(SerialPortInfo {
                port_name: port_name,
                port_type: port_device.port_type(),
            });
        }
    }
    Ok(ports)
}

#[test]
fn test_parsing_usb_port_information() {
    let bm_uart_hwid = r"USB\VID_1D50&PID_6018&MI_02\6&A694CA9&0&0000";
    let info = parse_usb_port_info(bm_uart_hwid).unwrap();

    assert_eq!(info.vid, 0x1D50);
    assert_eq!(info.pid, 0x6018);
    // FIXME: The 'serial number' as reported by the HWID likely needs some review
    assert_eq!(info.serial_number, Some("6".to_string()));
    #[cfg(feature = "usbportinfo-interface")]
    assert_eq!(info.interface, Some(2));

    let ftdi_serial_hwid = r"FTDIBUS\VID_0403+PID_6001+A702TB52A\0000";
    let info = parse_usb_port_info(ftdi_serial_hwid).unwrap();

    assert_eq!(info.vid, 0x0403);
    assert_eq!(info.pid, 0x6001);
    assert_eq!(info.serial_number, Some("A702TB52A".to_string()));
    #[cfg(feature = "usbportinfo-interface")]
    assert_eq!(info.interface, None);

    let pyboard_hwid = r"USB\VID_F055&PID_9802\385435603432";
    let info = parse_usb_port_info(pyboard_hwid).unwrap();

    assert_eq!(info.vid, 0xF055);
    assert_eq!(info.pid, 0x9802);
    assert_eq!(info.serial_number, Some("385435603432".to_string()));
    #[cfg(feature = "usbportinfo-interface")]
    assert_eq!(info.interface, None);
}
