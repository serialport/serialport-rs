use std::collections::HashSet;
use std::{mem, ptr};

use regex::Regex;
use winapi::shared::guiddef::*;
use winapi::shared::minwindef::*;
use winapi::shared::winerror::*;
use winapi::um::cfgmgr32::MAX_DEVICE_ID_LEN;
use winapi::um::cguid::GUID_NULL;
use winapi::um::errhandlingapi::GetLastError;
use winapi::um::setupapi::*;
use winapi::um::winnt::KEY_READ;
use winapi::um::winreg::*;

use crate::{Error, ErrorKind, Result, SerialPortInfo, SerialPortType, UsbPortInfo};

/// takes normal Rust `str` and outputs a null terminated UTF-16 encoded string
fn as_utf16(utf8: &str) -> Vec<u16> {
    utf8.encode_utf16().chain(Some(0)).collect()
}

/// takes a UTF-16 encoded slice (null termination not required)
/// and converts to a UTF8 Rust string. Trailing null chars are removed
fn from_utf16_lossy_trimmed(utf16: &[u16]) -> String {
    let num_chars = utf16
        .iter()
        .rev()
        .position(|&ch| ch != 0) // count the number of chars equal to `0`
        .map(|num_nulls| utf16.len() - num_nulls) // subtract that from the slice len
        .unwrap_or(0); // length is 0 if no non-`0` chars were found by position
    String::from_utf16_lossy(&utf16[..num_chars])
}

/// According to the MSDN docs, we should use SetupDiGetClassDevs, SetupDiEnumDeviceInfo
/// and SetupDiGetDeviceInstanceId in order to enumerate devices.
/// https://msdn.microsoft.com/en-us/windows/hardware/drivers/install/enumerating-installed-devices
fn get_ports_guids() -> Result<Vec<GUID>> {
    // SetupDiGetClassDevs returns the devices associated with a particular class of devices.
    // We want the list of devices which are listed as COM ports (generally those that show up in the
    // Device Manager as "Ports (COM & LPT)" which is otherwise known as the "Ports" class).
    //
    // The list of system defined classes can be found here:
    // https://learn.microsoft.com/en-us/windows-hardware/drivers/install/system-defined-device-setup-classes-available-to-vendors
    let class_names = ["Ports", "Modem"];
    let mut guids: Vec<GUID> = Vec::new();
    for class_name in class_names {
        let class_name_w = as_utf16(class_name);
        let mut num_guids: DWORD = 1; // Initially assume that there is only 1 guid per name.
        let class_start_idx = guids.len(); // start idx for this name (for potential resize with multiple guids)

        // first attempt with size == 1, second with the size returned from the first try
        for _ in 0..2 {
            guids.resize(class_start_idx + num_guids as usize, GUID_NULL);
            let guid_buffer = &mut guids[class_start_idx..];
            // Find out how many GUIDs are associated with this class name.  num_guids will tell us how many there actually are.
            let res = unsafe {
                SetupDiClassGuidsFromNameW(
                    class_name_w.as_ptr(),
                    guid_buffer.as_mut_ptr(),
                    guid_buffer.len() as DWORD,
                    &mut num_guids,
                )
            };
            if res == FALSE {
                return Err(Error::new(
                    ErrorKind::Unknown,
                    "Unable to determine number of Ports GUIDs",
                ));
            }
            let len_cmp = guid_buffer.len().cmp(&(num_guids as usize));
            // under allocated
            if len_cmp == std::cmp::Ordering::Less {
                continue; // retry
            }
            // allocation > required len
            else if len_cmp == std::cmp::Ordering::Greater {
                guids.truncate(class_start_idx + num_guids as usize);
            }
            break; // next name
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
            hdi: unsafe { SetupDiGetClassDevsW(guid, ptr::null(), ptr::null_mut(), DIGCF_PRESENT) },
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
    /// Retrieves the device instance id string associated with this device. Some examples of
    /// instance id strings are:
    ///
    /// * MicroPython Board:  USB\VID_F055&PID_9802\385435603432
    /// * FTDI USB Adapter:   FTDIBUS\VID_0403+PID_6001+A702TB52A\0000
    /// * Black Magic Probe (Composite device with 2 UARTS):
    ///   * GDB Port:       USB\VID_1D50&PID_6018&MI_00\6&A694CA9&0&0000
    ///   * UART Port:      USB\VID_1D50&PID_6018&MI_02\6&A694CA9&0&0002
    ///
    /// Reference: https://learn.microsoft.com/en-us/windows-hardware/drivers/install/device-instance-ids
    fn instance_id(&mut self) -> Option<String> {
        let mut result_buf = [0u16; MAX_DEVICE_ID_LEN];
        let working_buffer_len = result_buf.len() - 1; // always null terminated
        let mut desired_result_len = 0; // possibly larger than the buffer
        let res = unsafe {
            SetupDiGetDeviceInstanceIdW(
                self.hdi,
                &mut self.devinfo_data,
                result_buf.as_mut_ptr(),
                working_buffer_len as DWORD,
                &mut desired_result_len,
            )
        };
        if res == FALSE {
            // Try to retrieve hardware id property.
            self.property(SPDRP_HARDWAREID)
        } else {
            let actual_result_len = working_buffer_len.min(desired_result_len as usize);
            Some(from_utf16_lossy_trimmed(&result_buf[..actual_result_len]))
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
        let value_name = as_utf16("PortName");

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

        from_utf16_lossy_trimmed(port_name)
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
        let mut property_buf = vec![0u16; MAX_PATH];
        let mut desired_len = 0;

        for _ in 0..2 {
            let res = unsafe {
                SetupDiGetDeviceRegistryPropertyW(
                    self.hdi,
                    &mut self.devinfo_data,
                    property_id,
                    ptr::null_mut(),
                    property_buf.as_mut_ptr() as PBYTE,
                    property_buf.len() as DWORD,
                    &mut desired_len,
                )
            };

            let retry = (res == FALSE && unsafe { GetLastError() } != ERROR_INSUFFICIENT_BUFFER)
                || desired_len as usize > property_buf.len();
            if !retry {
                break;
            }
        }

        let actual_len = property_buf.len().min(desired_len as usize);
        // Using the unicode version of 'SetupDiGetDeviceRegistryProperty' seems to report the
        // entire mfg registry string. This may include some driver information that we should discard.
        // Example string: 'FTDI5.inf,%ftdi%;FTDI'
        from_utf16_lossy_trimmed(&property_buf[..actual_len])
            .split(';')
            .last()
            .map(str::to_string)
    }
}

/// Not all COM ports are listed under the "Ports" device class
/// The full list of COM ports is available from the registry at
/// HKEY_LOCAL_MACHINE\HARDWARE\DEVICEMAP\SERIALCOMM
///
/// port of https://learn.microsoft.com/en-us/windows/win32/sysinfo/enumerating-registry-subkeys
fn get_registry_com_ports() -> HashSet<String> {
    let mut ports_list = HashSet::new();

    let reg_key = as_utf16("HARDWARE\\DEVICEMAP\\SERIALCOMM");
    let mut ports_key = std::ptr::null_mut();

    // SAFETY: ffi, all inputs are correct
    let open_res = unsafe {
        RegOpenKeyExW(
            HKEY_LOCAL_MACHINE,
            reg_key.as_ptr(),
            0,
            KEY_READ,
            &mut ports_key,
        )
    };
    if SUCCEEDED(open_res) {
        let mut num_key_values = 0;

        // SAFETY: ffi, all inputs are correct
        let query_res = unsafe {
            RegQueryInfoKeyW(
                ports_key,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                &mut num_key_values,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut(),
            )
        };
        if SUCCEEDED(query_res) {
            for idx in 0..num_key_values {
                let mut val_name_buff = [0u16; MAX_PATH];
                let mut val_name_size = MAX_PATH as u32;
                let mut value_type = 0;
                // if 100 chars is not enough for COM<number> something is very wrong
                let mut val_data = [0u16; 100];
                let byte_len = 2 * val_data.len() as u32; // len doubled
                let mut data_size = byte_len;
                // SAFETY: ffi, all inputs are correct
                let res = unsafe {
                    RegEnumValueW(
                        ports_key,
                        idx,
                        val_name_buff.as_mut_ptr(),
                        &mut val_name_size,
                        std::ptr::null_mut(),
                        &mut value_type,
                        val_data.as_mut_ptr() as *mut u8,
                        &mut data_size,
                    )
                };
                if FAILED(res) || val_data.len() < data_size as usize {
                    break;
                }
                // SAFETY: data_size is checked and pointer is valid
                let val_data = from_utf16_lossy_trimmed(unsafe {
                    let utf16_len = data_size / 2;
                    std::slice::from_raw_parts(val_data.as_ptr(), utf16_len as usize)
                });
                ports_list.insert(val_data);
            }
        }
        // SAFETY: ffi, all inputs are correct
        unsafe { RegCloseKey(ports_key) };
    }
    ports_list
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
            if !port_name.starts_with("COM") {
                continue;
            }

            ports.push(SerialPortInfo {
                port_name,
                port_type: port_device.port_type(),
            });
        }
    }
    // ports identified through the registry have no additional information
    let mut raw_ports_set = get_registry_com_ports();
    if raw_ports_set.len() > ports.len() {
        // remove any duplicates. HashSet makes this relatively cheap
        for port in ports.iter() {
            raw_ports_set.remove(&port.port_name);
        }
        // add remaining ports as "unknown" type
        for raw_port in raw_ports_set {
            ports.push(SerialPortInfo {
                port_name: raw_port,
                port_type: SerialPortType::Unknown,
            })
        }
    }
    Ok(ports)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parsing_usb_port_information() {
        let bm_uart_hwid = r"USB\VID_1D50&PID_6018&MI_02\6&A694CA9&0&0000";
        let info = parse_usb_port_info(bm_uart_hwid).unwrap();

        assert_eq!(info.vid, 0x1D50);
        assert_eq!(info.pid, 0x6018);
        // FIXME: The 'serial number' as reported by the HWID likely needs some review
        assert_eq!(info.serial_number, Some("6".to_string()));
        assert_eq!(info.interface, Some(2));

        let ftdi_serial_hwid = r"FTDIBUS\VID_0403+PID_6001+A702TB52A\0000";
        let info = parse_usb_port_info(ftdi_serial_hwid).unwrap();

        assert_eq!(info.vid, 0x0403);
        assert_eq!(info.pid, 0x6001);
        assert_eq!(info.serial_number, Some("A702TB52A".to_string()));
        assert_eq!(info.interface, None);

        let pyboard_hwid = r"USB\VID_F055&PID_9802\385435603432";
        let info = parse_usb_port_info(pyboard_hwid).unwrap();

        assert_eq!(info.vid, 0xF055);
        assert_eq!(info.pid, 0x9802);
        assert_eq!(info.serial_number, Some("385435603432".to_string()));
        assert_eq!(info.interface, None);
    }

    #[test]
    fn encoding_trimming_utf16() {
        let test_str = "Testing";
        let wtest_str: Vec<u16> = as_utf16(test_str);
        let wtest_str_trailing = wtest_str
            .iter()
            .copied()
            .chain([0, 0, 0, 0]) // add some null chars
            .collect::<Vec<_>>();
        let and_back = from_utf16_lossy_trimmed(&wtest_str_trailing);

        assert_eq!(test_str, and_back);
    }
}
