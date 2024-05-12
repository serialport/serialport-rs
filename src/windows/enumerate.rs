use std::collections::HashSet;
use std::ffi::{CStr, CString};
use std::{mem, ptr};

use regex::Regex;
use winapi::shared::guiddef::*;
use winapi::shared::minwindef::*;
use winapi::shared::winerror::*;
use winapi::um::cfgmgr32::*;
use winapi::um::cguid::GUID_NULL;
use winapi::um::errhandlingapi::GetLastError;
use winapi::um::setupapi::*;
use winapi::um::winnt::KEY_READ;
use winapi::um::winreg::*;

use crate::{Error, ErrorKind, Result, SerialPortInfo, SerialPortType, UsbPortInfo};

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
    let class_names = [
        // Note; since names are valid UTF-8, unwrap can't fail
        CString::new("Ports").unwrap(),
        CString::new("Modem").unwrap(),
    ];
    let mut guids: Vec<GUID> = Vec::new();
    for class_name in class_names {
        let mut num_guids: DWORD = 1; // Initially assume that there is only 1 guid per name.
        let class_start_idx = guids.len(); // start idx for this name (for potential resize with multiple guids)

        // first attempt with size == 1, second with the size returned from the first try
        for _ in 0..2 {
            guids.resize(class_start_idx + num_guids as usize, GUID_NULL);
            let guid_buffer = &mut guids[class_start_idx..];
            // Find out how many GUIDs are associated with this class name.  num_guids will tell us how many there actually are.
            let res = unsafe {
                SetupDiClassGuidsFromNameA(
                    class_name.as_ptr(),
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
/// For composite USB devices, the HWID string will be for the interface. In
/// this case, the parent HWID string must be provided so that the correct
/// serial number can be determined.
///
/// Some HWID examples are:
///   - MicroPython pyboard:    USB\VID_F055&PID_9802\385435603432
///   - BlackMagic GDB Server:  USB\VID_1D50&PID_6018&MI_00\6&A694CA9&0&0000
///   - BlackMagic UART port:   USB\VID_1D50&PID_6018&MI_02\6&A694CA9&0&0002
///   - FTDI Serial Adapter:    FTDIBUS\VID_0403+PID_6001+A702TB52A\0000
fn parse_usb_port_info(hardware_id: &str, parent_hardware_id: Option<&str>) -> Option<UsbPortInfo> {
    let re = Regex::new(concat!(
        r"VID_(?P<vid>[[:xdigit:]]{4})",
        r"[&+]PID_(?P<pid>[[:xdigit:]]{4})",
        r"(?:[&+]MI_(?P<iid>[[:xdigit:]]{2})){0,1}",
        r"([\\+](?P<serial>\w+))?"
    ))
    .unwrap();

    let mut caps = re.captures(hardware_id)?;

    let interface = caps
        .name("iid")
        .and_then(|m| u8::from_str_radix(m.as_str(), 16).ok());

    if interface.is_some() {
        // If this is a composite device, we need to parse the parent's HWID to get the correct information.
        caps = re.captures(parent_hardware_id?)?;
    }

    Some(UsbPortInfo {
        vid: u16::from_str_radix(&caps[1], 16).ok()?,
        pid: u16::from_str_radix(&caps[2], 16).ok()?,
        serial_number: caps.name("serial").map(|m| {
            let m = m.as_str();
            if m.contains('&') {
                m.split('&').nth(1).unwrap().to_string()
            } else {
                m.to_string()
            }
        }),
        manufacturer: None,
        product: None,
        #[cfg(feature = "usbportinfo-interface")]
        interface: interface,
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
    // Retrieves the device instance id string associated with this device's parent.
    // This is useful for determining the serial number of a composite USB device.
    fn parent_instance_id(&mut self) -> Option<String> {
        let mut result_buf = [0i8; MAX_PATH];
        let mut parent_device_instance_id = 0;

        let res =
            unsafe { CM_Get_Parent(&mut parent_device_instance_id, self.devinfo_data.DevInst, 0) };
        if res == CR_SUCCESS {
            let res = unsafe {
                CM_Get_Device_IDA(
                    parent_device_instance_id,
                    result_buf.as_mut_ptr(),
                    (result_buf.len() - 1) as ULONG,
                    0,
                )
            };

            if res == CR_SUCCESS {
                let end_of_buffer = result_buf.len() - 1;
                result_buf[end_of_buffer] = 0;
                Some(unsafe {
                    CStr::from_ptr(result_buf.as_ptr())
                        .to_string_lossy()
                        .into_owned()
                })
            } else {
                None
            }
        } else {
            None
        }
    }

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

    /// Retrieves the problem status of this device. For example, `CM_PROB_DISABLED` indicates
    /// the device has been disabled in Device Manager.
    fn problem(&mut self) -> Option<ULONG> {
        let mut status = 0;
        let mut problem_number = 0;
        let res = unsafe {
            CM_Get_DevNode_Status(
                &mut status,
                &mut problem_number,
                self.devinfo_data.DevInst,
                0,
            )
        };
        if res == CR_SUCCESS {
            Some(problem_number)
        } else {
            None
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
            .map(|s| (s, self.parent_instance_id())) // Get parent instance id if it exists.
            .and_then(|(d, p)| parse_usb_port_info(&d, p.as_deref()))
            .map(|mut info: UsbPortInfo| {
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

        if res == FALSE && unsafe { GetLastError() } != ERROR_INSUFFICIENT_BUFFER {
            return None;
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

/// Not all COM ports are listed under the "Ports" device class
/// The full list of COM ports is available from the registry at
/// HKEY_LOCAL_MACHINE\HARDWARE\DEVICEMAP\SERIALCOMM
///
/// port of https://learn.microsoft.com/en-us/windows/win32/sysinfo/enumerating-registry-subkeys
fn get_registry_com_ports() -> HashSet<String> {
    let mut ports_list = HashSet::new();

    let reg_key = b"HARDWARE\\DEVICEMAP\\SERIALCOMM\0";
    let key_ptr = reg_key.as_ptr() as *const i8;
    let mut ports_key = std::ptr::null_mut();

    // SAFETY: ffi, all inputs are correct
    let open_res =
        unsafe { RegOpenKeyExA(HKEY_LOCAL_MACHINE, key_ptr, 0, KEY_READ, &mut ports_key) };
    if SUCCEEDED(open_res) {
        let mut class_name_buff = [0i8; MAX_PATH];
        let mut class_name_size = MAX_PATH as u32;
        let mut sub_key_count = 0;
        let mut largest_sub_key = 0;
        let mut largest_class_string = 0;
        let mut num_key_values = 0;
        let mut longest_value_name = 0;
        let mut longest_value_data = 0;
        let mut size_security_desc = 0;
        let mut last_write_time = FILETIME {
            dwLowDateTime: 0,
            dwHighDateTime: 0,
        };
        // SAFETY: ffi, all inputs are correct
        let query_res = unsafe {
            RegQueryInfoKeyA(
                ports_key,
                class_name_buff.as_mut_ptr(),
                &mut class_name_size,
                std::ptr::null_mut(),
                &mut sub_key_count,
                &mut largest_sub_key,
                &mut largest_class_string,
                &mut num_key_values,
                &mut longest_value_name,
                &mut longest_value_data,
                &mut size_security_desc,
                &mut last_write_time,
            )
        };
        if SUCCEEDED(query_res) {
            for idx in 0..num_key_values {
                let mut val_name_buff = [0i8; MAX_PATH];
                let mut val_name_size = MAX_PATH as u32;
                let mut value_type = 0;
                // if 100 chars is not enough for COM<number> something is very wrong
                let mut val_data = [0; 100];
                let mut data_size = val_data.len() as u32;
                // SAFETY: ffi, all inputs are correct
                let res = unsafe {
                    RegEnumValueA(
                        ports_key,
                        idx,
                        val_name_buff.as_mut_ptr(),
                        &mut val_name_size,
                        std::ptr::null_mut(),
                        &mut value_type,
                        val_data.as_mut_ptr(),
                        &mut data_size,
                    )
                };
                if FAILED(res) || val_data.len() < data_size as usize {
                    break;
                }
                // SAFETY: data_size is checked and pointer is valid
                let val_data = CStr::from_bytes_with_nul(unsafe {
                    std::slice::from_raw_parts(val_data.as_ptr(), data_size as usize)
                });

                if let Ok(port) = val_data {
                    ports_list.insert(port.to_string_lossy().into_owned());
                }
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
            // Ignore nonfunctional devices
            if port_device.problem() != Some(0) {
                continue;
            }

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

#[test]
fn test_parsing_usb_port_information() {
    let bm_uart_hwid = r"USB\VID_1D50&PID_6018&MI_02\6&A694CA9&0&0000";
    let bm_parent_hwid = r"USB\VID_1D50&PID_6018\85A12F01";
    let info = parse_usb_port_info(bm_uart_hwid, Some(bm_parent_hwid)).unwrap();

    assert_eq!(info.vid, 0x1D50);
    assert_eq!(info.pid, 0x6018);
    assert_eq!(info.serial_number, Some("85A12F01".to_string()));
    #[cfg(feature = "usbportinfo-interface")]
    assert_eq!(info.interface, Some(2));

    let ftdi_serial_hwid = r"FTDIBUS\VID_0403+PID_6001+A702TB52A\0000";
    let info = parse_usb_port_info(ftdi_serial_hwid, None).unwrap();

    assert_eq!(info.vid, 0x0403);
    assert_eq!(info.pid, 0x6001);
    assert_eq!(info.serial_number, Some("A702TB52A".to_string()));
    #[cfg(feature = "usbportinfo-interface")]
    assert_eq!(info.interface, None);

    let pyboard_hwid = r"USB\VID_F055&PID_9802\385435603432";
    let info = parse_usb_port_info(pyboard_hwid, None).unwrap();

    assert_eq!(info.vid, 0xF055);
    assert_eq!(info.pid, 0x9802);
    assert_eq!(info.serial_number, Some("385435603432".to_string()));
    #[cfg(feature = "usbportinfo-interface")]
    assert_eq!(info.interface, None);
}
