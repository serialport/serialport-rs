use std::ffi::{CStr, CString};
use std::{mem, ptr};

use regex::Regex;
use winapi::shared::guiddef::*;
use winapi::shared::minwindef::*;
use winapi::shared::ntdef::CHAR;
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
        let mut port_name_buffer = [0u8; MAX_PATH];
        let mut port_name_len = port_name_buffer.len() as DWORD;
        let value_name = CString::new("PortName").unwrap();
        unsafe {
            RegQueryValueExA(
                hkey,
                value_name.as_ptr(),
                ptr::null_mut(),
                ptr::null_mut(),
                port_name_buffer.as_mut_ptr(),
                &mut port_name_len,
            )
        };
        unsafe { RegCloseKey(hkey) };

        let mut port_name = &port_name_buffer[0..port_name_len as usize];

        // Strip any nul bytes from the end of the buffer
        while port_name.last().map_or(false, |c| *c == b'\0') {
            port_name = &port_name[..port_name.len() - 1];
        }

        String::from_utf8_lossy(port_name).into_owned()
    }

    // Determines the port_type for this device, and if it's a USB port populate the various fields.
    pub fn port_type(&mut self) -> SerialPortType {
        if let Some(hardware_id) = self.instance_id() {
            // Some examples of what the hardware_id looks like:
            //  MicroPython pyboard:    USB\VID_F055&PID_9802\385435603432
            //  BlackMagic GDB Server:  USB\VID_1D50&PID_6018&MI_00\6&A694CA9&0&0000
            //  BlackMagic UART port:   USB\VID_1D50&PID_6018&MI_02\6&A694CA9&0&0002
            //  FTDI Serial Adapter:    FTDIBUS\VID_0403+PID_6001+A702TB52A\0000

            let re = Regex::new(concat!(
                r"VID_(?P<vid>[[:xdigit:]]{4})",
                r"[&+]PID_(?P<pid>[[:xdigit:]]{4})",
                r"([\\+](?P<serial>\w+))?"
            ))
            .unwrap();
            if let Some(caps) = re.captures(&hardware_id) {
                if let Ok(vid) = u16::from_str_radix(&caps[1], 16) {
                    if let Ok(pid) = u16::from_str_radix(&caps[2], 16) {
                        return SerialPortType::UsbPort(UsbPortInfo {
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
        SerialPortType::Unknown
    }

    // Retrieves a device property and returns it, if it exists. Returns None if the property
    // doesn't exist.
    fn property(&mut self, property_id: DWORD) -> Option<String> {
        let mut result_buf: [CHAR; MAX_PATH] = [0; MAX_PATH];
        let res = unsafe {
            SetupDiGetDeviceRegistryPropertyA(
                self.hdi,
                &mut self.devinfo_data,
                property_id,
                ptr::null_mut(),
                result_buf.as_mut_ptr() as PBYTE,
                (result_buf.len() - 1) as DWORD,
                ptr::null_mut(),
            )
        };
        if res == FALSE {
            if unsafe { GetLastError() } != ERROR_INSUFFICIENT_BUFFER {
                return None;
            }
        }
        let end_of_buffer = result_buf.len() - 1;
        result_buf[end_of_buffer] = 0;
        Some(unsafe {
            CStr::from_ptr(result_buf.as_ptr())
                .to_string_lossy()
                .into_owned()
        })
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
