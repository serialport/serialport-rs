#[cfg(any(target_os = "ios", target_os = "macos"))]
use nix::libc::{c_char, c_void};
#[cfg(all(target_os = "linux", not(target_env = "musl"), feature = "libudev"))]
use std::ffi::OsStr;
#[cfg(any(target_os = "ios", target_os = "macos"))]
use std::ffi::{CStr, CString};
#[cfg(any(target_os = "ios", target_os = "macos"))]
use std::mem::MaybeUninit;

use cfg_if::cfg_if;
#[cfg(any(target_os = "ios", target_os = "macos"))]
use CoreFoundation_sys::*;
#[cfg(any(target_os = "ios", target_os = "macos"))]
use IOKit_sys::*;

#[cfg(any(
    target_os = "freebsd",
    target_os = "ios",
    target_os = "linux",
    target_os = "macos"
))]
use crate::SerialPortType;
#[cfg(any(
    target_os = "ios",
    all(target_os = "linux", not(target_env = "musl"), feature = "libudev"),
    target_os = "macos"
))]
use crate::UsbPortInfo;
#[cfg(any(
    target_os = "android",
    target_os = "ios",
    all(target_os = "linux", not(target_env = "musl"), feature = "libudev"),
    target_os = "macos",
    target_os = "netbsd"
))]
use crate::{Error, ErrorKind};
use crate::{Result, SerialPortInfo};

/// Retrieves the udev property value named by `key`. If the value exists, then it will be
/// converted to a String, otherwise None will be returned.
#[cfg(all(target_os = "linux", not(target_env = "musl"), feature = "libudev"))]
fn udev_property_as_string(d: &libudev::Device, key: &str) -> Option<String> {
    d.property_value(key)
        .and_then(OsStr::to_str)
        .map(|s| s.to_string())
}

/// Retrieves the udev property value named by `key`. This function assumes that the retrieved
/// string is comprised of hex digits and the integer value of this will be returned as  a u16.
/// If the property value doesn't exist or doesn't contain valid hex digits, then an error
/// will be returned.
#[cfg(all(target_os = "linux", not(target_env = "musl"), feature = "libudev"))]
fn udev_hex_property_as_u16(d: &libudev::Device, key: &str) -> Result<u16> {
    if let Some(hex_str) = d.property_value(key).and_then(OsStr::to_str) {
        if let Ok(num) = u16::from_str_radix(hex_str, 16) {
            Ok(num)
        } else {
            Err(Error::new(ErrorKind::Unknown, "value not hex string"))
        }
    } else {
        Err(Error::new(ErrorKind::Unknown, "key not found"))
    }
}

#[cfg(all(target_os = "linux", not(target_env = "musl"), feature = "libudev"))]
fn port_type(d: &libudev::Device) -> Result<SerialPortType> {
    match d.property_value("ID_BUS").and_then(OsStr::to_str) {
        Some("usb") => {
            let serial_number = udev_property_as_string(d, "ID_SERIAL_SHORT");
            Ok(SerialPortType::UsbPort(UsbPortInfo {
                vid: udev_hex_property_as_u16(d, "ID_VENDOR_ID")?,
                pid: udev_hex_property_as_u16(d, "ID_MODEL_ID")?,
                serial_number,
                manufacturer: udev_property_as_string(d, "ID_VENDOR_FROM_DATABASE")
                    .or_else(|| udev_property_as_string(d, "ID_VENDOR")),
                product: udev_property_as_string(d, "ID_MODEL_FROM_DATABASE")
                    .or_else(|| udev_property_as_string(d, "ID_MODEL")),
            }))
        }
        Some("pci") => Ok(SerialPortType::PciPort),
        _ => Ok(SerialPortType::Unknown),
    }
}

#[cfg(any(target_os = "ios", target_os = "macos"))]
fn get_parent_device_by_type(
    device: io_object_t,
    parent_type: *const c_char,
) -> Option<io_registry_entry_t> {
    let parent_type = unsafe { CStr::from_ptr(parent_type) };
    use mach::kern_return::KERN_SUCCESS;
    let mut device = device;
    loop {
        let mut class_name = MaybeUninit::<[c_char; 128]>::uninit();
        unsafe { IOObjectGetClass(device, class_name.as_mut_ptr() as *mut c_char) };
        let class_name = unsafe { class_name.assume_init() };
        let name = unsafe { CStr::from_ptr(&class_name[0]) };
        if name == parent_type {
            return Some(device);
        }
        let mut parent = MaybeUninit::uninit();
        if unsafe {
            IORegistryEntryGetParentEntry(device, kIOServiceClass(), parent.as_mut_ptr())
                != KERN_SUCCESS
        } {
            return None;
        }
        device = unsafe { parent.assume_init() };
    }
}

#[cfg(any(target_os = "ios", target_os = "macos"))]
#[allow(non_upper_case_globals)]
/// Returns a specific property of the given device as an integer.
fn get_int_property(
    device_type: io_registry_entry_t,
    property: &str,
    cf_number_type: CFNumberType,
) -> Option<u32> {
    unsafe {
        let prop_str = CString::new(property).unwrap();
        let key = CFStringCreateWithCString(
            kCFAllocatorDefault,
            prop_str.as_ptr(),
            kCFStringEncodingUTF8,
        );
        let container = IORegistryEntryCreateCFProperty(device_type, key, kCFAllocatorDefault, 0);
        if container.is_null() {
            return None;
        }
        let num = match cf_number_type {
            kCFNumberSInt16Type => {
                let mut num: u16 = 0;
                let num_ptr: *mut c_void = &mut num as *mut _ as *mut c_void;
                CFNumberGetValue(container as CFNumberRef, cf_number_type, num_ptr);
                Some(num as u32)
            }
            kCFNumberSInt32Type => {
                let mut num: u32 = 0;
                let num_ptr: *mut c_void = &mut num as *mut _ as *mut c_void;
                CFNumberGetValue(container as CFNumberRef, cf_number_type, num_ptr);
                Some(num)
            }
            _ => None,
        };
        CFRelease(container);

        num
    }
}

#[cfg(any(target_os = "ios", target_os = "macos"))]
/// Returns a specific property of the given device as a string.
fn get_string_property(device_type: io_registry_entry_t, property: &str) -> Option<String> {
    unsafe {
        let prop_str = CString::new(property).unwrap();
        let key = CFStringCreateWithCString(
            kCFAllocatorDefault,
            prop_str.as_ptr(),
            kCFStringEncodingUTF8,
        );
        let container = IORegistryEntryCreateCFProperty(device_type, key, kCFAllocatorDefault, 0);
        if container.is_null() {
            return None;
        }

        let str_ptr = CFStringGetCStringPtr(container as CFStringRef, kCFStringEncodingMacRoman);
        if str_ptr.is_null() {
            CFRelease(container);
            return None;
        }
        let opt_str = CStr::from_ptr(str_ptr).to_str().ok().map(String::from);

        CFRelease(container);

        opt_str
    }
}

#[cfg(any(target_os = "ios", target_os = "macos"))]
/// Determine the serial port type based on the service object (like that returned by
/// `IOIteratorNext`). Specific properties are extracted for USB devices.
fn port_type(service: io_object_t) -> SerialPortType {
    let bluetooth_device_class_name = b"IOBluetoothSerialClient\0".as_ptr() as *const c_char;
    let usb_device_class_name = b"IOUSBHostDevice\0".as_ptr() as *const c_char;
    let legacy_usb_device_class_name = kIOUSBDeviceClassName();

    let maybe_usb_device = get_parent_device_by_type(service, usb_device_class_name)
        .or_else(|| get_parent_device_by_type(service, legacy_usb_device_class_name));
    if let Some(usb_device) = maybe_usb_device {
        SerialPortType::UsbPort(UsbPortInfo {
            vid: get_int_property(usb_device, "idVendor", kCFNumberSInt16Type).unwrap_or_default()
                as u16,
            pid: get_int_property(usb_device, "idProduct", kCFNumberSInt16Type).unwrap_or_default()
                as u16,
            serial_number: get_string_property(usb_device, "USB Serial Number"),
            manufacturer: get_string_property(usb_device, "USB Vendor Name"),
            product: get_string_property(usb_device, "USB Product Name"),
        })
    } else if get_parent_device_by_type(service, bluetooth_device_class_name).is_some() {
        SerialPortType::BluetoothPort
    } else {
        SerialPortType::PciPort
    }
}

cfg_if! {
    if #[cfg(any(target_os = "ios", target_os = "macos"))] {
        /// Scans the system for serial ports and returns a list of them.
        /// The `SerialPortInfo` struct contains the name of the port which can be used for opening it.
        pub fn available_ports() -> Result<Vec<SerialPortInfo>> {
            use mach::kern_return::KERN_SUCCESS;
            use mach::port::{mach_port_t, MACH_PORT_NULL};

            let mut vec = Vec::new();
            unsafe {
                // Create a dictionary for specifying the search terms against the IOService
                let classes_to_match = IOServiceMatching(kIOSerialBSDServiceValue());
                if classes_to_match.is_null() {
                    return Err(Error::new(
                        ErrorKind::Unknown,
                        "IOServiceMatching returned a NULL dictionary.",
                    ));
                }

                // Populate the search dictionary with a single key/value pair indicating that we're
                // searching for serial devices matching the RS232 device type.
                let key = CFStringCreateWithCString(
                    kCFAllocatorDefault,
                    kIOSerialBSDTypeKey(),
                    kCFStringEncodingUTF8,
                );
                if key.is_null() {
                    return Err(Error::new(
                        ErrorKind::Unknown,
                        "Failed to allocate key string.",
                    ));
                }
                let value = CFStringCreateWithCString(
                    kCFAllocatorDefault,
                    kIOSerialBSDAllTypes(),
                    kCFStringEncodingUTF8,
                );
                if value.is_null() {
                    return Err(Error::new(
                        ErrorKind::Unknown,
                        "Failed to allocate value string.",
                    ));
                }
                CFDictionarySetValue(classes_to_match, key as CFTypeRef, value as CFTypeRef);

                // Get an interface to IOKit
                let mut master_port: mach_port_t = MACH_PORT_NULL;
                let mut kern_result = IOMasterPort(MACH_PORT_NULL, &mut master_port);
                if kern_result != KERN_SUCCESS {
                    return Err(Error::new(
                        ErrorKind::Unknown,
                        format!("ERROR: {}", kern_result),
                    ));
                }

                // Run the search.
                let mut matching_services = MaybeUninit::uninit();
                kern_result = IOServiceGetMatchingServices(
                    kIOMasterPortDefault,
                    classes_to_match,
                    matching_services.as_mut_ptr(),
                );
                if kern_result != KERN_SUCCESS {
                    return Err(Error::new(
                        ErrorKind::Unknown,
                        format!("ERROR: {}", kern_result),
                    ));
                }
                let matching_services = matching_services.assume_init();

                loop {
                    // Grab the next result.
                    let modem_service = IOIteratorNext(matching_services);

                    // Break out if we've reached the end of the iterator
                    if modem_service == MACH_PORT_NULL {
                        break;
                    }

                    // Fetch all properties of the current search result item.
                    let mut props = MaybeUninit::uninit();
                    let result = IORegistryEntryCreateCFProperties(
                        modem_service,
                        props.as_mut_ptr(),
                        kCFAllocatorDefault,
                        0,
                    );
                    if result == KERN_SUCCESS {
                        for key in ["IOCalloutDevice", "IODialinDevice"].iter() {
                            let key = CString::new(*key).unwrap();
                            let key_cfstring = CFStringCreateWithCString(
                                kCFAllocatorDefault,
                                key.as_ptr(),
                                kCFStringEncodingUTF8,
                            );
                            let value = CFDictionaryGetValue(props.assume_init(), key_cfstring as *const c_void);

                            let type_id = CFGetTypeID(value);
                            if type_id == CFStringGetTypeID() {
                                let mut buf = Vec::with_capacity(256);

                                CFStringGetCString(
                                    value as CFStringRef,
                                    buf.as_mut_ptr(),
                                    256,
                                    kCFStringEncodingUTF8,
                                );
                                let path = CStr::from_ptr(buf.as_ptr()).to_string_lossy();
                                vec.push(SerialPortInfo {
                                    port_name: path.to_string(),
                                    port_type: port_type(modem_service),
                                });
                            } else {
                                return Err(Error::new(
                                    ErrorKind::Unknown,
                                    "Found invalid type for TypeID",
                                ));
                            }
                        }
                    } else {
                        return Err(Error::new(ErrorKind::Unknown, format!("ERROR: {}", result)));
                    }

                    // Clean up after we're done processing htis result
                    IOObjectRelease(modem_service);
                }
            }
            Ok(vec)
        }
    } else if #[cfg(all(target_os = "linux", not(target_env = "musl"), feature = "libudev"))] {
        /// Scans the system for serial ports and returns a list of them.
        /// The `SerialPortInfo` struct contains the name of the port
        /// which can be used for opening it.
        pub fn available_ports() -> Result<Vec<SerialPortInfo>> {
            let mut vec = Vec::new();
            if let Ok(context) = libudev::Context::new() {
                let mut enumerator = libudev::Enumerator::new(&context)?;
                enumerator.match_subsystem("tty")?;
                let devices = enumerator.scan_devices()?;
                for d in devices {
                    if let Some(p) = d.parent() {
                        if let Some(devnode) = d.devnode() {
                            if let Some(path) = devnode.to_str() {
                                if let Some(driver) = p.driver() {
                                    if driver == "serial8250" && crate::new(path, 9600).open().is_err() {
                                        continue;
                                    }
                                }
                                // Stop bubbling up port_type errors here so problematic ports are just
                                // skipped instead of causing no ports to be returned.
                                if let Ok(pt) = port_type(&d) {
                                    vec.push(SerialPortInfo {
                                        port_name: String::from(path),
                                        port_type: pt,
                                    });
                                }
                            }
                        }
                    }
                }
            }
            Ok(vec)
        }
    } else if #[cfg(target_os = "linux")] {
        use std::fs::File;
        use std::io::Read;
        use std::path::Path;

        /// Enumerating serial ports on non-Linux POSIX platforms is disabled by disabled the "libudev"
        /// default feature.
        pub fn available_ports() -> Result<Vec<SerialPortInfo>> {
            let mut vec = Vec::new();
            let sys_path = Path::new("/sys/class/tty/");
            let mut s;
            for path in sys_path.read_dir().expect("/sys/class/tty/ doesn't exist on this system") {
                let raw_path = path?.path().clone();
                let mut path = raw_path.clone();

                path.push("device");
                if !path.is_dir() {
                    continue;
                }

                path.push("driver_override");
                if path.is_file() {
                    s = String::new();
                    File::open(path)?.read_to_string(&mut s)?;
                    if &s == "(null)\n" {
                        continue;
                    }
                }

                vec.push(SerialPortInfo {
                    port_name: raw_path.to_string_lossy().to_string(),
                    port_type: SerialPortType::Unknown,
                });
            }
            Ok(vec)
        }
    } else if #[cfg(target_os = "freebsd")] {
        use std::path::Path;

        /// Scans the system for serial ports and returns a list of them.
        /// The `SerialPortInfo` struct contains the name of the port
        /// which can be used for opening it.
        pub fn available_ports() -> Result<Vec<SerialPortInfo>> {
            let mut vec = Vec::new();
            let dev_path = Path::new("/dev/");
            for path in dev_path.read_dir()? {
                let path = path?;
                let filename = path.file_name();
                let filename_string = filename.to_string_lossy();
                if filename_string.starts_with("cuaU") || filename_string.starts_with("cuau") || filename_string.starts_with("cuad") {
                    if !filename_string.ends_with(".init") && !filename_string.ends_with(".lock") {
                        vec.push(SerialPortInfo {
                            port_name: path.path().to_string_lossy().to_string(),
                            port_type: SerialPortType::Unknown,
                        });
                    }
                }
            }
            Ok(vec)
        }
    } else {
        /// Enumerating serial ports on this platform is not supported
        pub fn available_ports() -> Result<Vec<SerialPortInfo>> {
            Err(Error::new(
                ErrorKind::Unknown,
                "Not implemented for this OS",
            ))
        }
    }
}
