use cfg_if::cfg_if;

#[cfg(all(target_os = "linux", not(target_env = "musl"), feature = "libudev"))]
use std::ffi::OsStr;

cfg_if! {
    if #[cfg(any(target_os = "ios", target_os = "macos"))] {
        use core_foundation_sys::base::*;
        use core_foundation_sys::dictionary::*;
        use core_foundation_sys::number::*;
        use core_foundation_sys::string::*;
        use io_kit_sys::*;
        use io_kit_sys::keys::*;
        use io_kit_sys::serial::keys::*;
        use io_kit_sys::types::*;
        use io_kit_sys::usb::lib::*;
        use nix::libc::{c_char, c_void};
        use std::convert::TryInto;
        use std::ffi::{CStr, CString};
        use std::mem::MaybeUninit;
    }
}

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
    target_os = "netbsd",
    target_os = "openbsd",
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
/// This function uses a built-in type's `from_str_radix` to implementation to perform the
/// actual conversion.
#[cfg(all(target_os = "linux", not(target_env = "musl"), feature = "libudev"))]
fn udev_hex_property_as_int<T>(
    d: &libudev::Device,
    key: &str,
    from_str_radix: &dyn Fn(&str, u32) -> std::result::Result<T, std::num::ParseIntError>,
) -> Result<T> {
    if let Some(hex_str) = d.property_value(key).and_then(OsStr::to_str) {
        if let Ok(num) = from_str_radix(hex_str, 16) {
            Ok(num)
        } else {
            Err(Error::new(ErrorKind::Unknown, "value not hex string"))
        }
    } else {
        Err(Error::new(ErrorKind::Unknown, "key not found"))
    }
}

/// Looks up a property which is provided in two "flavors": Where special charaters and whitespaces
/// are encoded/escaped and where they are replaced (with underscores). This is for example done
/// by udev for manufacturer and model information.
///
/// See
/// https://github.com/systemd/systemd/blob/38c258398427d1f497268e615906759025e51ea6/src/udev/udev-builtin-usb_id.c#L432
/// for details.
#[cfg(all(target_os = "linux", not(target_env = "musl"), feature = "libudev"))]
fn udev_property_encoded_or_replaced_as_string(
    d: &libudev::Device,
    encoded_key: &str,
    replaced_key: &str,
) -> Option<String> {
    udev_property_as_string(d, encoded_key)
        .and_then(|s| unescaper::unescape(&s).ok())
        .or_else(|| udev_property_as_string(d, replaced_key))
        .map(udev_restore_spaces)
}

/// Converts the underscores from `udev_replace_whitespace` back to spaces quick and dirtily. We
/// are ignoring the different types of whitespaces and the substitutions from `udev_replace_chars`
/// deliberately for keeping a low profile.
///
/// See
/// https://github.com/systemd/systemd/blob/38c258398427d1f497268e615906759025e51ea6/src/shared/udev-util.c#L281
/// for more details.
#[cfg(all(target_os = "linux", not(target_env = "musl"), feature = "libudev"))]
fn udev_restore_spaces(source: String) -> String {
    source.replace('_', " ")
}

#[cfg(all(target_os = "linux", not(target_env = "musl"), feature = "libudev"))]
fn port_type(d: &libudev::Device) -> Result<SerialPortType> {
    match d.property_value("ID_BUS").and_then(OsStr::to_str) {
        Some("usb") => {
            let serial_number = udev_property_as_string(d, "ID_SERIAL_SHORT");
            // For devices on the USB, udev also provides manufacturer and product information from
            // its hardware dataase. Use this as a fallback if this information is not provided
            // from the device itself.
            let manufacturer =
                udev_property_encoded_or_replaced_as_string(d, "ID_VENDOR_ENC", "ID_VENDOR")
                    .or_else(|| udev_property_as_string(d, "ID_VENDOR_FROM_DATABASE"));
            let product =
                udev_property_encoded_or_replaced_as_string(d, "ID_MODEL_ENC", "ID_MODEL")
                    .or_else(|| udev_property_as_string(d, "ID_MODEL_FROM_DATABASE"));
            Ok(SerialPortType::UsbPort(UsbPortInfo {
                vid: udev_hex_property_as_int(d, "ID_VENDOR_ID", &u16::from_str_radix)?,
                pid: udev_hex_property_as_int(d, "ID_MODEL_ID", &u16::from_str_radix)?,
                serial_number,
                manufacturer,
                product,
                #[cfg(feature = "usbportinfo-interface")]
                interface: udev_hex_property_as_int(d, "ID_USB_INTERFACE_NUM", &u8::from_str_radix)
                    .ok(),
            }))
        }
        Some("pci") => {
            let usb_properties = vec![
                d.property_value("ID_USB_VENDOR_ID"),
                d.property_value("ID_USB_MODEL_ID"),
                d.property_value("ID_USB_VENDOR"),
                d.property_value("ID_USB_MODEL"),
                d.property_value("ID_USB_SERIAL_SHORT"),
            ]
            .into_iter()
            .collect::<Option<Vec<_>>>();
            if usb_properties.is_some() {
                // For USB devices reported at a PCI bus, there is apparently no fallback
                // information from udevs hardware database provided.
                let manufacturer = udev_property_encoded_or_replaced_as_string(
                    d,
                    "ID_USB_VENDOR_ENC",
                    "ID_USB_VENDOR",
                );
                let product = udev_property_encoded_or_replaced_as_string(
                    d,
                    "ID_USB_MODEL_ENC",
                    "ID_USB_MODEL",
                );
                Ok(SerialPortType::UsbPort(UsbPortInfo {
                    vid: udev_hex_property_as_int(d, "ID_USB_VENDOR_ID", &u16::from_str_radix)?,
                    pid: udev_hex_property_as_int(d, "ID_USB_MODEL_ID", &u16::from_str_radix)?,
                    serial_number: udev_property_as_string(d, "ID_USB_SERIAL_SHORT"),
                    manufacturer,
                    product,
                    #[cfg(feature = "usbportinfo-interface")]
                    interface: udev_hex_property_as_int(
                        d,
                        "ID_USB_INTERFACE_NUM",
                        &u8::from_str_radix,
                    )
                    .ok(),
                }))
            } else {
                Ok(SerialPortType::PciPort)
            }
        }
        _ => Ok(SerialPortType::Unknown),
    }
}

#[cfg(any(target_os = "ios", target_os = "macos"))]
fn get_parent_device_by_type(
    device: io_object_t,
    parent_type: *const c_char,
) -> Option<io_registry_entry_t> {
    let parent_type = unsafe { CStr::from_ptr(parent_type) };
    use mach2::kern_return::KERN_SUCCESS;
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
            IORegistryEntryGetParentEntry(device, kIOServiceClass, parent.as_mut_ptr())
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
        let _key_guard = scopeguard::guard((), |_| {
            CFRelease(key as *const c_void);
        });

        let container = IORegistryEntryCreateCFProperty(device_type, key, kCFAllocatorDefault, 0);
        if container.is_null() {
            return None;
        }
        let _container_guard = scopeguard::guard((), |_| {
            CFRelease(container);
        });

        match cf_number_type {
            kCFNumberSInt8Type => {
                let mut num: u8 = 0;
                let num_ptr: *mut c_void = &mut num as *mut _ as *mut c_void;
                CFNumberGetValue(container as CFNumberRef, cf_number_type, num_ptr);
                Some(num as u32)
            }
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
        }
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
        let _key_guard = scopeguard::guard((), |_| {
            CFRelease(key as *const c_void);
        });

        let container = IORegistryEntryCreateCFProperty(device_type, key, kCFAllocatorDefault, 0);
        if container.is_null() {
            return None;
        }
        let _container_guard = scopeguard::guard((), |_| {
            CFRelease(container);
        });

        let mut buf = Vec::with_capacity(256);
        let result = CFStringGetCString(
            container as CFStringRef,
            buf.as_mut_ptr(),
            buf.capacity().try_into().unwrap(),
            kCFStringEncodingUTF8,
        );
        let opt_str = if result != 0 {
            CStr::from_ptr(buf.as_ptr()).to_str().ok().map(String::from)
        } else {
            None
        };

        opt_str
    }
}

#[cfg(any(target_os = "ios", target_os = "macos"))]
/// Determine the serial port type based on the service object (like that returned by
/// `IOIteratorNext`). Specific properties are extracted for USB devices.
fn port_type(service: io_object_t) -> SerialPortType {
    let bluetooth_device_class_name = b"IOBluetoothSerialClient\0".as_ptr() as *const c_char;
    let usb_device_class_name = b"IOUSBHostDevice\0".as_ptr() as *const c_char;
    let legacy_usb_device_class_name = kIOUSBDeviceClassName;

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
            // Apple developer documentation indicates `bInterfaceNumber` is the supported key for
            // looking up the composite usb interface id. `idVendor` and `idProduct` are included in the same tables, so
            // we will lookup the interface number using the same method. See:
            //
            // https://developer.apple.com/documentation/bundleresources/entitlements/com_apple_developer_driverkit_transport_usb
            // https://developer.apple.com/library/archive/documentation/DeviceDrivers/Conceptual/USBBook/USBOverview/USBOverview.html#//apple_ref/doc/uid/TP40002644-BBCEACAJ
            #[cfg(feature = "usbportinfo-interface")]
            interface: get_int_property(usb_device, "bInterfaceNumber", kCFNumberSInt8Type)
                .map(|x| x as u8),
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
            use mach2::kern_return::KERN_SUCCESS;
            use mach2::port::{mach_port_t, MACH_PORT_NULL};

            let mut vec = Vec::new();
            unsafe {
                // Create a dictionary for specifying the search terms against the IOService
                let classes_to_match = IOServiceMatching(kIOSerialBSDServiceValue);
                if classes_to_match.is_null() {
                    return Err(Error::new(
                        ErrorKind::Unknown,
                        "IOServiceMatching returned a NULL dictionary.",
                    ));
                }
                let _classes_to_match_guard = scopeguard::guard((), |_| {
                    CFRelease(classes_to_match as *const c_void);
                });

                // Populate the search dictionary with a single key/value pair indicating that we're
                // searching for serial devices matching the RS232 device type.
                let key = CFStringCreateWithCString(
                    kCFAllocatorDefault,
                    kIOSerialBSDTypeKey,
                    kCFStringEncodingUTF8,
                );
                if key.is_null() {
                    return Err(Error::new(
                        ErrorKind::Unknown,
                        "Failed to allocate key string.",
                    ));
                }
                let _key_guard = scopeguard::guard((), |_| {
                    CFRelease(key as *const c_void);
                });

                let value = CFStringCreateWithCString(
                    kCFAllocatorDefault,
                    kIOSerialBSDAllTypes,
                    kCFStringEncodingUTF8,
                );
                if value.is_null() {
                    return Err(Error::new(
                        ErrorKind::Unknown,
                        "Failed to allocate value string.",
                    ));
                }
                let _value_guard = scopeguard::guard((), |_| {
                    CFRelease(value as *const c_void);
                });

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
                    CFRetain(classes_to_match as *const c_void) as *const __CFDictionary,
                    matching_services.as_mut_ptr(),
                );
                if kern_result != KERN_SUCCESS {
                    return Err(Error::new(
                        ErrorKind::Unknown,
                        format!("ERROR: {}", kern_result),
                    ));
                }
                let matching_services = matching_services.assume_init();
                let _matching_services_guard = scopeguard::guard((), |_| {
                    IOObjectRelease(matching_services);
                });

                loop {
                    // Grab the next result.
                    let modem_service = IOIteratorNext(matching_services);
                    // Break out if we've reached the end of the iterator
                    if modem_service == MACH_PORT_NULL {
                        break;
                    }
                    let _modem_service_guard = scopeguard::guard((), |_| {
                        IOObjectRelease(modem_service);
                    });

                    // Fetch all properties of the current search result item.
                    let mut props = MaybeUninit::uninit();
                    let result = IORegistryEntryCreateCFProperties(
                        modem_service,
                        props.as_mut_ptr(),
                        kCFAllocatorDefault,
                        0,
                    );
                    let _props_guard = scopeguard::guard((), |_| {
                        CFRelease(props.assume_init() as *const c_void);
                    });

                    if result == KERN_SUCCESS {
                        for key in ["IOCalloutDevice", "IODialinDevice"].iter() {
                            let key_cstring = CString::new(*key).unwrap();
                            let key_cfstring = CFStringCreateWithCString(
                                kCFAllocatorDefault,
                                key_cstring.as_ptr(),
                                kCFStringEncodingUTF8,
                            );
                            let _key_cfstring_guard = scopeguard::guard((), |_| {
                                CFRelease(key_cfstring as *const c_void);
                            });

                            let mut value = std::ptr::null();
                            let found = CFDictionaryGetValueIfPresent(props.assume_init(), key_cfstring as *const c_void, &mut value);
                            if found == true as Boolean {
                                let type_id = CFGetTypeID(value);
                                if type_id == CFStringGetTypeID() {
                                    let mut buf = Vec::with_capacity(256);

                                    CFStringGetCString(
                                        value as CFStringRef,
                                        buf.as_mut_ptr(),
                                        buf.capacity() as isize,
                                        kCFStringEncodingUTF8,
                                    );
                                    let path = CStr::from_ptr(buf.as_ptr()).to_string_lossy();
                                    vec.push(SerialPortInfo {
                                        port_name: path.to_string(),
                                        port_type: port_type(modem_service),
                                    });
                                } else {
                                    return Err(Error::new(ErrorKind::Unknown, "Found invalid type for TypeID"));
                                }
                            } else {
                                return Err(Error::new(ErrorKind::Unknown, format!("Key {} missing in dict", key)));
                            }
                        }
                    } else {
                        return Err(Error::new(ErrorKind::Unknown, format!("ERROR: {}", result)));
                    }
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

        /// Scans `/sys/class/tty` for serial devices (on Linux systems without libudev).
        pub fn available_ports() -> Result<Vec<SerialPortInfo>> {
            let mut vec = Vec::new();
            let sys_path = Path::new("/sys/class/tty/");
            let device_path = Path::new("/dev");
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

                // Generate the device file path `/dev/DEVICE` from the TTY class path
                // `/sys/class/tty/DEVICE` and emit a serial device if this path exists. There are
                // no further checks (yet) due to `Path::is_file` reports only regular files.
                //
                // See https://github.com/serialport/serialport-rs/issues/66 for details.
                if let Some(file_name) = raw_path.file_name() {
                    let device_file = device_path.join(file_name);
                    if !device_file.exists() {
                        continue;
                    }

                    vec.push(SerialPortInfo {
                        port_name: device_file.to_string_lossy().to_string(),
                        port_type: SerialPortType::Unknown,
                    });
                }
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
