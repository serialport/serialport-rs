use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(all(target_os = "linux", not(target_env = "musl"), feature = "libudev"))]{
        use std::ffi::OsStr;
    }
}

cfg_if! {
    if #[cfg(any(target_os = "ios", target_os = "macos"))] {
        use core_foundation::base::CFType;
        use core_foundation::base::TCFType;
        use core_foundation::dictionary::CFDictionary;
        use core_foundation::dictionary::CFMutableDictionary;
        use core_foundation::number::CFNumber;
        use core_foundation::string::CFString;
        use core_foundation_sys::base::{kCFAllocatorDefault, CFRetain};
        use io_kit_sys::*;
        use io_kit_sys::keys::*;
        use io_kit_sys::serial::keys::*;
        use io_kit_sys::types::*;
        use io_kit_sys::usb::lib::*;
        use nix::libc::{c_char, c_void};
        use std::ffi::CStr;
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
#[cfg(any(target_os = "ios", target_os = "linux", target_os = "macos"))]
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
        None if is_rfcomm(d) => Ok(SerialPortType::BluetoothPort),
        None => find_usb_interface_from_parents(d.parent())
            .and_then(get_modalias_from_device)
            .as_deref()
            .and_then(parse_modalias)
            .map_or(Ok(SerialPortType::Unknown), |port_info| {
                Ok(SerialPortType::UsbPort(port_info))
            }),
        _ => Ok(SerialPortType::Unknown),
    }
}

#[cfg(all(target_os = "linux", not(target_env = "musl"), feature = "libudev"))]
fn find_usb_interface_from_parents(parent: Option<libudev::Device>) -> Option<libudev::Device> {
    let mut p = parent?;

    // limit the query depth
    for _ in 1..4 {
        match p.devtype() {
            None => match p.parent() {
                None => break,
                Some(x) => p = x,
            },
            Some(s) => {
                if s.to_str()? == "usb_interface" {
                    break;
                } else {
                    match p.parent() {
                        None => break,
                        Some(x) => p = x,
                    }
                }
            }
        }
    }

    Some(p)
}

#[cfg(all(target_os = "linux", not(target_env = "musl"), feature = "libudev"))]
fn get_modalias_from_device(d: libudev::Device) -> Option<String> {
    Some(
        d.property_value("MODALIAS")
            .and_then(OsStr::to_str)?
            .to_owned(),
    )
}

//  MODALIAS = usb:v303Ap1001d0101dcEFdsc02dp01ic02isc02ip00in00
//  v    303A  (device vendor)
//  p    1001  (device product)
//  d    0101  (bcddevice)
//  dc     EF  (device class)
//  dsc    02  (device subclass)
//  dp     01  (device protocol)
//  ic     02  (interface class)
//  isc    02  (interface subclass)
//  ip     00  (interface protocol)
//  in     00  (interface number)
#[cfg(all(target_os = "linux", not(target_env = "musl"), feature = "libudev"))]
fn parse_modalias(moda: &str) -> Option<UsbPortInfo> {
    // Find the start of the string, will start with "usb:"
    let mod_start = moda.find("usb:v")?;

    // Tail to update while searching.
    let mut mod_tail = moda.get(mod_start + 5..)?;

    // The next four characters should be hex values of the vendor.
    let vid = mod_tail.get(..4)?;
    mod_tail = mod_tail.get(4..)?;

    // The next portion we care about is the device product ID.
    let pid_start = mod_tail.find('p')?;
    let pid = mod_tail.get(pid_start + 1..pid_start + 5)?;

    Some(UsbPortInfo {
        vid: u16::from_str_radix(vid, 16).ok()?,
        pid: u16::from_str_radix(pid, 16).ok()?,
        serial_number: None,
        manufacturer: None,
        product: None,
        // Only attempt to find the interface if the feature is enabled.
        #[cfg(feature = "usbportinfo-interface")]
        interface: mod_tail.get(pid_start + 4..).and_then(|mod_tail| {
            mod_tail.find("in").and_then(|i_start| {
                mod_tail
                    .get(i_start + 2..i_start + 4)
                    .and_then(|interface| u8::from_str_radix(interface, 16).ok())
            })
        }),
    })
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
fn get_int_property(device_type: io_registry_entry_t, property: &str) -> Result<u32> {
    let cf_property = CFString::new(property);

    let cf_type_ref = unsafe {
        IORegistryEntryCreateCFProperty(
            device_type,
            cf_property.as_concrete_TypeRef(),
            kCFAllocatorDefault,
            0,
        )
    };
    if cf_type_ref.is_null() {
        return Err(Error::new(ErrorKind::Unknown, "Failed to get property"));
    }

    let cf_type = unsafe { CFType::wrap_under_create_rule(cf_type_ref) };
    cf_type
        .downcast::<CFNumber>()
        .and_then(|n| n.to_i64())
        .map(|n| n as u32)
        .ok_or(Error::new(
            ErrorKind::Unknown,
            "Failed to get numerical value",
        ))
}

#[cfg(any(target_os = "ios", target_os = "macos"))]
/// Returns a specific property of the given device as a string.
fn get_string_property(device_type: io_registry_entry_t, property: &str) -> Result<String> {
    let cf_property = CFString::new(property);

    let cf_type_ref = unsafe {
        IORegistryEntryCreateCFProperty(
            device_type,
            cf_property.as_concrete_TypeRef(),
            kCFAllocatorDefault,
            0,
        )
    };
    if cf_type_ref.is_null() {
        return Err(Error::new(ErrorKind::Unknown, "Failed to get property"));
    }

    let cf_type = unsafe { CFType::wrap_under_create_rule(cf_type_ref) };
    cf_type
        .downcast::<CFString>()
        .map(|s| s.to_string())
        .ok_or(Error::new(ErrorKind::Unknown, "Failed to get string value"))
}

#[cfg(any(target_os = "ios", target_os = "macos"))]
/// Determine the serial port type based on the service object (like that returned by
/// `IOIteratorNext`). Specific properties are extracted for USB devices.
fn port_type(service: io_object_t) -> SerialPortType {
    let bluetooth_device_class_name = b"IOBluetoothSerialClient\0".as_ptr() as *const c_char;
    let usb_device_class_name = b"IOUSBHostInterface\0".as_ptr() as *const c_char;
    let legacy_usb_device_class_name = kIOUSBDeviceClassName;

    let maybe_usb_device = get_parent_device_by_type(service, usb_device_class_name)
        .or_else(|| get_parent_device_by_type(service, legacy_usb_device_class_name));
    if let Some(usb_device) = maybe_usb_device {
        SerialPortType::UsbPort(UsbPortInfo {
            vid: get_int_property(usb_device, "idVendor").unwrap_or_default() as u16,
            pid: get_int_property(usb_device, "idProduct").unwrap_or_default() as u16,
            serial_number: get_string_property(usb_device, "USB Serial Number").ok(),
            manufacturer: get_string_property(usb_device, "USB Vendor Name").ok(),
            product: get_string_property(usb_device, "USB Product Name").ok(),
            // Apple developer documentation indicates `bInterfaceNumber` is the supported key for
            // looking up the composite usb interface id. `idVendor` and `idProduct` are included in the same tables, so
            // we will lookup the interface number using the same method. See:
            //
            // https://developer.apple.com/documentation/bundleresources/entitlements/com_apple_developer_driverkit_transport_usb
            // https://developer.apple.com/library/archive/documentation/DeviceDrivers/Conceptual/USBBook/USBOverview/USBOverview.html#//apple_ref/doc/uid/TP40002644-BBCEACAJ
            #[cfg(feature = "usbportinfo-interface")]
            interface: get_int_property(usb_device, "bInterfaceNumber")
                .map(|x| x as u8)
                .ok(),
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
                let mut classes_to_match = CFMutableDictionary::wrap_under_create_rule(classes_to_match);

                // Populate the search dictionary with a single key/value pair indicating that we're
                // searching for serial devices matching the RS232 device type.
                let search_key = CStr::from_ptr(kIOSerialBSDTypeKey);
                let search_key = CFString::from_static_string(search_key.to_str().map_err(|_| Error::new(ErrorKind::Unknown, "Failed to convert search key string"))?);
                let search_value = CStr::from_ptr(kIOSerialBSDAllTypes);
                let search_value = CFString::from_static_string(search_value.to_str().map_err(|_| Error::new(ErrorKind::Unknown, "Failed to convert search key string"))?);
                classes_to_match.set(search_key, search_value);

                // Get an interface to IOKit
                let mut master_port: mach_port_t = MACH_PORT_NULL;
                let mut kern_result = IOMasterPort(MACH_PORT_NULL, &mut master_port);
                if kern_result != KERN_SUCCESS {
                    return Err(Error::new(
                        ErrorKind::Unknown,
                        format!("ERROR: {}", kern_result),
                    ));
                }

                // Run the search. IOServiceGetMatchingServices consumes one reference count of
                // classes_to_match, so explicitly retain.
                //
                // TODO: We could also just mem::forget classes_to_match like in
                // TCFType::into_CFType. Is there a special reason that there is no
                // TCFType::into_concrete_TypeRef()?
                CFRetain(classes_to_match.as_CFTypeRef());
                let mut matching_services = MaybeUninit::uninit();
                kern_result = IOServiceGetMatchingServices(
                    kIOMasterPortDefault,
                    classes_to_match.as_concrete_TypeRef(),
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
                    if result == KERN_SUCCESS {
                        // A successful call to IORegistryEntryCreateCFProperties indicates that a
                        // properties dict has been allocated and we as the caller are in charge of
                        // releasing it.
                        let props = props.assume_init();
                        let props: CFDictionary<CFString, *const c_void> = CFDictionary::wrap_under_create_rule(props);

                        for key in ["IOCalloutDevice", "IODialinDevice"].iter() {
                            let cf_key = CFString::new(key);

                            if let Some(cf_ref) = props.find(cf_key) {
                                let cf_type = CFType::wrap_under_get_rule(*cf_ref);
                                match cf_type
                                     .downcast::<CFString>()
                                     .map(|s| s.to_string())
                                {
                                    Some(path) => {
                                        vec.push(SerialPortInfo {
                                            port_name: path,
                                            port_type: port_type(modem_service),
                                        });
                                    }
                                    None => return Err(Error::new(ErrorKind::Unknown, format!("Failed to get string value for {}", key))),
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
        fn is_rfcomm(device: &libudev::Device) -> bool {
            device
                .sysname()
                .and_then(|o| o.to_str())
                .map(|s| s.starts_with("rfcomm"))
                .unwrap_or(false)
        }

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
                    if let Some(devnode) = d.devnode().and_then(|o| o.to_str()) {
                        let parent = d.parent();
                        if parent.is_some() || is_rfcomm(&d) {
                            if let Some(driver) = parent.as_ref().and_then(|d| d.driver()) {
                                if driver == "serial8250" && crate::new(devnode, 9600).open().is_err() {
                                    continue;
                                }
                            }

                            // Stop bubbling up port_type errors here so problematic ports are just
                            // skipped instead of causing no ports to be returned.
                            if let Ok(pt) = port_type(&d) {
                                vec.push(SerialPortInfo {
                                    port_name: String::from(devnode),
                                    port_type: pt,
                                });
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

        fn is_rfcomm(path: &Path) -> bool {
            path
                .file_name()
                .and_then(|o| o.to_str())
                .map(|s| s.starts_with("rfcomm"))
                .unwrap_or(false)
        }

        fn read_file_to_trimmed_string(dir: &Path, file: &str) -> Option<String> {
            let path = dir.join(file);
            let mut s = String::new();
            File::open(path).ok()?.read_to_string(&mut s).ok()?;
            Some(s.trim().to_owned())
        }

        fn read_file_to_u16(dir: &Path, file: &str) -> Option<u16> {
            u16::from_str_radix(&read_file_to_trimmed_string(dir, file)?, 16).ok()
        }

        #[cfg(feature = "usbportinfo-interface")]
        fn read_file_to_u8(dir: &Path, file: &str) -> Option<u8> {
            u8::from_str_radix(&read_file_to_trimmed_string(dir, file)?, 16).ok()
        }

        fn read_port_type(path: &Path) -> Option<SerialPortType> {
            let path = path
                .canonicalize()
                .ok()?;
            let subsystem = path.join("subsystem").canonicalize().ok()?;
            let subsystem = subsystem.file_name()?.to_string_lossy();

            match subsystem.as_ref() {
                // Broadcom SoC UARTs (of Raspberry Pi devices).
                "amba" => Some(SerialPortType::Unknown),
                "pci" => Some(SerialPortType::PciPort),
                "pnp" => Some(SerialPortType::Unknown),
                "usb" => usb_port_type(&path),
                "usb-serial" => usb_port_type(path.parent()?),
                _ => None,
            }
        }

        fn usb_port_type(interface_path: &Path) -> Option<SerialPortType> {
            let info = read_usb_port_info(interface_path)?;
            Some(SerialPortType::UsbPort(info))
        }

        fn read_usb_port_info(interface_path: &Path) -> Option<UsbPortInfo> {
            let device_path = interface_path.parent()?;

            let vid = read_file_to_u16(&device_path, "idVendor")?;
            let pid = read_file_to_u16(&device_path, "idProduct")?;
            #[cfg(feature = "usbportinfo-interface")]
            let interface = read_file_to_u8(&interface_path, &"bInterfaceNumber");
            let serial_number = read_file_to_trimmed_string(&device_path, &"serial");
            let product = read_file_to_trimmed_string(&device_path, &"product");
            let manufacturer = read_file_to_trimmed_string(&device_path, &"manufacturer");

            Some(UsbPortInfo {
                vid,
                pid,
                serial_number,
                manufacturer,
                product,
                #[cfg(feature = "usbportinfo-interface")]
                interface,
            })
        }

        /// Scans `/sys/class/tty` for serial devices (on Linux systems without libudev).
        pub fn available_ports() -> Result<Vec<SerialPortInfo>> {
            let mut vec = Vec::new();
            let sys_path = Path::new("/sys/class/tty/");
            let dev_path = Path::new("/dev");
            for path in sys_path.read_dir().expect("/sys/class/tty/ doesn't exist on this system") {
                let raw_path = path?.path().clone();
                let mut path = raw_path.clone();

                let port_type = if is_rfcomm(&raw_path) {
                    SerialPortType::BluetoothPort
                } else {
                    path.push("device");
                    if !path.is_dir() {
                        continue;
                    }

                    // Determine port type and proceed, if it's a known.
                    //
                    // TODO: Switch to a likely more readable let-else statement when our MSRV supports
                    // it.
                    let port_type = read_port_type(&path);
                    if let Some(port_type) = port_type {
                        port_type
                    } else {
                        continue;
                    }
                };

                // Generate the device file path `/dev/DEVICE` from the TTY class path
                // `/sys/class/tty/DEVICE` and emit a serial device if this path exists. There are
                // no further checks (yet) due to `Path::is_file` reports only regular files.
                //
                // See https://github.com/serialport/serialport-rs/issues/66 for details.
                if let Some(file_name) = raw_path.file_name() {
                    let device_file = dev_path.join(file_name);
                    if !device_file.exists() {
                        continue;
                    }

                    vec.push(SerialPortInfo {
                        port_name: device_file.to_string_lossy().to_string(),
                        port_type,
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

#[cfg(all(
    test,
    target_os = "linux",
    not(target_env = "musl"),
    feature = "libudev"
))]
mod tests {
    use super::*;

    use quickcheck_macros::quickcheck;

    #[quickcheck]
    fn quickcheck_parse_modalias_does_not_panic_from_random_data(modalias: String) -> bool {
        let _ = parse_modalias(&modalias);
        true
    }

    #[test]
    fn parse_modalias_canonical() {
        const MODALIAS: &str = "usb:v303Ap1001d0101dcEFdsc02dp01ic02isc02ip00in0C";

        let port_info = parse_modalias(MODALIAS).expect("parse failed");

        assert_eq!(port_info.vid, 0x303A, "vendor parse invalid");
        assert_eq!(port_info.pid, 0x1001, "product parse invalid");

        #[cfg(feature = "usbportinfo-interface")]
        assert_eq!(port_info.interface, Some(0x0C), "interface parse invalid");
    }

    #[test]
    fn parse_modalias_corner_cases() {
        assert!(parse_modalias("").is_none());
        assert!(parse_modalias("usb").is_none());
        assert!(parse_modalias("usb:").is_none());
        assert!(parse_modalias("usb:vdcdc").is_none());
        assert!(parse_modalias("usb:pdcdc").is_none());

        // Just vendor and product IDs.
        let info = parse_modalias("usb:vdcdcpabcd").unwrap();
        assert_eq!(info.vid, 0xdcdc);
        assert_eq!(info.pid, 0xabcd);
        #[cfg(feature = "usbportinfo-interface")]
        assert!(info.interface.is_none());

        // Vendor and product ID plus an interface number.
        let info = parse_modalias("usb:v1234p5678indc").unwrap();
        assert_eq!(info.vid, 0x1234);
        assert_eq!(info.pid, 0x5678);
        #[cfg(feature = "usbportinfo-interface")]
        assert_eq!(info.interface, Some(0xdc));
    }
}
