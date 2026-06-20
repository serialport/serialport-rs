use core::ffi::{c_int, c_uint, CStr};
use core::mem::MaybeUninit;

use crate::{Error, ErrorKind, Result, SerialPortInfo, SerialPortType, UsbPortInfo};

use objc2_core_foundation::{
    kCFAllocatorDefault, CFMutableDictionary, CFNumber, CFRetained, CFString, CFType,
};
use objc2_io_kit::{
    io_object_t, io_registry_entry_t, kIOMainPortDefault, kIOSerialBSDAllTypes,
    kIOSerialBSDServiceValue, kIOSerialBSDTypeKey, kIOServicePlane, kIOUSBDeviceClassName,
    kIOUSBHostInterfaceClassName, IOIteratorNext, IOMainPort, IOObjectGetClass, IOObjectRelease,
    IORegistryEntryCreateCFProperties, IORegistryEntryCreateCFProperty,
    IORegistryEntryGetParentEntry, IOServiceGetMatchingServices, IOServiceMatching,
};

// NOTE: Do not use `mach` nor `mach2` crates, they're unmaintained,
// and don't work on tvOS/watchOS/visionOS.
//
// Instead, define the types ourselves, we use sufficiently little
// that this is not a very hard task.
#[allow(non_camel_case_types)]
type kern_return_t = c_int;
#[allow(non_camel_case_types)]
type mach_port_t = c_uint;
const KERN_SUCCESS: kern_return_t = 0;
const MACH_PORT_NULL: mach_port_t = 0;

fn get_parent_device_by_type(
    device: io_object_t,
    parent_type: &CStr,
) -> Option<io_registry_entry_t> {
    let mut device = device;
    loop {
        let mut class_name = MaybeUninit::zeroed();
        // SAFETY: `device` is supposed to be valid `io_object_t` and `class_name` points to a
        // zero-initialzed buffer of the required size.
        if KERN_SUCCESS != unsafe { IOObjectGetClass(device, class_name.as_mut_ptr()) } {
            return None;
        }
        // SAFETY: We successfully called `IOObjectGetClass` and can now assume that `class_name`
        // contains either completely zeroed data ot a valid C string.
        let class_name = unsafe { class_name.assume_init() };

        // SAFETY: `class_name` contains a valid C sting (which might be empty).
        let name = unsafe { CStr::from_ptr(class_name.as_ptr()) };
        if name == parent_type {
            return Some(device);
        }

        let mut parent = MaybeUninit::uninit();
        if KERN_SUCCESS
            != unsafe {
                IORegistryEntryGetParentEntry(
                    device,
                    kIOServicePlane.as_ptr() as _,
                    parent.as_mut_ptr(),
                )
            }
        {
            return None;
        }
        // SAFETY: We checked right above that we successfully called
        // `IORegistryEntryGetParentEntry` and can assume a valid `io_registry_entry_t` in `parent`.
        device = unsafe { parent.assume_init() };
    }
}

#[allow(non_upper_case_globals)]
/// Returns a specific property of the given device as an integer.
fn get_int_property(device_type: io_registry_entry_t, property: &str) -> Result<u32> {
    let cf_property = CFString::from_str(property);

    // SAFETY: We are calling `IORegistryEntryCreateCFProperty` with valid arguments and allocated
    // `cf_property` right above.
    let cf_type = unsafe {
        IORegistryEntryCreateCFProperty(device_type, Some(&cf_property), kCFAllocatorDefault, 0)
    }
    .ok_or_else(|| Error::new(ErrorKind::Unknown, "Failed to get property"))?;

    cf_type
        .downcast::<CFNumber>()
        .ok()
        .and_then(|n| n.as_i64())
        .ok_or_else(|| Error::new(ErrorKind::Unknown, "Failed to get numerical value"))
        .and_then(|n| {
            n.try_into().map_err(|e| {
                Error::new(
                    ErrorKind::Unknown,
                    format!("Failed to convert to u32 ({e})"),
                )
            })
        })
}

/// Returns a specific property of the given device as a string.
fn get_string_property(device_type: io_registry_entry_t, property: &str) -> Result<String> {
    let cf_property = CFString::from_str(property);

    // SAFETY: We are calling `IORegistryEntryCreateCFProperty` with valid arguments and have
    // allocated `cf_property` right above.
    let cf_type = unsafe {
        IORegistryEntryCreateCFProperty(device_type, Some(&cf_property), kCFAllocatorDefault, 0)
    }
    .ok_or_else(|| Error::new(ErrorKind::Unknown, "Failed to get property"))?;

    cf_type
        .downcast::<CFString>()
        .ok()
        .map(|s| s.to_string())
        .ok_or(Error::new(ErrorKind::Unknown, "Failed to get string value"))
}

/// Determine the serial port type based on the service object (like that returned by
/// `IOIteratorNext`). Specific properties are extracted for USB devices.
fn port_type(service: io_object_t) -> SerialPortType {
    let bluetooth_device_class_name =
        CStr::from_bytes_with_nul(b"IOBluetoothSerialClient\0").unwrap();
    let usb_device_class_name = kIOUSBHostInterfaceClassName;
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

/// Scans the system for serial ports and returns a list of them.
/// The `SerialPortInfo` struct contains the name of the port which can be used for opening it.
pub fn available_ports() -> Result<Vec<SerialPortInfo>> {
    let mut vec = Vec::new();
    unsafe {
        // Create a dictionary for specifying the search terms against the IOService
        let classes_to_match =
            IOServiceMatching(kIOSerialBSDServiceValue.as_ptr()).ok_or_else(|| {
                Error::new(
                    ErrorKind::Unknown,
                    "IOServiceMatching returned a NULL dictionary.",
                )
            })?;
        let classes_to_match = classes_to_match.cast_unchecked::<CFString, CFType>();

        // Populate the search dictionary with a single key/value pair indicating that we're
        // searching for serial devices matching the RS232 device type.
        let search_key =
            CFString::from_static_str(kIOSerialBSDTypeKey.to_str().map_err(|_| {
                Error::new(ErrorKind::Unknown, "Failed to convert search key string")
            })?);
        let search_value =
            CFString::from_static_str(kIOSerialBSDAllTypes.to_str().map_err(|_| {
                Error::new(ErrorKind::Unknown, "Failed to convert search key string")
            })?);
        classes_to_match.set(&search_key, &search_value);

        // Get an interface to IOKit
        let mut master_port: mach_port_t = MACH_PORT_NULL;
        let mut kern_result = IOMainPort(MACH_PORT_NULL, &mut master_port);
        if kern_result != KERN_SUCCESS {
            return Err(Error::new(
                ErrorKind::Unknown,
                format!("ERROR: {}", kern_result),
            ));
        }

        // Run the search.
        let mut matching_services = MaybeUninit::uninit();
        kern_result = IOServiceGetMatchingServices(
            kIOMainPortDefault,
            Some(classes_to_match.as_opaque().into()),
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
                let props: CFRetained<CFMutableDictionary> =
                    CFRetained::from_raw(core::ptr::NonNull::new(props).unwrap());
                let props = props.cast_unchecked::<CFString, CFType>();

                for key in ["IOCalloutDevice", "IODialinDevice"].iter() {
                    let cf_key = CFString::from_str(key);

                    if let Some(cf_type) = props.get(&cf_key) {
                        match cf_type.downcast_ref::<CFString>().map(|s| s.to_string()) {
                            Some(path) => {
                                vec.push(SerialPortInfo {
                                    port_name: path,
                                    port_type: port_type(modem_service),
                                });
                            }
                            None => {
                                return Err(Error::new(
                                    ErrorKind::Unknown,
                                    format!("Failed to get string value for {}", key),
                                ))
                            }
                        }
                    } else {
                        return Err(Error::new(
                            ErrorKind::Unknown,
                            format!("Key {} missing in dict", key),
                        ));
                    }
                }
            } else {
                return Err(Error::new(ErrorKind::Unknown, format!("ERROR: {}", result)));
            }
        }
    }
    Ok(vec)
}
