use std::ffi::OsStr;

use crate::{Error, ErrorKind, Result, SerialPortInfo, SerialPortType, UsbPortInfo};

/// Retrieves the udev property value named by `key`. If the value exists, then it will be
/// converted to a String, otherwise None will be returned.
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
fn udev_restore_spaces(source: String) -> String {
    source.replace('_', " ")
}

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
        interface: mod_tail.get(pid_start + 4..).and_then(|mod_tail| {
            mod_tail.find("in").and_then(|i_start| {
                mod_tail
                    .get(i_start + 2..i_start + 4)
                    .and_then(|interface| u8::from_str_radix(interface, 16).ok())
            })
        }),
    })
}

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

#[cfg(test)]
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
        assert!(info.interface.is_none());

        // Vendor and product ID plus an interface number.
        let info = parse_modalias("usb:v1234p5678indc").unwrap();
        assert_eq!(info.vid, 0x1234);
        assert_eq!(info.pid, 0x5678);
        assert_eq!(info.interface, Some(0xdc));
    }
}
