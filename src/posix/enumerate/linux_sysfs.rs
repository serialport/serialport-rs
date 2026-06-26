use std::fs::File;
use std::io::Read;
use std::path::Path;

use crate::{Result, SerialPortInfo, SerialPortType, UsbPortInfo};

fn is_rfcomm(path: &Path) -> bool {
    path.file_name()
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

fn read_file_to_u8(dir: &Path, file: &str) -> Option<u8> {
    u8::from_str_radix(&read_file_to_trimmed_string(dir, file)?, 16).ok()
}

fn read_port_type(path: &Path) -> Option<SerialPortType> {
    let path = path.canonicalize().ok()?;
    let subsystem = path.join("subsystem").canonicalize().ok()?;
    let subsystem = subsystem.file_name()?.to_string_lossy();

    match subsystem.as_ref() {
        // Broadcom SoC UARTs (of Raspberry Pi devices).
        "amba" => Some(SerialPortType::Unknown),
        "pci" => Some(SerialPortType::PciPort),
        "pnp" => Some(SerialPortType::Unknown),
        "serial-base" => Some(SerialPortType::Unknown),
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
        interface,
    })
}

/// Scans `/sys/class/tty` for serial devices (on Linux systems without libudev).
pub fn available_ports() -> Result<Vec<SerialPortInfo>> {
    let mut vec = Vec::new();
    let sys_path = Path::new("/sys/class/tty/");
    let dev_path = Path::new("/dev");
    for path in sys_path
        .read_dir()
        .expect("/sys/class/tty/ doesn't exist on this system")
    {
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
