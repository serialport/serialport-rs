use std::path::Path;

use crate::{Result, SerialPortInfo, SerialPortType};

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
        if (filename_string.starts_with("cuaU")
            || filename_string.starts_with("cuau")
            || filename_string.starts_with("cuad"))
            && !filename_string.ends_with(".init")
            && !filename_string.ends_with(".lock")
        {
            vec.push(SerialPortInfo {
                port_name: path.path().to_string_lossy().to_string(),
                port_type: SerialPortType::Unknown,
            });
        }
    }
    Ok(vec)
}
