extern crate serialport;

fn main() {

    if let Ok(ports) = serialport::available_ports() {
        match ports.len() {
            0 => println!("No ports found."),
            1 => println!("Found 1 port:"),
            n => println!("Found {} ports:", n),
        };
        for p in ports {
            println!("  {}", p.port_name);
            if let serialport::SerialPortType::UsbPort(info) = p.port_type {
                println!("    VID:{:04x} PID:{:04x}", info.vid, info.pid);
                println!("     Serial Number: {}",
                         info.serial_number.as_ref().map_or("", String::as_str));
                println!("      Manufacturer: {}",
                         info.manufacturer.as_ref().map_or("", String::as_str));
                println!("           Product: {}",
                         info.product.as_ref().map_or("", String::as_str));
            }
        }
    } else {
        print!("Error listing serial ports");
    }

}
