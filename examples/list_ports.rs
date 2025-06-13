use serialport::{available_ports, SerialPortType};

fn main() {
    match available_ports() {
        Ok(mut ports) => {
            // Let's output ports in a stable order to facilitate comparing the output from
            // different runs (on different platforms, with different features, ...).
            ports.sort_by_key(|i| i.port_name.clone());

            match ports.len() {
                0 => println!("No ports found."),
                1 => println!("Found 1 port:"),
                n => println!("Found {} ports:", n),
            };

            for p in ports {
                println!("    {}", p.port_name);
                match p.port_type {
                    SerialPortType::UsbPort(info) => {
                        println!("        Type: USB");
                        println!("        VID: {:04x}", info.vid);
                        println!("        PID: {:04x}", info.pid);
                        #[cfg(feature = "usbportinfo-interface")]
                        println!(
                            "        Interface: {}",
                            info.interface
                                .as_ref()
                                .map_or("".to_string(), |x| format!("{:02x}", *x))
                        );
                        println!(
                            "        Serial Number: {}",
                            info.serial_number.as_ref().map_or("", String::as_str)
                        );
                        println!(
                            "        Manufacturer: {}",
                            info.manufacturer.as_ref().map_or("", String::as_str)
                        );
                        println!(
                            "        Product: {}",
                            info.product.as_ref().map_or("", String::as_str)
                        );
                    }
                    SerialPortType::BluetoothPort => {
                        println!("        Type: Bluetooth");
                    }
                    SerialPortType::PciPort => {
                        println!("        Type: PCI");
                    }
                    SerialPortType::Unknown => {
                        println!("        Type: Unknown");
                    }
                }
            }
        }
        Err(e) => {
            eprintln!("{:?}", e);
            eprintln!("Error listing serial ports");
        }
    }
}
