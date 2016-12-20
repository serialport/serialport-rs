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
        }
    } else {
        print!("Error listing serial ports");
    }

}
