extern crate argparse;
extern crate serialport;

use std::io::{self, Write};

use argparse::{ArgumentParser, Store};

fn main() {
    let mut port_name = "".to_string();
    {
        let mut ap = ArgumentParser::new();
        ap.set_description("Read from the given serial port at 9600 baud");
        ap.refer(&mut port_name)
            .add_argument("port", Store, "Port name")
            .required();
        ap.parse_args_or_exit();
    }

    if let Ok(mut port) = serialport::open(&port_name) {
        let mut serial_buf: Vec<u8> = vec![0; 1000];
        println!("Receiving data on {} at 9600 baud:", &port_name);
        loop {
            if let Ok(t) = port.read(serial_buf.as_mut_slice()) {
                io::stdout().write_all(&serial_buf[..t]).unwrap();
            }
        }
    } else {
        println!("Error: Port '{}' not available", &port_name);
    }
}
