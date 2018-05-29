extern crate argparse;
extern crate serialport;

use std::io::{self, Write};
use std::time::Duration;

use argparse::{ArgumentParser, Store};
use serialport::prelude::*;

fn main() {
    let mut port_name = "".to_string();
    let mut baud_rate = "".to_string();
    {
        let mut ap = ArgumentParser::new();
        ap.set_description("Read from the given serial port");
        ap.refer(&mut port_name)
            .add_argument("port", Store, "Port name")
            .required();
        ap.refer(&mut baud_rate)
            .add_argument("baud", Store, "Baud rate")
            .required();
        ap.parse_args_or_exit();
    }

    let mut settings: SerialPortSettings = Default::default();
    settings.timeout = Duration::from_millis(10);
    if let Ok(rate) = baud_rate.parse::<u32>() {
        settings.baud_rate = rate.into();
    } else {
        println!("Error: Invalid baud rate '{}' specified", baud_rate);
        return;
    }

    if let Ok(mut port) = serialport::open_with_settings(&port_name, &settings) {
        let mut serial_buf: Vec<u8> = vec![0; 1000];
        println!("Receiving data on {} at {} baud:", &port_name, &baud_rate);
        loop {
            match port.read(serial_buf.as_mut_slice()) {
                Ok(t) => io::stdout().write_all(&serial_buf[..t]).unwrap(),
                Err(ref e) if e.kind() == io::ErrorKind::TimedOut => (),
                Err(e) => eprintln!("{:?}", e),
            }
        }
    } else {
        println!("Error: Port '{}' not available", &port_name);
    }
}
