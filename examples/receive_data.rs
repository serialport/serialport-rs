extern crate argparse;
extern crate serialport;

use std::io::{self, Write};

use argparse::{ArgumentParser, Store};
use serialport::prelude::*;

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
        port.set_baud_rate(BaudRate::Baud9600).unwrap();
        port.set_stop_bits(StopBits::One).unwrap();
        port.set_parity(Parity::None).unwrap();
        port.set_data_bits(DataBits::Eight).unwrap();
        port.set_flow_control(FlowControl::None).unwrap();
        let mut serial_buf: Vec<u8> = vec![0; 1000];
        println!("Receiving data on {} at 9600 baud:", &port_name);
        loop {
            if let Ok(t) = port.read(serial_buf.as_mut_slice()) {
                io::stdout().write(&serial_buf[..t]).unwrap();
            }
        }
    } else {
        println!("Error: Port '{}' not available", &port_name);
    }
}
