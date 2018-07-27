extern crate clap;
extern crate serialport;

use std::io::{self, Write};
use std::time::Duration;

use clap::{Arg, App, AppSettings};
use serialport::prelude::*;

fn main() {
    let matches = App::new("Serialport Example - Heartbeat")
        .about("Write bytes to a serial port at 1Hz")
        .setting(AppSettings::DisableVersion)
        .arg(Arg::with_name("port")
             .help("The device path to a serial port")
             .use_delimiter(false)
             .required(true))
        .arg(Arg::with_name("baud")
             .help("The baud rate to connect at")
             .use_delimiter(false)
             .required(true))
        .get_matches();
    let port_name = matches.value_of("port").unwrap();
    let baud_rate = matches.value_of("baud").unwrap();

    let mut settings: SerialPortSettings = Default::default();
    settings.timeout = Duration::from_millis(10);
    if let Ok(rate) = baud_rate.parse::<u32>() {
        settings.baud_rate = rate.into();
    } else {
        eprintln!("Error: Invalid baud rate '{}' specified", baud_rate);
        ::std::process::exit(1);
    }

    match serialport::open_with_settings(&port_name, &settings) {
        Ok(mut port) => {
            println!("Writing '.' to {} at {} baud at 1Hz", &port_name, &baud_rate);
            loop {
                match port.write(".".as_bytes()) {
                    Ok(_) => {
                        print!(".");
                        std::io::stdout().flush().unwrap();
                    },
                    Err(ref e) if e.kind() == io::ErrorKind::TimedOut => (),
                    Err(e) => eprintln!("{:?}", e),
                }
                std::thread::sleep(Duration::from_secs(1));
            }
        },
        Err(e) => {
            eprintln!(
                "Failed to open \"{}\". Error: {}",
                port_name,
                e
            );
            ::std::process::exit(1);
        },
    }
}
