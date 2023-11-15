use std::io::{self, Write};
use std::time::Duration;

use clap::{Arg, Command};

use serialport::{DataBits, StopBits};

fn main() {
    let matches = Command::new("Serialport Example - Heartbeat")
        .about("Write bytes to a serial port at 1Hz")
        .disable_version_flag(true)
        .arg(
            Arg::new("port")
                .help("The device path to a serial port")
                .required(true),
        )
        .arg(
            Arg::new("baud")
                .help("The baud rate to connect at")
                .use_value_delimiter(false)
                .required(true)
                .validator(valid_baud),
        )
        .arg(
            Arg::new("stop-bits")
                .long("stop-bits")
                .help("Number of stop bits to use")
                .takes_value(true)
                .possible_values(&["1", "2"])
                .default_value("1"),
        )
        .arg(
            Arg::new("data-bits")
                .long("data-bits")
                .help("Number of data bits to use")
                .takes_value(true)
                .possible_values(&["5", "6", "7", "8"])
                .default_value("8"),
        )
        .arg(
            Arg::new("rate")
                .long("rate")
                .help("Frequency (Hz) to repeat transmission of the pattern (0 indicates sending only once")
                .takes_value(true)
                .default_value("1"),
        )
        .arg(
            Arg::new("string")
                .long("string")
                .help("String to transmit")
                .takes_value(true)
                .default_value("."),
        )
        .get_matches();

    let port_name = matches.value_of("port").unwrap();
    let baud_rate = matches.value_of("baud").unwrap().parse::<u32>().unwrap();
    let stop_bits = match matches.value_of("stop-bits") {
        Some("2") => StopBits::Two,
        _ => StopBits::One,
    };
    let data_bits = match matches.value_of("data-bits") {
        Some("5") => DataBits::Five,
        Some("6") => DataBits::Six,
        Some("7") => DataBits::Seven,
        _ => DataBits::Eight,
    };
    let rate = matches.value_of("rate").unwrap().parse::<u32>().unwrap();
    let string = matches.value_of("string").unwrap();

    let builder = serialport::new(port_name, baud_rate)
        .stop_bits(stop_bits)
        .data_bits(data_bits);
    println!("{:?}", &builder);
    let mut port = builder.open().unwrap_or_else(|e| {
        eprintln!("Failed to open \"{}\". Error: {}", port_name, e);
        ::std::process::exit(1);
    });

    println!(
        "Writing '{}' to {} at {} baud at {}Hz",
        &string, &port_name, &baud_rate, &rate
    );
    loop {
        match port.write(string.as_bytes()) {
            Ok(_) => {
                print!("{}", &string);
                std::io::stdout().flush().unwrap();
            }
            Err(ref e) if e.kind() == io::ErrorKind::TimedOut => (),
            Err(e) => eprintln!("{:?}", e),
        }
        if rate == 0 {
            return;
        }
        std::thread::sleep(Duration::from_millis((1000.0 / (rate as f32)) as u64));
    }
}

fn valid_baud(val: &str) -> std::result::Result<(), String> {
    val.parse::<u32>()
        .map(|_| ())
        .map_err(|_| format!("Invalid baud rate '{}' specified", val))
}
