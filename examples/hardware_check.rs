//! This example performs a test using real hardware ports.
//!
//! This tool serves as a debugging aid when encountering errors using `serialport-rs`. It should
//! expose any kernel or driver bugs that your system may have by running physical ports through
//! many configurations.
//!
//! There are 3 ways to run this example:
//!
//!  1) With a single port not connected to an external device:
//!     `cargo run --example hardware_check /dev/ttyUSB0`
//!
//!  2) With a single port physically connected in loopback mode (RX<->TX)
//!     `cargo run --example hardware_check /dev/ttyUSB0 --loopback`
//!
//!  3) With two ports physically connected to each other
//!     `cargo run --example hardware_check /dev/ttyUSB0 /dev/ttyUSB1`

extern crate argparse;
extern crate serialport;

use argparse::{ArgumentParser, Store, StoreTrue};

use std::io::Write;
use std::str;
use std::time::Duration;

use serialport::prelude::*;

fn main() {
    let mut port1_name = "".to_string();
    let mut port2_name = "".to_string();
    let mut port1_loopback = false;
    {
        let mut ap = ArgumentParser::new();
        ap.set_description("Test serial ports");
        ap.refer(&mut port1_name)
            .add_argument("port1", Store, "Port 1 name")
            .required();
        ap.refer(&mut port1_loopback)
            .add_option(&["-l", "--loopback"], StoreTrue, "Enable loopback tests for port 1 (unavailable when both ports are specified)");
        ap.refer(&mut port2_name)
            .add_argument("port2", Store, "Port 2 name");
        ap.parse_args_or_exit();
    }

    // Loopback mode is only available when a single port is specified
    if port1_loopback && port2_name != "" {
        eprintln!("ERROR: loopback mode can only be enabled when a single port is specified.");
        ::std::process::exit(1);
    }

    // Run single-port tests on port1
    let mut port1 = match serialport::open(&port1_name) {
        Err(e) => {
            eprintln!(
                "Failed to open \"{}\". Error: {}",
                port2_name,
                e
            );
            ::std::process::exit(1);
        },
        Ok(p) => p,
    };
    test_single_port(&mut *port1, port1_loopback);

    if port2_name != "" {
        // Run single-port tests on port2
        let mut port2 = match serialport::open(&port2_name) {
            Err(e) => {
                eprintln!(
                    "Failed to open \"{}\". Error: {}",
                    port2_name,
                    e
                );
                ::std::process::exit(1);
            },
            Ok(p) => p,
        };
        test_single_port(&mut *port2, false);

        // Test loopback pair
        test_dual_ports(&mut *port1, &mut *port2);
    }
}

macro_rules! baud_rate_check {
    ($port:ident, $baud:expr) => {
        let baud_rate = $baud;
        if let Err(e) = $port.set_baud_rate(baud_rate) {
            println!("  {:?}: FAILED ({})", baud_rate, e);
        }
        match $port.baud_rate() {
            Err(_) => println!("  {:?}: FAILED (error retrieving baud rate)", baud_rate),
            Ok(r) if r != baud_rate => println!(
                "  {:?}: FAILED (baud rate {:?} does not match set baud rate {:?})",
                baud_rate, r, baud_rate
            ),
            Ok(_) => println!("  {:?}: success", baud_rate),
        }
    };
}

macro_rules! data_bits_check {
    ($port:ident, $data_bits:path) => {
        let data_bits = $data_bits;
        if let Err(e) = $port.set_data_bits(data_bits) {
            println!("  {:?}: FAILED ({})", data_bits, e);
        } else {
            match $port.data_bits() {
                Err(_) => println!("FAILED to retrieve data bits"),
                Ok(r) if r != data_bits => println!(
                    "  {:?}: FAILED (data bits {:?} does not match set data bits {:?})",
                    data_bits, r, data_bits
                ),
                Ok(_) => println!("  {:?}: success", data_bits),
            }
        }
    };
}

macro_rules! flow_control_check {
    ($port:ident, $flow_control:path) => {
        let flow_control = $flow_control;
        if let Err(e) = $port.set_flow_control(flow_control) {
            println!("  {:?}: FAILED ({})", flow_control, e);
        } else {
            match $port.flow_control() {
                Err(_) => println!("FAILED to retrieve flow control"),
                Ok(r) if r != flow_control => println!(
                    "  {:?}: FAILED (flow control {:?} does not match set flow control {:?})",
                    flow_control, r, flow_control
                ),
                Ok(_) => println!("  {:?}: success", flow_control),
            }
        }
    };
}

macro_rules! parity_check {
    ($port:ident, $parity:path) => {
        let parity = $parity;
        if let Err(e) = $port.set_parity(parity) {
            println!("  {:?}: FAILED ({})", parity, e);
        } else {
            match $port.parity() {
                Err(_) => println!("FAILED to retrieve parity"),
                Ok(r) if r != parity => println!(
                    "  {:?}: FAILED (parity {:?} does not match set parity {:?})",
                    parity, r, parity
                ),
                Ok(_) => println!("  {:?}: success", parity),
            }
        }
    };
}

macro_rules! stop_bits_check {
    ($port:ident, $stop_bits:path) => {
        let stop_bits = $stop_bits;
        if let Err(e) = $port.set_stop_bits(stop_bits) {
            println!("  {:?}: FAILED ({})", stop_bits, e);
        } else {
            match $port.stop_bits() {
                Err(_) => println!("FAILED to retrieve stop bits"),
                Ok(r) if r != stop_bits => println!(
                    "FAILED, stop bits {:?} does not match set stop bits {:?}",
                    r, stop_bits
                ),
                Ok(_) => println!("  {:?}: success", stop_bits),
            }
        }
    };
}

macro_rules! clear_check {
    ($port:ident, $buffer_direction:path) => {
        let buffer_direction = $buffer_direction;
        match $port.clear(buffer_direction) {
            Ok(_) => println!("  {:?}: success", buffer_direction),
            Err(ref e) => println!("  {:?}: FAILED ({})", buffer_direction, e),
        }
    };
}

macro_rules! call_query_method_check {
    ($port:ident, $func:path) => {
        match $func($port) {
            Ok(_) => println!("  {}: success", stringify!($func)),
            Err(ref e) => println!("  {}: FAILED ({})", stringify!($func), e),
        }
    };
}

fn test_single_port(port: &mut serialport::SerialPort, loopback: bool) {
    println!("Testing '{}':", port.name().unwrap());

    // Test setting standard baud rates
    println!("Testing baud rates...");
    baud_rate_check!(port, 9600);
    baud_rate_check!(port, 38_400);
    baud_rate_check!(port, 115_200);

    // Test setting non-standard baud rates
    println!("Testing non-standard baud rates...");
    baud_rate_check!(port, 10_000);
    baud_rate_check!(port, 600_000);
    baud_rate_check!(port, 1_800_000);

    // Test setting the data bits
    println!("Testing data bits...");
    data_bits_check!(port, DataBits::Five);
    data_bits_check!(port, DataBits::Six);
    data_bits_check!(port, DataBits::Seven);
    data_bits_check!(port, DataBits::Eight);

    // Test setting flow control
    println!("Testing flow control...");
    flow_control_check!(port, FlowControl::Software);
    flow_control_check!(port, FlowControl::Hardware);
    flow_control_check!(port, FlowControl::None);

    // Test setting parity
    println!("Testing parity...");
    parity_check!(port, Parity::Odd);
    parity_check!(port, Parity::Even);
    parity_check!(port, Parity::None);

    // Test setting stop bits
    println!("Testing stop bits...");
    stop_bits_check!(port, StopBits::Two);
    stop_bits_check!(port, StopBits::One);

    // Test bytes to read and write
    println!("Testing bytes to read and write...");
    call_query_method_check!(port, SerialPort::bytes_to_write);
    call_query_method_check!(port, SerialPort::bytes_to_read);

    // Test clearing a buffer
    println!("Test clearing software buffers...");
    clear_check!(port, ClearBuffer::Input);
    clear_check!(port, ClearBuffer::Output);
    clear_check!(port, ClearBuffer::All);

    // Test transmitting data
    print!("Testing data transmission...");
    std::io::stdout().flush().unwrap();
    // Make sure the port has sane defaults
    let port_settings: SerialPortSettings = Default::default();
    port
        .set_all(&port_settings)
        .expect("Resetting port to sane defaults failed");
    let msg = "Test Message";
    port.write_all(msg.as_bytes()).expect("Unable to write bytes.");
    println!("success");

    print!("Testing data reception...");
    if loopback {
        let mut buf = [0u8; 12];
        if port.read_exact(&mut buf).is_err() {
            println!("FAILED");
        } else {
            assert_eq!(
                str::from_utf8(&buf).unwrap(),
                msg,
                "Received message does not match sent"
            );
            println!("success");
        }
    }
}

fn test_dual_ports(port1: &mut serialport::SerialPort, port2: &mut serialport::SerialPort) {

    println!(
        "Testing paired ports '{}' and '{}':",
        port1.name().unwrap(),
        port2.name().unwrap()
    );

    // Make sure both ports are set to sane defaults
    let mut port_settings: SerialPortSettings = Default::default();
    port_settings.timeout = Duration::from_millis(100);
    port_settings.baud_rate = 115_200;
    port1
        .set_all(&port_settings)
        .expect("Resetting port1 to sane defaults failed");
    port2
        .set_all(&port_settings)
        .expect("Resetting port2 to sane defaults failed");

    let msg = "Test Message";
    let mut buf = [0u8; 12];

    // Test sending strings from port1 to port2
    println!(
        "  Transmitting from {} to {}...",
        port1.name().unwrap(),
        port2.name().unwrap()
    );
    let baud_rate = 2_000_000;
    print!("     At {},8,n,1,noflow...", baud_rate);
    std::io::stdout().flush().unwrap();
    if port1.set_baud_rate(baud_rate).is_ok()
        && port2.set_baud_rate(baud_rate).is_ok() {
        port1.write_all(msg.as_bytes()).expect("Unable to write bytes.");
        if port2.read_exact(&mut buf).is_err() {
            println!("FAILED");
        } else {
            assert_eq!(
                str::from_utf8(&buf).unwrap(),
                msg,
                "Received message does not match sent"
            );
            println!("success");
        }
    } else {
        println!("FAILED (does this platform & port support arbitrary baud rates?)");
    }
    let baud_rate = 115_200;
    print!("     At {},8,n,1,noflow...", baud_rate);
    std::io::stdout().flush().unwrap();
    if port1.set_baud_rate(baud_rate).is_ok()
        && port2.set_baud_rate(baud_rate).is_ok() {
        port1.write_all(msg.as_bytes()).expect("Unable to write bytes.");
        if port2.read_exact(&mut buf).is_err() {
            println!("FAILED");
        } else {
            assert_eq!(
                str::from_utf8(&buf).unwrap(),
                msg,
                "Received message does not match sent"
            );
            println!("success");
        }
    } else {
        println!("FAILED");
    }
    let baud_rate = 57_600;
    print!("     At {},8,n,1,noflow...", baud_rate);
    std::io::stdout().flush().unwrap();
    if port1.set_baud_rate(baud_rate).is_ok()
        && port2.set_baud_rate(baud_rate).is_ok() {
        port1.write_all(msg.as_bytes()).expect("Unable to write bytes.");
        if port2.read_exact(&mut buf).is_err() {
            println!("FAILED");
        } else {
            assert_eq!(
                str::from_utf8(&buf).unwrap(),
                msg,
                "Received message does not match sent"
            );
            println!("success");
        }
    } else {
        println!("FAILED");
    }
    let baud_rate = 10_000;
    print!("     At {},8,n,1,noflow...", baud_rate);
    std::io::stdout().flush().unwrap();
    if port1.set_baud_rate(baud_rate).is_ok()
        && port2.set_baud_rate(baud_rate).is_ok() {
        port1.write_all(msg.as_bytes()).expect("Unable to write bytes.");
        if port2.read_exact(&mut buf).is_err() {
            println!("FAILED");
        } else {
            assert_eq!(
                str::from_utf8(&buf).unwrap(),
                msg,
                "Received message does not match sent"
            );
            println!("success");
        }
    } else {
        println!("FAILED (does this platform & port support arbitrary baud rates?)");
    }
    let baud_rate = 9600;
    print!("     At {},8,n,1,noflow...", baud_rate);
    std::io::stdout().flush().unwrap();
    if port1.set_baud_rate(baud_rate).is_ok()
        && port2.set_baud_rate(baud_rate).is_ok() {
        port1.write_all(msg.as_bytes()).expect("Unable to write bytes.");
        if port2.read_exact(&mut buf).is_err() {
            println!("FAILED");
        } else {
            assert_eq!(
                str::from_utf8(&buf).unwrap(),
                msg,
                "Received message does not match sent"
            );
            println!("success");
        }
    } else {
        println!("FAILED");
    }

    // Test flow control
    port1.set_flow_control(FlowControl::Software).unwrap();
    port2.set_flow_control(FlowControl::Software).unwrap();
    print!("     At 9600,8,n,1,softflow...");
    std::io::stdout().flush().unwrap();
    port2.write_all(msg.as_bytes()).expect("Unable to write bytes.");
    if port1.read_exact(&mut buf).is_err() {
        println!("FAILED");
    } else {
        assert_eq!(
            str::from_utf8(&buf).unwrap(),
            msg,
            "Received message does not match sent"
        );
        println!("success");
    }
    port1.set_flow_control(FlowControl::Hardware).unwrap();
    port2.set_flow_control(FlowControl::Hardware).unwrap();
    print!("     At 9600,8,n,1,hardflow...");
    std::io::stdout().flush().unwrap();
    port2.write_all(msg.as_bytes()).expect("Unable to write bytes.");
    if port1.read_exact(&mut buf).is_err() {
        println!("FAILED");
    } else {
        assert_eq!(
            str::from_utf8(&buf).unwrap(),
            msg,
            "Received message does not match sent"
        );
        println!("success");
    }
}
