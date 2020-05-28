//! Tests for the `SerialPort` trait.
#![cfg(unix)]

use serialport::*;
use std::time::Duration;

#[test]
fn test_listing_ports() {
    let ports = serialport::available_ports().expect("No ports found!");
    for p in ports {
        println!("{}", p.port_name);
    }
}

#[test]
fn test_opening_found_ports() {
    let ports = serialport::available_ports().unwrap();
    for p in ports {
        let _port = serialport::new(p.port_name, 9600).open();
    }
}

#[test]
fn test_opening_port() {
    let _port = serialport::new("/dev/ttyUSB0", 9600).open();
}

#[test]
fn test_opening_native_port() {
    let _port = serialport::new("/dev/ttyUSB0", 9600).open_native();
}

#[test]
fn test_configuring_ports() {
    let _port = serialport::new("/dev/ttyUSB0", 9600)
        .data_bits(DataBits::Five)
        .flow_control(FlowControl::None)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .timeout(Duration::from_millis(1))
        .open();
}

#[test]
fn test_duplicating_port_config() {
    let port1_config = serialport::new("/dev/ttyUSB0", 9600)
        .data_bits(DataBits::Five)
        .flow_control(FlowControl::None)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .timeout(Duration::from_millis(1));

    let port2_config = port1_config.clone().path("/dev/ttyUSB1").baud_rate(115_200);

    let _port1 = port1_config.open();
    let _port1 = port2_config.open();
}
