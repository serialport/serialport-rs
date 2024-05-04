//! Tests for the `SerialPort` trait.
#![cfg(unix)]

use serialport::*;
use std::time::Duration;

const PORT_1: &str = env!("SERIALPORT_RS_TEST_PORT_1");
const PORT_2: &str = env!("SERIALPORT_RS_TEST_PORT_2");

#[test]
#[cfg_attr(feature = "ignore-hardware-tests", ignore)]
fn test_listing_ports() {
    let ports = serialport::available_ports().expect("No ports found!");
    for p in ports {
        println!("{}", p.port_name);
    }
}

#[test]
#[cfg_attr(feature = "ignore-hardware-tests", ignore)]
fn test_opening_found_ports() {
    // There is no guarantee that we even might open the ports returned by `available_ports`. But
    // the ports we are using for testing shall be among them.
    let ports = serialport::available_ports().unwrap();
    assert!(ports.iter().any(|info| info.port_name == PORT_1));
    assert!(ports.iter().any(|info| info.port_name == PORT_2));
}

#[test]
#[cfg_attr(feature = "ignore-hardware-tests", ignore)]
fn test_opening_port() {
    serialport::new(PORT_1, 9600).open().unwrap();
}

#[test]
#[cfg_attr(feature = "ignore-hardware-tests", ignore)]
fn test_opening_native_port() {
    serialport::new(PORT_1, 9600).open_native().unwrap();
}

#[test]
#[cfg_attr(feature = "ignore-hardware-tests", ignore)]
fn test_configuring_ports() {
    serialport::new(PORT_1, 9600)
        .data_bits(DataBits::Five)
        .flow_control(FlowControl::None)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .timeout(Duration::from_millis(1))
        .open()
        .unwrap();
}

#[test]
#[cfg_attr(feature = "ignore-hardware-tests", ignore)]
fn test_duplicating_port_config() {
    let port1_config = serialport::new(PORT_1, 9600)
        .data_bits(DataBits::Five)
        .flow_control(FlowControl::None)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .timeout(Duration::from_millis(1));

    let port2_config = port1_config.clone().path(PORT_2).baud_rate(115_200);

    let _port1 = port1_config.open().unwrap();
    let _port1 = port2_config.open().unwrap();
}
