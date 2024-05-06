//! Tests for the `SerialPort` trait.
mod config;

use config::{hw_config, HardwareConfig};
use rstest::rstest;
use serialport::*;
use std::time::Duration;

#[rstest]
#[cfg_attr(feature = "ignore-hardware-tests", ignore)]
fn test_listing_ports() {
    let ports = serialport::available_ports().expect("No ports found!");
    for p in ports {
        println!("{}", p.port_name);
    }
}

#[rstest]
#[cfg_attr(feature = "ignore-hardware-tests", ignore)]
fn test_opening_found_ports(hw_config: HardwareConfig) {
    // There is no guarantee that we even might open the ports returned by `available_ports`. But
    // the ports we are using for testing shall be among them.
    let ports = serialport::available_ports().unwrap();
    assert!(ports.iter().any(|info| info.port_name == hw_config.port_1));
    assert!(ports.iter().any(|info| info.port_name == hw_config.port_2));
}

#[rstest]
#[cfg_attr(feature = "ignore-hardware-tests", ignore)]
fn test_opening_port(hw_config: HardwareConfig) {
    serialport::new(hw_config.port_1, 9600).open().unwrap();
}

#[rstest]
#[cfg_attr(feature = "ignore-hardware-tests", ignore)]
fn test_opening_native_port(hw_config: HardwareConfig) {
    serialport::new(hw_config.port_1, 9600)
        .open_native()
        .unwrap();
}

#[rstest]
#[cfg_attr(feature = "ignore-hardware-tests", ignore)]
fn test_configuring_ports(hw_config: HardwareConfig) {
    serialport::new(hw_config.port_1, 9600)
        .data_bits(DataBits::Five)
        .flow_control(FlowControl::None)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .timeout(Duration::from_millis(1))
        .open()
        .unwrap();
}

#[rstest]
#[cfg_attr(feature = "ignore-hardware-tests", ignore)]
fn test_duplicating_port_config(hw_config: HardwareConfig) {
    let port1_config = serialport::new(hw_config.port_1, 9600)
        .data_bits(DataBits::Five)
        .flow_control(FlowControl::None)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .timeout(Duration::from_millis(1));

    let port2_config = port1_config
        .clone()
        .path(hw_config.port_2)
        .baud_rate(115_200);

    let _port1 = port1_config.open().unwrap();
    let _port1 = port2_config.open().unwrap();
}
