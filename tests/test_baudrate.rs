mod config;

use config::{hw_config, HardwareConfig};
use rstest::rstest;
use rstest_reuse::{self, apply, template};
use serialport::SerialPort;
use std::ops::Range;

const RESET_BAUD_RATE: u32 = 300;

/// Returs an acceptance interval for the actual baud rate returned from the device after setting
/// the supplied value. For example, the CP2102 driver on Linux returns the baud rate actually
/// configured a the device rather than the the value set.
fn accepted_actual_baud_for(baud: u32) -> Range<u32> {
    let delta = baud / 200;
    baud.checked_sub(delta).unwrap()..baud.checked_add(delta).unwrap()
}

fn check_baud_rate(port: &dyn SerialPort, baud: u32) {
    let accepted = accepted_actual_baud_for(baud);
    let actual = port.baud_rate().unwrap();
    assert!(accepted.contains(&actual));
}

#[template]
#[rstest]
#[case(9600)]
#[case(57600)]
#[case(115200)]
fn standard_baud_rates(#[case] baud: u32) {}

#[template]
#[rstest]
#[case(1000)]
#[case(42000)]
#[case(100000)]
fn non_standard_baud_rates(#[case] baud: u32) {}

/// Test cases for setting the baud rate via [`SerialPortBuilder`].
mod builder {
    use super::*;

    #[apply(standard_baud_rates)]
    #[cfg_attr(feature = "ignore-hardware-tests", ignore)]
    fn test_standard_baud_rate(hw_config: HardwareConfig, #[case] baud: u32) {
        let port = serialport::new(hw_config.port_1, RESET_BAUD_RATE)
            .baud_rate(baud)
            .open()
            .unwrap();
        check_baud_rate(port.as_ref(), baud);
    }

    #[apply(non_standard_baud_rates)]
    #[cfg_attr(
        any(
            feature = "ignore-hardware-tests",
            not(all(target_os = "linux", target_env = "musl")),
        ),
        ignore
    )]
    fn test_non_standard_baud_rate_fails_where_not_supported(
        hw_config: HardwareConfig,
        #[case] baud: u32,
    ) {
        let res = serialport::new(hw_config.port_1, RESET_BAUD_RATE)
            .baud_rate(baud)
            .open();
        assert!(res.is_err());
    }

    #[apply(non_standard_baud_rates)]
    #[cfg_attr(
        any(
            feature = "ignore-hardware-tests",
            all(target_os = "linux", target_env = "musl"),
        ),
        ignore
    )]
    fn test_non_standard_baud_rate_succeeds_where_supported(
        hw_config: HardwareConfig,
        #[case] baud: u32,
    ) {
        let port = serialport::new(hw_config.port_1, RESET_BAUD_RATE)
            .baud_rate(baud)
            .open()
            .unwrap();
        check_baud_rate(port.as_ref(), baud);
    }
}

/// Test cases for setting the baud rate via [`serialport::new`].
mod new {
    use super::*;

    #[apply(standard_baud_rates)]
    #[cfg_attr(feature = "ignore-hardware-tests", ignore)]
    fn test_standard_baud_rate(hw_config: HardwareConfig, #[case] baud: u32) {
        let port = serialport::new(hw_config.port_1, baud).open().unwrap();
        check_baud_rate(port.as_ref(), baud);
    }

    #[apply(non_standard_baud_rates)]
    #[cfg_attr(
        any(
            feature = "ignore-hardware-tests",
            not(all(target_os = "linux", target_env = "musl")),
        ),
        ignore
    )]
    fn test_non_standard_baud_rate_fails_where_not_supported(
        hw_config: HardwareConfig,
        #[case] baud: u32,
    ) {
        assert!(serialport::new(hw_config.port_1, baud).open().is_err());
    }

    #[apply(non_standard_baud_rates)]
    #[cfg_attr(
        any(
            feature = "ignore-hardware-tests",
            all(target_os = "linux", target_env = "musl"),
        ),
        ignore
    )]
    fn test_non_standard_baud_rate_succeeds_where_supported(
        hw_config: HardwareConfig,
        #[case] baud: u32,
    ) {
        let port = serialport::new(hw_config.port_1, baud).open().unwrap();
        check_baud_rate(port.as_ref(), baud);
    }
}

/// Test cases for setting the baud rate via [`SerialPort::set_baud`].
mod set_baud {
    use super::*;

    #[apply(standard_baud_rates)]
    #[cfg_attr(feature = "ignore-hardware-tests", ignore)]
    fn test_standard_baud_rate(hw_config: HardwareConfig, #[case] baud: u32) {
        let mut port = serialport::new(hw_config.port_1, RESET_BAUD_RATE)
            .open()
            .unwrap();
        check_baud_rate(port.as_ref(), RESET_BAUD_RATE);

        port.set_baud_rate(baud).unwrap();
        check_baud_rate(port.as_ref(), baud);
    }

    #[apply(non_standard_baud_rates)]
    #[cfg_attr(
        any(
            feature = "ignore-hardware-tests",
            not(all(target_os = "linux", target_env = "musl")),
        ),
        ignore
    )]
    fn test_non_standard_baud_rate_fails_where_not_supported(
        hw_config: HardwareConfig,
        #[case] baud: u32,
    ) {
        let mut port = serialport::new(hw_config.port_1, RESET_BAUD_RATE)
            .open()
            .unwrap();
        check_baud_rate(port.as_ref(), RESET_BAUD_RATE);

        assert!(port.set_baud_rate(baud).is_err());
        check_baud_rate(port.as_ref(), RESET_BAUD_RATE);
    }

    #[apply(non_standard_baud_rates)]
    #[cfg_attr(
        any(
            feature = "ignore-hardware-tests",
            all(target_os = "linux", target_env = "musl"),
        ),
        ignore
    )]
    fn test_non_standard_baud_rate_succeeds_where_supported(
        hw_config: HardwareConfig,
        #[case] baud: u32,
    ) {
        let mut port = serialport::new(hw_config.port_1, RESET_BAUD_RATE)
            .open()
            .unwrap();
        check_baud_rate(port.as_ref(), RESET_BAUD_RATE);

        port.set_baud_rate(baud).unwrap();
        check_baud_rate(port.as_ref(), baud);
    }
}
