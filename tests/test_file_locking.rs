mod config;

use config::{hw_config, HardwareConfig};
use nix::fcntl::FlockArg;
use nix::ioctl_none_bad;
use rstest::rstest;
use serialport::ErrorKind;
use std::fs::File;
use std::os::fd::AsRawFd;

// Locally create a wrapper for the TIOCEXCL ioctl.
ioctl_none_bad!(tiocexcl, libc::TIOCEXCL);

#[rstest]
#[cfg_attr(not(feature = "hardware-tests"), ignore)]
fn opening_multiple_times(hw_config: HardwareConfig) {
    // Try to open (and close) the same port multiple times in a row to check that acquiring and
    // releasing loks does not lock out ourselves.
    for _ in 0..3 {
        // The port gets immediately dropped and therefor closed immediately.
        serialport::new(&hw_config.port_1, 115200).open().unwrap();
    }
}

#[rstest]
#[cfg_attr(not(all(unix, feature = "hardware-tests")), ignore)]
fn second_open_fails_open(hw_config: HardwareConfig) {
    // Open the port for the first time with serialport. This is expected to put some locking in
    // place by default.
    let _first = serialport::new(&hw_config.port_1, 115200).open().unwrap();

    // Now try to open the same port for a second time and check that this fails.
    let second = serialport::new(hw_config.port_1, 115200).open();
    assert!(second.is_err());
    assert_eq!(second.unwrap_err().kind(), ErrorKind::NoDevice);
}

#[rstest]
#[cfg_attr(not(all(unix, feature = "hardware-tests")), ignore)]
fn second_open_fails_flock(hw_config: HardwareConfig) {
    // Open the port for the first time and apply an exclusive flock.
    let first = File::open(&hw_config.port_1).unwrap();
    let fd = first.as_raw_fd();
    nix::fcntl::flock(fd, FlockArg::LockExclusiveNonblock).unwrap();

    // Now try to open the same port for a second time. This is expected to fail.
    let second = serialport::new(&hw_config.port_1, 115200).open();
    assert!(second.is_err());
    assert_eq!(second.unwrap_err().kind(), ErrorKind::NoDevice);
}

#[rstest]
#[cfg_attr(not(all(unix, feature = "hardware-tests")), ignore)]
fn second_open_fails_lock(hw_config: HardwareConfig) {
    // Open the port for the first time and lock it exclusively with Rust's default file locking
    // mechanism which is expected to be flock.
    let first = File::open(&hw_config.port_1).unwrap();
    first.lock().unwrap();

    // Now try to open the same port for a second time. This is expected to fail.
    let second = serialport::new(&hw_config.port_1, 115200).open();
    assert!(second.is_err());
    assert_eq!(second.unwrap_err().kind(), ErrorKind::NoDevice);
}

#[rstest]
#[cfg_attr(not(all(unix, feature = "hardware-tests")), ignore)]
fn second_open_fails_tiocexcl(hw_config: HardwareConfig) {
    // Open the port for the first time and apply locking via TIOCEXL. This is one of the locking
    // mechanisms used by TTYPort.
    let first = File::open(&hw_config.port_1).unwrap();
    let fd = first.as_raw_fd();
    unsafe { tiocexcl(fd).unwrap() };

    // Now try to open the same port for a second time. This is expected to fail.
    let second = serialport::new(&hw_config.port_1, 115200).open();
    assert!(second.is_err());
    assert_eq!(second.unwrap_err().kind(), ErrorKind::NoDevice);
}
