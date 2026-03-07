mod config;

use cfg_if::cfg_if;
use config::{hw_config, HardwareConfig};
use rstest::rstest;
use serialport::SerialPort;

cfg_if! {
    if #[cfg(unix)] {
        use std::os::unix::prelude::*;
        use nix::fcntl::FlockArg;
        use nix::ioctl_none_bad;
        use serialport::ErrorKind;
        use std::fs::File;

        // Locally create a wrapper for the TIOCEXCL and TIOCNXCL ioctl.
        ioctl_none_bad!(tiocexcl, libc::TIOCEXCL);
        ioctl_none_bad!(tiocnxcl, libc::TIOCNXCL);
    }
}

/// Abstract lock mode for making intent more clearly visible than with a `bool`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum LockMode {
    Exclusive,
    Shared,
}

impl LockMode {
    pub fn is_exclusive(&self) -> bool {
        match self {
            LockMode::Exclusive => true,
            LockMode::Shared => false,
        }
    }
}

#[cfg(unix)]
impl From<LockMode> for FlockArg {
    fn from(exclusivity: LockMode) -> FlockArg {
        match exclusivity {
            LockMode::Exclusive => FlockArg::LockExclusiveNonblock,
            LockMode::Shared => FlockArg::LockSharedNonblock,
        }
    }
}

mod checks {
    use super::*;

    #[must_use]
    pub fn open_file_successful(hw_config: &HardwareConfig) -> File {
        // Be gentle on macOS when opening a /dev/tty.* device: When opening in blocking mode it waits
        // for DCD and will likely stall the test. Our SerialPortBuilder::open already takes care of
        // that.
        File::options()
            .read(true)
            .write(true)
            .custom_flags(libc::O_NONBLOCK)
            .open(&hw_config.port_1)
            .unwrap()
    }

    pub fn open_port_fails(hw_config: &HardwareConfig, exclusivity: LockMode) {
        let port = serialport::new(&hw_config.port_1, 115200)
            .exclusive(exclusivity.is_exclusive())
            .open();
        assert!(port.is_err());
        assert_eq!(port.unwrap_err().kind(), ErrorKind::NoDevice);
    }

    #[must_use]
    pub fn open_port_successful(
        hw_config: &HardwareConfig,
        exclusivity: LockMode,
    ) -> Box<dyn SerialPort> {
        serialport::new(&hw_config.port_1, 115200)
            .exclusive(exclusivity.is_exclusive())
            .open()
            .unwrap()
    }
}

#[rstest]
#[cfg_attr(not(feature = "hardware-tests"), ignore)]
#[case(LockMode::Exclusive)]
#[case(LockMode::Shared)]
fn opening_multiple_times(hw_config: HardwareConfig, #[case] exclusivity: LockMode) {
    // Try to open (and close) the same port multiple times in a row to check that acquiring and
    // releasing loks does not lock out ourselves.
    for _ in 0..3 {
        // The port gets closed by dropping it before the next iteration step.
        let _ = checks::open_port_successful(&hw_config, exclusivity);
    }
}

mod second_exclusive_open {
    use super::*;

    #[rstest]
    #[cfg_attr(not(feature = "hardware-tests"), ignore)]
    #[case(LockMode::Exclusive)]
    #[case(LockMode::Shared)]
    fn fails_after_open(hw_config: HardwareConfig, #[case] first_mode: LockMode) {
        // Open the port for the first time and keep it open.
        let _first = checks::open_port_successful(&hw_config, first_mode);

        // Opening the same port exclusively for a second time is expected to fail regardless of
        // the previous locking mode.
        checks::open_port_fails(&hw_config, LockMode::Exclusive);
    }

    #[rstest]
    #[cfg_attr(not(feature = "hardware-tests"), ignore)]
    #[case(LockMode::Exclusive)]
    #[case(LockMode::Shared)]
    fn fails_after_lock(hw_config: HardwareConfig, #[case] first_mode: LockMode) {
        // Open the port for the first time, keep it open, and lock it with Rust's default locking
        // mechanism. Our locking is supposed to be compatible with that.
        let first = checks::open_file_successful(&hw_config);
        match first_mode {
            LockMode::Exclusive => first.lock().unwrap(),
            LockMode::Shared => first.lock_shared().unwrap(),
        }

        // Opening the same port exclusively for a second time is expected to fail regardless of
        // the previous mode.
        checks::open_port_fails(&hw_config, LockMode::Exclusive);
    }

    #[rstest]
    #[cfg(unix)]
    #[cfg_attr(not(feature = "hardware-tests"), ignore)]
    #[case(FlockArg::LockExclusiveNonblock)]
    #[case(FlockArg::LockSharedNonblock)]
    fn fails_after_flock(hw_config: HardwareConfig, #[case] first_mode: FlockArg) {
        // Open the port file for the first time, keep it open, and flock it in the specified mode.
        let first = checks::open_file_successful(&hw_config);
        let fd = first.as_raw_fd();
        nix::fcntl::flock(fd, first_mode).unwrap();

        // Opening the same port exclusively for a second time is expected to fail regardless of
        // the previous mode.
        checks::open_port_fails(&hw_config, LockMode::Exclusive);
    }

    #[rstest]
    #[cfg(unix)]
    #[cfg_attr(not(feature = "hardware-tests"), ignore)]
    fn fails_after_tiocexcl(hw_config: HardwareConfig) {
        // Open the port file for the first time, keep it open, and apply locking via TIOCEXL.
        let first = checks::open_file_successful(&hw_config);
        unsafe { tiocexcl(first.as_raw_fd()).unwrap() };

        // Opening a port locked with TIOCEXCL for a second time is expected to fail.
        checks::open_port_fails(&hw_config, LockMode::Exclusive);
    }

    #[rstest]
    #[cfg(unix)]
    #[cfg_attr(not(feature = "hardware-tests"), ignore)]
    fn succeeds_after_tiocnxcl(hw_config: HardwareConfig) {
        // Open the port file for the first time, keep it open, and unlock it via TIOCNXCL.
        let first = checks::open_file_successful(&hw_config);
        unsafe { tiocnxcl(first.as_raw_fd()).unwrap() };

        // Opening the same port again is expected to succeed as only TIOCEXCL prevents opening
        // this terminal again.
        let _second = checks::open_port_successful(&hw_config, LockMode::Exclusive);
    }
}

mod second_non_exclusive_open {
    use super::*;

    #[rstest]
    #[cfg_attr(not(feature = "hardware-tests"), ignore)]
    #[case(LockMode::Exclusive)]
    #[case(LockMode::Shared)]
    fn after_open(hw_config: HardwareConfig, #[case] first_mode: LockMode) {
        // Open the port for the first time and keep it open.
        let _first = checks::open_port_successful(&hw_config, first_mode);

        // Open the same port a second time.
        match first_mode {
            LockMode::Exclusive => checks::open_port_fails(&hw_config, LockMode::Shared),
            LockMode::Shared => {
                let _ = checks::open_port_successful(&hw_config, LockMode::Shared);
            }
        }
    }

    #[rstest]
    #[cfg_attr(not(feature = "hardware-tests"), ignore)]
    #[case(LockMode::Exclusive)]
    #[case(LockMode::Shared)]
    fn after_lock(hw_config: HardwareConfig, #[case] first_mode: LockMode) {
        // Open the port for the first time, keep it open, and lock it with Rust's default locking
        // mechanism (flock at the time of writing).
        let first = checks::open_file_successful(&hw_config);
        match first_mode {
            LockMode::Exclusive => first.lock().unwrap(),
            LockMode::Shared => first.lock_shared().unwrap(),
        }

        // Open the same port for a second time.
        match first_mode {
            LockMode::Exclusive => checks::open_port_fails(&hw_config, LockMode::Shared),
            LockMode::Shared => {
                let _ = checks::open_port_successful(&hw_config, LockMode::Shared);
            }
        }
    }

    #[rstest]
    #[cfg(unix)]
    #[cfg_attr(not(feature = "hardware-tests"), ignore)]
    #[case(LockMode::Exclusive)]
    #[case(LockMode::Shared)]
    fn after_flock(hw_config: HardwareConfig, #[case] first_mode: LockMode) {
        // Open the port file for the first time, keep it open, and flock it in the specified mode.
        let first = checks::open_file_successful(&hw_config);
        nix::fcntl::flock(first.as_raw_fd(), first_mode.into()).unwrap();

        // Open the same port for a second time.
        match first_mode {
            LockMode::Exclusive => checks::open_port_fails(&hw_config, LockMode::Shared),
            LockMode::Shared => {
                let _ = checks::open_port_successful(&hw_config, LockMode::Shared);
            }
        }
    }

    #[rstest]
    #[cfg(unix)]
    #[cfg_attr(not(feature = "hardware-tests"), ignore)]
    fn fails_after_tiocexcl(hw_config: HardwareConfig) {
        // Open the port file for the first time, keep it open, and apply locking via TIOCEXL.
        let first = checks::open_file_successful(&hw_config);
        unsafe { tiocexcl(first.as_raw_fd()).unwrap() };

        // Opening a port locked with TIOCEXCL for a second time is expected to fail.
        checks::open_port_fails(&hw_config, LockMode::Exclusive);
    }

    #[rstest]
    #[cfg(unix)]
    #[cfg_attr(not(feature = "hardware-tests"), ignore)]
    fn succeeds_after_tiocnxcl(hw_config: HardwareConfig) {
        // Open the port file for the first time, keep it open, and unlock it via TIOCNXCL.
        let first = checks::open_file_successful(&hw_config);
        unsafe { tiocnxcl(first.as_raw_fd()).unwrap() };

        // Opening the same port again is expected to succeed as only TIOCEXCL prevents opening
        // this terminal again.
        let _second = checks::open_port_successful(&hw_config, LockMode::Exclusive);
    }
}
