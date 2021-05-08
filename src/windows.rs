//! Provides windows-only extensions to the SerialPort type.

// Note: windows-specific things are imported under narrow scope because this
// mod is also compiled on unix when doing a `doc` build.

use std::convert::TryFrom;
use std::time::Duration;

#[cfg(windows)]
use winapi::shared::minwindef::DWORD;
#[cfg(windows)]
use winapi::um::winbase::COMMTIMEOUTS;

use crate::Result;

/// Represents COM Port Timeouts. Equivalent to
/// https://docs.microsoft.com/en-us/windows/win32/api/winbase/ns-winbase-commtimeouts.
///
/// See official documentation of the `COMMTIMEOUTS` C struct for details.
///
/// Timeouts are given as durations instead of integer milliseconds. When applied,
/// they will have at-most millisecond precision.
#[derive(Debug, Copy, Clone)]
pub struct CommTimeouts {
    read_interval_timeout: Duration,
    read_total_timeout_multiplier: Duration,
    read_total_timeout_constant: Duration,
    write_total_timeout_multiplier: Duration,
    write_total_timeout_constant: Duration,
}

#[cfg(windows)]
impl From<COMMTIMEOUTS> for CommTimeouts {
    fn from(timeouts: COMMTIMEOUTS) -> Self {
        CommTimeouts {
            read_interval_timeout: Duration::from_millis(timeouts.ReadIntervalTimeout as u64),
            read_total_timeout_multiplier: Duration::from_millis(
                timeouts.ReadTotalTimeoutMultiplier as u64,
            ),
            read_total_timeout_constant: Duration::from_millis(
                timeouts.ReadTotalTimeoutConstant as u64,
            ),
            write_total_timeout_multiplier: Duration::from_millis(
                timeouts.WriteTotalTimeoutMultiplier as u64,
            ),
            write_total_timeout_constant: Duration::from_millis(
                timeouts.WriteTotalTimeoutConstant as u64,
            ),
        }
    }
}

#[cfg(windows)]
impl From<CommTimeouts> for COMMTIMEOUTS {
    fn from(timeouts: CommTimeouts) -> Self {
        COMMTIMEOUTS {
            ReadIntervalTimeout: DWORD::try_from(timeouts.read_interval_timeout.as_millis())
                .unwrap_or(DWORD::MAX),
            ReadTotalTimeoutMultiplier: DWORD::try_from(
                timeouts.read_total_timeout_multiplier.as_millis(),
            )
            .unwrap_or(DWORD::MAX),
            ReadTotalTimeoutConstant: DWORD::try_from(
                timeouts.read_total_timeout_constant.as_millis(),
            )
            .unwrap_or(DWORD::MAX),
            WriteTotalTimeoutMultiplier: DWORD::try_from(
                timeouts.write_total_timeout_multiplier.as_millis(),
            )
            .unwrap_or(DWORD::MAX),
            WriteTotalTimeoutConstant: DWORD::try_from(
                timeouts.write_total_timeout_constant.as_millis(),
            )
            .unwrap_or(DWORD::MAX),
        }
    }
}

/// Windows-only extensions to the SerialPort type.
pub trait SerialPortExt {
    /// Gets the current timeouts set on the serial port. While the `read_timeout`
    /// and `write_timeout` methods only return the most recent value set through
    /// the `SerialPort` API, this will return the values actually seen by Windows.
    fn comm_timeouts(&self) -> Result<CommTimeouts>;

    /// Sets the timeouts used by the serialport. Unlike `set_read_timeout` and
    /// `set_write_timeout`, which are limited to simple posix-like behavior, this
    /// method allows access to the full range of available timeout settings allowed
    /// by windows.
    fn set_comm_timeouts(&self, timeouts: CommTimeouts) -> Result<()>;
}

impl SerialPortExt for crate::SerialPort {
    fn comm_timeouts(&self) -> Result<CommTimeouts> {
        self.0.comm_timeouts()
    }

    fn set_comm_timeouts(&self, timeouts: CommTimeouts) -> Result<()> {
        self.0.set_comm_timeouts(timeouts)
    }
}
