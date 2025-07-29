//! guid

use super::wide::*;
use std::ffi::OsString;
use std::fmt::Debug;
use std::{error, fmt};
use windows_sys::Win32::System::Rpc::{UuidFromStringW, RPC_S_INVALID_STRING_UUID};

/// Simple error type when we fail to convert a string into a guid
#[derive(Debug)]
pub struct InvalidUuidString(Vec<u16>);
impl error::Error for InvalidUuidString {}
impl fmt::Display for InvalidUuidString {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "invalid uuid string {:?}", self.0)
    }
}

#[derive(Copy, Clone, Default)]
pub struct Guid(pub windows_sys::core::GUID);
impl Guid {
    /// Create a new Guid from an OsString. Will return an encoded wide version of the OsString on
    /// failure
    pub fn new<S>(s: S) -> Result<Self, InvalidUuidString>
    where
        S: Into<OsString>,
    {
        let uuid = to_wide(s);
        let mut me = unsafe { std::mem::zeroed() };
        let result = unsafe { UuidFromStringW(uuid.as_ptr(), &mut me) };
        match result {
            RPC_S_INVALID_STRING_UUID => Err(InvalidUuidString(uuid)),
            _ => Ok(Self(me)),
        }
    }

    /// Unwrap into the inner [`windows_sys::core::GUID`]
    pub fn into_inner(self) -> windows_sys::core::GUID {
        self.0
    }
}

impl Debug for Guid {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Guid")
            .field("data1", &self.0.data1)
            .field("data2", &self.0.data2)
            .field("data3", &self.0.data3)
            .field("data4", &self.0.data4)
            .finish()
    }
}

impl PartialEq for Guid {
    fn eq(&self, other: &Self) -> bool {
        self.0.data1 == other.0.data1
            && self.0.data2 == other.0.data2
            && self.0.data3 == other.0.data3
            && self.0.data4 == other.0.data4
    }
}

impl From<windows_sys::core::GUID> for Guid {
    fn from(value: windows_sys::core::GUID) -> Self {
        Self(value)
    }
}

impl From<Guid> for windows_sys::core::GUID {
    fn from(value: Guid) -> Self {
        value.0
    }
}

/// Initializes a `GUID` from literal values.
#[macro_export]
macro_rules! guid {
    (
        $a:expr,
        $b:expr,
        $c:expr,
        $d:expr
    ) => {
        windows_sys::core::GUID {
            data1: $a,
            data2: $b,
            data3: $c,
            data4: $d,
        }
    };

    (
        $a:expr,
        $b:expr,
        $c:expr,
        $d0:expr,
        $d1:expr,
        $d2:expr,
        $d3:expr,
        $d4:expr,
        $d5:expr,
        $d6:expr,
        $d7:expr
    ) => {
        windows_sys::core::GUID {
            data1: $a,
            data2: $b,
            data3: $c,
            data4: [$d0, $d1, $d2, $d3, $d4, $d5, $d6, $d7],
        }
    };
}
