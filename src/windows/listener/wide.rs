use std::{ffi::OsString, os::windows::ffi::OsStringExt};

/// Convert a u16 array into an OsString.
///
/// Safety: The u16 array must be null terminated
pub unsafe fn from_wide(ptr: *const u16) -> OsString {
    let mut seek = ptr;
    loop {
        if *seek == 0 {
            break;
        } else {
            seek = seek.add(1);
        }
    }
    let len = (seek as usize - ptr as usize) / std::mem::size_of::<u16>();
    OsString::from_wide(std::slice::from_raw_parts(ptr, len))
}

pub fn to_wide<O>(s: O) -> Vec<u16>
where
    O: Into<OsString>,
{
    use std::os::windows::prelude::*;
    s.into().encode_wide().chain(Some(0).into_iter()).collect()
}
