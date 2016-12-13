extern crate libc;
#[cfg(target_os = "linux")]
extern crate libudev;

use std::error::Error;
use std::ffi::CStr;
use std::io;
use std::str;

use self::libc::{c_int,c_char,size_t};

pub fn last_os_error() -> ::Error {
    from_raw_os_error(errno())
}

pub fn from_raw_os_error(errno: i32) -> ::Error {
    use self::libc::{EBUSY,EISDIR,ELOOP,ENOTDIR,ENOENT,ENODEV,ENXIO,EACCES,EINVAL,ENAMETOOLONG,EINTR,EWOULDBLOCK};

    let kind = match errno {
        EBUSY | EISDIR | ELOOP | ENOTDIR | ENOENT | ENODEV | ENXIO | EACCES => ::ErrorKind::NoDevice,
        EINVAL | ENAMETOOLONG => ::ErrorKind::InvalidInput,

        EINTR => ::ErrorKind::Io(io::ErrorKind::Interrupted),
        EWOULDBLOCK => ::ErrorKind::Io(io::ErrorKind::WouldBlock),
        _ => ::ErrorKind::Io(io::ErrorKind::Other)
    };

    ::Error::new(kind, error_string(errno))
}

pub fn from_io_error(io_error: io::Error) -> ::Error {
    match io_error.raw_os_error() {
        Some(errno) => from_raw_os_error(errno),
        None => {
            let description = io_error.description().to_string();

            ::Error::new(::ErrorKind::Io(io_error.kind()), description)
        }
    }
}

#[cfg(target_os = "linux")]
impl From<libudev::Error> for ::Error {
    fn from(e: libudev::Error) -> ::Error {
        let description = e.description().to_string();
        match e.kind() {
            libudev::ErrorKind::NoMem => ::Error::new(::ErrorKind::Unknown, description),
            libudev::ErrorKind::InvalidInput => ::Error::new(::ErrorKind::InvalidInput, description),
            libudev::ErrorKind::Io(a) => ::Error::new(::ErrorKind::Io(a), description)
        }
    }
}

// the rest of this module is borrowed from libstd

const TMPBUF_SZ: usize = 128;

pub fn errno() -> i32 {
    #[cfg(any(target_os = "macos",
              target_os = "ios",
              target_os = "freebsd"))]
    unsafe fn errno_location() -> *const c_int {
        extern { fn __error() -> *const c_int; }
        __error()
    }

    #[cfg(target_os = "bitrig")]
    fn errno_location() -> *const c_int {
        extern {
            fn __errno() -> *const c_int;
        }
        unsafe {
            __errno()
        }
    }

    #[cfg(target_os = "dragonfly")]
    unsafe fn errno_location() -> *const c_int {
        extern { fn __dfly_error() -> *const c_int; }
        __dfly_error()
    }

    #[cfg(target_os = "openbsd")]
    unsafe fn errno_location() -> *const c_int {
        extern { fn __errno() -> *const c_int; }
        __errno()
    }

    #[cfg(any(target_os = "linux", target_os = "android"))]
    unsafe fn errno_location() -> *const c_int {
        extern { fn __errno_location() -> *const c_int; }
        __errno_location()
    }

    unsafe {
        (*errno_location()) as i32
    }
}

pub fn error_string(errno: i32) -> String {
    #[cfg(target_os = "linux")]
    extern {
        #[link_name = "__xpg_strerror_r"]
        fn strerror_r(errnum: c_int, buf: *mut c_char,
                      buflen: size_t) -> c_int;
    }
    #[cfg(not(target_os = "linux"))]
    extern {
        fn strerror_r(errnum: c_int, buf: *mut c_char,
                      buflen: size_t) -> c_int;
    }

    let mut buf = [0 as c_char; TMPBUF_SZ];

    let p = buf.as_mut_ptr();
    unsafe {
        if strerror_r(errno as c_int, p, buf.len() as size_t) < 0 {
            panic!("strerror_r failure");
        }

        let p = p as *const _;
        str::from_utf8(CStr::from_ptr(p).to_bytes()).unwrap().to_string()
    }
}
