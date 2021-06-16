use std::io;

use crate::{Error, ErrorKind};

#[cfg(all(target_os = "linux", not(target_env = "musl"), feature = "libudev"))]
impl From<libudev::Error> for Error {
    fn from(e: libudev::Error) -> Error {
        let description = e.description().to_string();
        match e.kind() {
            libudev::ErrorKind::NoMem => Error::new(ErrorKind::Unknown, description),
            libudev::ErrorKind::InvalidInput => Error::new(ErrorKind::InvalidInput, description),
            libudev::ErrorKind::Io(a) => Error::new(ErrorKind::Io(a), description),
        }
    }
}

impl From<nix::Error> for Error {
    fn from(e: nix::Error) -> Error {
        match e {
            nix::Error::InvalidPath | nix::Error::InvalidUtf8 => {
                Error::new(ErrorKind::InvalidInput, "Invalid input")
            }
            nix::Error::UnsupportedOperation => Error::new(ErrorKind::Unknown, "Unknown error"),
            nix::Error::Sys(e @ nix::errno::Errno::ETIMEDOUT) => {
                Error::new(ErrorKind::Io(io::ErrorKind::TimedOut), e.desc())
            }
            nix::Error::Sys(e @ nix::errno::Errno::ECONNABORTED) => {
                Error::new(ErrorKind::Io(io::ErrorKind::ConnectionAborted), e.desc())
            }
            nix::Error::Sys(e @ nix::errno::Errno::ECONNRESET) => {
                Error::new(ErrorKind::Io(io::ErrorKind::ConnectionReset), e.desc())
            }
            nix::Error::Sys(e @ nix::errno::Errno::ECONNREFUSED) => {
                Error::new(ErrorKind::Io(io::ErrorKind::ConnectionRefused), e.desc())
            }
            nix::Error::Sys(e @ nix::errno::Errno::ENOTCONN) => {
                Error::new(ErrorKind::Io(io::ErrorKind::NotConnected), e.desc())
            }
            nix::Error::Sys(e @ nix::errno::Errno::EADDRINUSE) => {
                Error::new(ErrorKind::Io(io::ErrorKind::AddrInUse), e.desc())
            }
            nix::Error::Sys(e @ nix::errno::Errno::EADDRNOTAVAIL) => {
                Error::new(ErrorKind::Io(io::ErrorKind::AddrNotAvailable), e.desc())
            }
            nix::Error::Sys(e @ nix::errno::Errno::EAGAIN) => {
                Error::new(ErrorKind::Io(io::ErrorKind::WouldBlock), e.desc())
            }
            nix::Error::Sys(e @ nix::errno::Errno::EINTR) => {
                Error::new(ErrorKind::Io(io::ErrorKind::Interrupted), e.desc())
            }
            nix::Error::Sys(e @ nix::errno::Errno::EACCES) => {
                Error::new(ErrorKind::Io(io::ErrorKind::PermissionDenied), e.desc())
            }
            nix::Error::Sys(e @ nix::errno::Errno::ENOENT) => {
                Error::new(ErrorKind::Io(io::ErrorKind::NotFound), e.desc())
            }
            nix::Error::Sys(e) => Error::new(ErrorKind::Unknown, e.desc()),
        }
    }
}
