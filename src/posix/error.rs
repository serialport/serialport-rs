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
            nix::Error::EINVAL => Error::new(ErrorKind::InvalidInput, "Invalid input"),
            nix::Error::ENOTSUP => Error::new(ErrorKind::Unknown, "Unknown error"),
            nix::Error::ETIMEDOUT => Error::new(ErrorKind::Io(io::ErrorKind::TimedOut), e.desc()),
            nix::Error::ECONNABORTED => {
                Error::new(ErrorKind::Io(io::ErrorKind::ConnectionAborted), e.desc())
            }
            nix::Error::ECONNRESET => {
                Error::new(ErrorKind::Io(io::ErrorKind::ConnectionReset), e.desc())
            }
            nix::Error::ECONNREFUSED => {
                Error::new(ErrorKind::Io(io::ErrorKind::ConnectionRefused), e.desc())
            }
            nix::Error::ENOTCONN => {
                Error::new(ErrorKind::Io(io::ErrorKind::NotConnected), e.desc())
            }
            nix::Error::EADDRINUSE => Error::new(ErrorKind::Io(io::ErrorKind::AddrInUse), e.desc()),
            nix::Error::EADDRNOTAVAIL => {
                Error::new(ErrorKind::Io(io::ErrorKind::AddrNotAvailable), e.desc())
            }
            nix::Error::EAGAIN => Error::new(ErrorKind::Io(io::ErrorKind::WouldBlock), e.desc()),
            nix::Error::EINTR => Error::new(ErrorKind::Io(io::ErrorKind::Interrupted), e.desc()),
            nix::Error::EACCES => {
                Error::new(ErrorKind::Io(io::ErrorKind::PermissionDenied), e.desc())
            }
            nix::Error::ENOENT => Error::new(ErrorKind::Io(io::ErrorKind::NotFound), e.desc()),
            e => Error::new(ErrorKind::Unknown, e.desc()),
        }
    }
}
