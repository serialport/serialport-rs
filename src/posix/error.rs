use std::io;

use crate::{Error, ErrorKind};

#[cfg(all(target_os = "linux", not(target_env = "musl"), feature = "libudev"))]
impl From<libudev::Error> for Error {
    fn from(e: libudev::Error) -> Error {
        use libudev::ErrorKind as K;
        let kind = match e.kind() {
            K::NoMem => ErrorKind::Unknown,
            K::InvalidInput => ErrorKind::InvalidInput,
            K::Io(a) => ErrorKind::Io(a),
        };
        Error::new(kind, e.description())
    }
}

impl From<nix::Error> for Error {
    fn from(e: nix::Error) -> Error {
        use io::ErrorKind as IO;
        use nix::errno::Errno as E;
        use ErrorKind as K;
        let kind = match e {
            E::ETIMEDOUT => K::Io(IO::TimedOut),
            E::ECONNABORTED => K::Io(IO::ConnectionAborted),
            E::ECONNRESET => K::Io(IO::ConnectionReset),
            E::ECONNREFUSED => K::Io(IO::ConnectionRefused),
            E::ENOTCONN => K::Io(IO::NotConnected),
            E::EADDRINUSE => K::Io(IO::AddrInUse),
            E::EADDRNOTAVAIL => K::Io(IO::AddrNotAvailable),
            E::EAGAIN => K::Io(IO::WouldBlock),
            E::EINTR => K::Io(IO::Interrupted),
            E::EACCES => K::Io(IO::PermissionDenied),
            E::ENOENT => K::Io(IO::NotFound),
            _ => K::Unknown,
        };
        Error::new(kind, e.desc())
    }
}
