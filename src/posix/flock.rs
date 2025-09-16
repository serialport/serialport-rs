//! Internal convenience wrappers for using `flock`.

use crate::{Error, ErrorKind, Result};
use nix::fcntl::FlockArg;
use std::os::unix::prelude::*;

/// Convenience method to acquire an exclusive lock using flock
///
/// This is used ensure that no other applications are using the port. This requires that the other
/// application also uses flock, so this does not work with all applications.
pub(crate) fn lock_exclusive(fd: RawFd) -> Result<()> {
    nix::fcntl::flock(fd, FlockArg::LockExclusiveNonblock).map_err(|e| {
        if e == nix::errno::Errno::EWOULDBLOCK {
            Error::new(
                ErrorKind::NoDevice,
                "Unable to acquire exclusive lock on serial port",
            )
        } else {
            e.into()
        }
    })
}

/// Convenience method to acquire a shared lock using flock
///
/// With the shared lock, other applications are able to use the port as well, if they're also
/// using a shared lock, but this makes sure that they cannot acquire an exclusive lock while we're
/// using the port.
pub(crate) fn lock_shared(fd: RawFd) -> Result<()> {
    nix::fcntl::flock(fd, FlockArg::LockSharedNonblock).map_err(|e| {
        if e == nix::errno::Errno::EWOULDBLOCK {
            Error::new(
                ErrorKind::NoDevice,
                "Unable to acquire shared lock on serial port",
            )
        } else {
            e.into()
        }
    })
}
