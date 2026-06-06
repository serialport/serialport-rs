//! Internal convenience wrappers for using `flock`.

use crate::{Error, ErrorKind, Result};
use nix::errno::Errno;
use nix::fcntl::{Flock, FlockArg};
use std::os::unix::prelude::*;

/// Convenience method to acquire an exclusive lock using flock
///
/// This is used ensure that no other applications are using the port. This requires that the other
/// application also uses flock, so this does not work with all applications.
pub fn lock_exclusive(fd: OwnedFd) -> Result<Flock<OwnedFd>> {
    Flock::lock(fd, FlockArg::LockExclusiveNonblock).map_err(|(_, e)| convert_exclusive_error(e))
}

/// Convenience method to acquire a shared lock using flock
///
/// With the shared lock, other applications are able to use the port as well, if they're also
/// using a shared lock, but this makes sure that they cannot acquire an exclusive lock while we're
/// using the port.
pub fn lock_shared(fd: OwnedFd) -> Result<Flock<OwnedFd>> {
    Flock::lock(fd, FlockArg::LockSharedNonblock).map_err(|(_, e)| convert_shared_error(e))
}

/// Changes the flock to an exclusive lock.
pub fn relock_exclusive(fd: &Flock<OwnedFd>) -> Result<()> {
    fd.relock(FlockArg::LockExclusiveNonblock)
        .map_err(convert_exclusive_error)
}

/// Changes the flock to a shared lock.
pub fn relock_shared(fd: &Flock<OwnedFd>) -> Result<()> {
    fd.relock(FlockArg::LockSharedNonblock)
        .map_err(convert_exclusive_error)
}

fn convert_exclusive_error(e: Errno) -> Error {
    if e == Errno::EWOULDBLOCK {
        Error::new(
            ErrorKind::NoDevice,
            "Unable to acquire exclusive lock on serial port",
        )
    } else {
        e.into()
    }
}

fn convert_shared_error(e: Errno) -> Error {
    if e == Errno::EWOULDBLOCK {
        Error::new(
            ErrorKind::NoDevice,
            "Unable to acquire shared lock on serial port",
        )
    } else {
        e.into()
    }
}
