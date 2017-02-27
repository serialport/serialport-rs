#![allow(non_camel_case_types,dead_code)]

use std::io;
use std::time::Duration;

use libc;
use nix;
use nix::poll::{EventFlags, POLLHUP, POLLIN, POLLNVAL, POLLOUT, PollFd};
use nix::sys::signal::SigSet;
use nix::sys::time::{TimeSpec, TimeValLike};

pub fn wait_read_fd(fd: libc::c_int, timeout: Duration) -> io::Result<()> {
    wait_fd(fd, POLLIN, timeout)
}

pub fn wait_write_fd(fd: libc::c_int, timeout: Duration) -> io::Result<()> {
    wait_fd(fd, POLLOUT, timeout)
}

fn wait_fd(fd: libc::c_int, events: EventFlags, timeout: Duration) -> io::Result<()> {
    use libc::{EINTR, EPIPE, EIO};

    let mut fds = vec![PollFd::new(fd, events, EventFlags::empty())];


    let milliseconds = timeout.as_secs() as i64 * 1000 + timeout.subsec_nanos() as i64 / 1_000_000;
    let timespec = TimeSpec::milliseconds(milliseconds);
    #[cfg(target_os = "linux")]
    let wait = nix::poll::ppoll(fds.as_mut_slice(), timespec, SigSet::empty())?;
    #[cfg(not(target_os = "linux"))]
    let wait = nix::poll::poll(fds.as_mut_slice(), milliseconds)?;

    // Check for any errors that occurred during polling
    match wait {
        // If it's less than 0,
        i if i < 0 => {
            let errno = nix::errno::errno();

            let kind = match errno {
                EINTR => io::ErrorKind::Interrupted,
                _ => io::ErrorKind::Other,
            };
            return Err(io::Error::new(kind, super::error::error_string(errno)));
        }
        0 => return Err(io::Error::new(io::ErrorKind::TimedOut, "Operation timed out")),
        _ => (),
    }

    // Check the result of ppoll() by looking at the revents field
    match fds[0].revents() {
        Some(e) if e == events => return Ok(()),
        // If there was a hangout or invalid request
        Some(e) if e.contains(POLLHUP) || e.contains(POLLNVAL) => {
            return Err(io::Error::new(io::ErrorKind::BrokenPipe,
                                      super::error::error_string(EPIPE)));
        }
        Some(_) | None => (),
    }

    Err(io::Error::new(io::ErrorKind::Other, super::error::error_string(EIO)))
}
