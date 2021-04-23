#[cfg(unix)]
mod posix;

#[cfg(windows)]
mod windows;

#[cfg(not(any(unix, windows)))]
mod unsupported;

#[cfg(unix)]
pub use posix::*;

#[cfg(windows)]
pub use windows::*;

#[cfg(not(any(unix, windows)))]
pub use unsupported::*;