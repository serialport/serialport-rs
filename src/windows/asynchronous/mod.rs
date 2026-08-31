//! Async serial port backends for Windows.

#[cfg(all(feature = "async-io", feature = "tokio"))]
compile_error!("Features `async-io` and `tokio` are mutually exclusive. Enable only one.");

#[cfg(feature = "async-io")]
mod async_io;
#[cfg(feature = "async-io")]
pub use async_io::*;
