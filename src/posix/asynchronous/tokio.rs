//! Native `tokio` backend for async serial ports on Unix.

compile_error!(
    "The native `tokio` backend is not yet implemented. \
     Use the `async-io` feature instead, which also works \
     with tokio."
);
