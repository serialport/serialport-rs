//! This example does not actually do anything, but it shows that a serialport
//! instance can be passed to functions / drivers that expect a type that
//! implements the embedded-hal traits.

fn take_nonblocking_reader<R: embedded_hal_nb::serial::Read<u8>>(_r: &R) {
    // do nothing, but things should typecheck
}

fn take_nonblocking_writer<W: embedded_hal_nb::serial::Write<u8>>(_w: &W) {
    // do nothing, but things should typecheck
}

fn take_blocking_writer<W: embedded_hal::serial::Write<u8>>(_w: &W) {
    // do nothing, but things should typecheck
}

fn main() {
    let port = serialport::new("/dev/null", 9600)
        .open()
        .expect("This example isn't meant for running. It just demonstrates compatibility with embedded-hal on a type level.");
    take_nonblocking_reader(&port);
    take_nonblocking_writer(&port);
    take_blocking_writer(&port);
}
