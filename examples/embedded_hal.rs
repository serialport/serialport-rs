//! This example does not actually do anything, but it shows that a serialport
//! instance can be passed to functions / drivers that expect a type that
//! implements the embedded-hal traits.

use embedded_hal_nb::serial;

fn take_reader<R: serial::Read<u8>>(_r: &R) {
    // do nothing, but things should typecheck
}

fn take_writer<W: serial::Write<u8>>(_w: &W) {
    // do nothing, but things should typecheck
}

fn main() {
    let port = serialport::new("/dev/null", 9600)
        .open()
        .expect("This example isn't meant for running. It just demonstrates compatibility with embedded-hal on a type level.");
    take_reader(&port);
    take_writer(&port);
}
