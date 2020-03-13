//! Opt-in support for embedded-hal traits.
//!
//! Can be enabled with the "embedded" cargo feature.

use std::io;

use embedded_hal::serial;

use crate::SerialPort;

fn io_error_to_nb(err: io::Error) -> nb::Error<io::ErrorKind> {
    match err.kind() {
        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut | io::ErrorKind::Interrupted => {
            nb::Error::WouldBlock
        }
        other => nb::Error::Other(other),
    }
}

impl serial::Read<u8> for Box<dyn SerialPort> {
    type Error = io::ErrorKind;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        let mut buffer = [0; 1];
        let bytes_read = io::Read::read(self, &mut buffer).map_err(io_error_to_nb)?;
        if bytes_read > 0 {
            Ok(buffer[0])
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl serial::Write<u8> for Box<dyn SerialPort> {
    type Error = io::ErrorKind;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        io::Write::write(self, &[word])
            .map_err(io_error_to_nb)
            .map(|_| ())
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        io::Write::flush(self).map_err(io_error_to_nb)
    }
}
