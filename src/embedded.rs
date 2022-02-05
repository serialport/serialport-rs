//! Opt-in support for embedded-hal traits.
//!
//! Can be enabled with the "embedded" cargo feature.

use std::io;

use embedded_hal::serial;

use crate::SerialPort;

#[derive(Debug, Copy, Clone)]
pub struct SerialError {
    kind: io::ErrorKind,
}

// Implement serial::Error for SerialError
impl serial::Error for SerialError {
    fn kind(&self) -> serial::ErrorKind {
        #[allow(clippy::match_single_binding)]
        match self.kind {
            _other => serial::ErrorKind::Other,
        }
    }
}

fn io_error_to_nb(err: io::Error) -> nb::Error<SerialError> {
    match err.kind() {
        io::ErrorKind::WouldBlock | io::ErrorKind::Interrupted => nb::Error::WouldBlock,
        other => nb::Error::Other(SerialError { kind: other }),
    }
}

impl serial::nb::Read<u8> for Box<dyn SerialPort> {
    type Error = SerialError;

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

impl serial::nb::Write<u8> for Box<dyn SerialPort> {
    type Error = SerialError;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        io::Write::write(self, &[word])
            .map_err(io_error_to_nb)
            .map(|_| ())
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        io::Write::flush(self).map_err(io_error_to_nb)
    }
}
