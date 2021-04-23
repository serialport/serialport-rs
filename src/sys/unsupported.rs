
use std::io;

use crate::{Error, ErrorKind, Result, SerialPortInfo};

pub fn available_ports() -> Result<Vec<SerialPortInfo>> {
    Err(Error::new(
        ErrorKind::Unknown,
        "available_ports() not implemented for platform",
    ))
}

/// Unsupported serial port type can never be constructed.
#[derive(Debug)]
pub enum SerialPort { }

impl SerialPort {
    pub fn open(builder: SerialPortBuilder, path: impl AsRef<Path>) -> Result<SerialPort> {
        Err(Error::new(
            ErrorKind::Unknown,
            "open() not implemented for platform",
        ))
    }

    pub fn name(&self) -> Option<&str> {
        unimplemented!()
    }

    pub fn timeout(&self) -> Duration {
        unimplemented!()
    }

    pub fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        unimplemented!()
    }

    pub fn write_request_to_send(&mut self, level: bool) -> Result<()> {
        unimplemented!()
    }

    pub fn write_data_terminal_ready(&mut self, level: bool) -> Result<()> {
       unimplemented!()
    }

    pub fn read_clear_to_send(&mut self) -> Result<bool> {
        unimplemented!()
    }

    pub fn read_data_set_ready(&mut self) -> Result<bool> {
        unimplemented!()
    }

    pub fn read_ring_indicator(&mut self) -> Result<bool> {
        unimplemented!()
    }

    pub fn read_carrier_detect(&mut self) -> Result<bool> {
        unimplemented!()
    }

    pub fn baud_rate(&self) -> Result<u32> {
        unimplemented!()
    }

    pub fn data_bits(&self) -> Result<DataBits> {
        unimplemented!()
    }

    pub fn parity(&self) -> Result<Parity> {
        unimplemented!()
    }

    pub fn stop_bits(&self) -> Result<StopBits> {
        unimplemented!()
    }

    pub fn flow_control(&self) -> Result<FlowControl> {
        unimplemented!()
    }

    pub fn set_baud_rate(&mut self, baud_rate: u32) -> Result<()> {
        unimplemented!()
    }

    pub fn set_data_bits(&mut self, data_bits: DataBits) -> Result<()> {
        unimplemented!()
    }

    pub fn set_parity(&mut self, parity: Parity) -> Result<()> {
        unimplemented!()
    }

    pub fn set_stop_bits(&mut self, stop_bits: StopBits) -> Result<()> {
        unimplemented!()
    }

    pub fn set_flow_control(&mut self, flow_control: FlowControl) -> Result<()> {
        unimplemented!()
    }

    pub fn bytes_to_read(&self) -> Result<u32> {
        unimplemented!()
    }

    pub fn bytes_to_write(&self) -> Result<u32> {
        unimplemented!()
    }

    pub fn clear(&self, buffer_to_clear: ClearBuffer) -> Result<()> {
        unimplemented!()
    }

    pub fn try_clone(&self) -> Result<Self> {
        unimplemented!()
    }

    pub fn set_break(&self) -> Result<()> {
        unimplemented!()
    }

    pub fn clear_break(&self) -> Result<()> {
        unimplemented!()
    }
}

impl io::Read for &SerialPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        unimplemented!()
    }
}

impl io::Write for &SerialPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        unimplemented!()
    }

    fn flush(&mut self) -> io::Result<()> {
        unimplemented!()
    }
}