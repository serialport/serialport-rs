//! Platform-agnostic plumbing shared by all async backends.
//!
//! Implement [`AsyncBackend`] to get a blanket [`AsyncSerialPortExt`] impl for free.

use std::fmt;
use std::time::Duration;

use crate::{
    AsyncSerialPortExt, ClearBuffer, DataBits, Error, ErrorKind, FlowControl, Parity, Result,
    SerialPort, StopBits,
};

pub(crate) struct PortInfo {
    pub(crate) port_name: Option<String>,
    pub(crate) timeout: Duration,
}

impl PortInfo {
    pub(crate) fn new(port: &dyn SerialPort) -> Self {
        Self {
            port_name: port.name(),
            timeout: port.timeout(),
        }
    }
}

/// Access to the underlying platform port and its cached info.
///
/// Implementors get a blanket [`AsyncSerialPortExt`] for free.
pub(crate) trait AsyncBackend: Send {
    /// Returns a reference to the cached port info.
    fn port_info(&self) -> &PortInfo;

    /// Returns a mutable reference to the cached port info.
    fn port_info_mut(&mut self) -> &mut PortInfo;

    /// Returns a reference to the underlying serial port.
    ///
    /// Fails with [`ErrorKind::NoDevice`] if the port has been closed.
    fn port_ref(&self) -> Result<&dyn SerialPort>;

    /// Returns a mutable reference to the underlying serial port,
    /// or `None` if it has been closed.
    fn port_mut(&mut self) -> Option<&mut dyn SerialPort>;
}

impl<T: AsyncBackend> AsyncSerialPortExt for T {
    fn name(&self) -> Option<String> {
        self.port_info().port_name.clone()
    }

    fn baud_rate(&self) -> Result<u32> {
        self.port_ref()?.baud_rate()
    }

    fn data_bits(&self) -> Result<DataBits> {
        self.port_ref()?.data_bits()
    }

    fn flow_control(&self) -> Result<FlowControl> {
        self.port_ref()?.flow_control()
    }

    fn parity(&self) -> Result<Parity> {
        self.port_ref()?.parity()
    }

    fn stop_bits(&self) -> Result<StopBits> {
        self.port_ref()?.stop_bits()
    }

    fn timeout(&self) -> Duration {
        self.port_info().timeout
    }

    fn set_baud_rate(&mut self, baud_rate: u32) -> Result<()> {
        self.with_port_mut(|p| p.set_baud_rate(baud_rate))
    }

    fn set_data_bits(&mut self, data_bits: DataBits) -> Result<()> {
        self.with_port_mut(|p| p.set_data_bits(data_bits))
    }

    fn set_flow_control(&mut self, flow_control: FlowControl) -> Result<()> {
        self.with_port_mut(|p| p.set_flow_control(flow_control))
    }

    fn set_parity(&mut self, parity: Parity) -> Result<()> {
        self.with_port_mut(|p| p.set_parity(parity))
    }

    fn set_stop_bits(&mut self, stop_bits: StopBits) -> Result<()> {
        self.with_port_mut(|p| p.set_stop_bits(stop_bits))
    }

    fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        self.with_port_mut(|p| p.set_timeout(timeout))?;
        self.port_info_mut().timeout = timeout;
        Ok(())
    }

    fn write_request_to_send(&mut self, level: bool) -> Result<()> {
        self.with_port_mut(|p| p.write_request_to_send(level))
    }

    fn write_data_terminal_ready(&mut self, level: bool) -> Result<()> {
        self.with_port_mut(|p| p.write_data_terminal_ready(level))
    }

    fn read_clear_to_send(&mut self) -> Result<bool> {
        self.with_port_mut(|p| p.read_clear_to_send())
    }

    fn read_data_set_ready(&mut self) -> Result<bool> {
        self.with_port_mut(|p| p.read_data_set_ready())
    }

    fn read_ring_indicator(&mut self) -> Result<bool> {
        self.with_port_mut(|p| p.read_ring_indicator())
    }

    fn read_carrier_detect(&mut self) -> Result<bool> {
        self.with_port_mut(|p| p.read_carrier_detect())
    }

    fn bytes_to_read(&self) -> Result<u32> {
        self.port_ref()?.bytes_to_read()
    }

    fn bytes_to_write(&self) -> Result<u32> {
        self.port_ref()?.bytes_to_write()
    }

    fn clear(&self, buffer_to_clear: ClearBuffer) -> Result<()> {
        self.port_ref()?.clear(buffer_to_clear)
    }

    fn set_break(&self) -> Result<()> {
        self.port_ref()?.set_break()
    }

    fn clear_break(&self) -> Result<()> {
        self.port_ref()?.clear_break()
    }
}

/// Helper to reduce boilerplate in the blanket impl above.
trait AsyncBackendExt: AsyncBackend {
    fn with_port_mut<R>(&mut self, f: impl FnOnce(&mut dyn SerialPort) -> Result<R>) -> Result<R> {
        let port = self
            .port_mut()
            .ok_or_else(|| Error::new(ErrorKind::NoDevice, "port is closed"))?;
        f(port)
    }
}

impl<T: AsyncBackend> AsyncBackendExt for T {}

/// Shared [`fmt::Debug`] output for all async backends.
pub(crate) fn debug_fmt(
    backend: &dyn AsyncBackend,
    type_name: &str,
    f: &mut fmt::Formatter<'_>,
) -> fmt::Result {
    let mut d = f.debug_struct(type_name);
    d.field("name", &backend.port_info().port_name);
    if let Ok(port) = backend.port_ref() {
        d.field("baud_rate", &port.baud_rate());
        d.field("data_bits", &port.data_bits());
        d.field("flow_control", &port.flow_control());
        d.field("parity", &port.parity());
        d.field("stop_bits", &port.stop_bits());
    }
    d.finish()
}
