use std::fmt;
use std::future::{poll_fn, Future};
use std::io;
use std::pin::Pin;
use std::task::{Context, Poll};
use std::time::Duration;

use async_io::Async;
use blocking::{unblock, Task};
use futures_io::{AsyncRead, AsyncWrite};

use crate::{
    ClearBuffer, DataBits, Error, ErrorKind, FlowControl, Parity, Result, SerialPort,
    SerialPortBuilder, StopBits,
};

use super::tty::TTYPort;

/// An asynchronous serial port for Unix targets.
///
/// This type implements [`futures_io::AsyncRead`] and [`futures_io::AsyncWrite`]
/// on top of the crate's native [`TTYPort`].
pub struct AsyncSerialPort {
    tty: Option<Async<TTYPort>>,
    port_name: Option<String>,
    timeout: Duration,
    flush_task: Option<Task<io::Result<()>>>,
}

impl AsyncSerialPort {
    /// Opens an asynchronous serial port with the specified settings.
    pub fn open(builder: &SerialPortBuilder) -> Result<Self> {
        let tty = TTYPort::open_nonblocking(builder)?;
        let port_name = tty.name();
        let timeout = tty.timeout();
        let tty = Async::new_nonblocking(tty)?;

        Ok(Self {
            tty: Some(tty),
            port_name,
            timeout,
            flush_task: None,
        })
    }

    fn tty_ref(&self) -> Result<&TTYPort> {
        self.tty
            .as_ref()
            .map(Async::get_ref)
            .ok_or_else(|| Error::new(ErrorKind::NoDevice, "port is closed"))
    }

    fn async_tty(&self) -> io::Result<&Async<TTYPort>> {
        self.tty
            .as_ref()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotConnected, "port is closed"))
    }

    fn tty_mut(&mut self) -> Option<&mut TTYPort> {
        match self.tty.as_mut() {
            // SAFETY: The wrapped `TTYPort` stays registered with `async-io` for its entire
            // lifetime. Callers use this mutable access only for serial I/O and settings changes,
            // which preserve the underlying file descriptor rather than replacing or closing it.
            Some(tty) => unsafe { Some(tty.get_mut()) },
            None => None,
        }
    }

    fn close_tty(&mut self) {
        self.flush_task = None;
        self.tty.take();
    }

    fn start_flush(&mut self) -> io::Result<()> {
        if self.flush_task.is_some() {
            return Ok(());
        }

        let Some(tty) = self.tty.as_ref() else {
            return Err(io::Error::new(
                io::ErrorKind::NotConnected,
                "port is closed",
            ));
        };

        // `TTYPort::flush()` is a blocking `tcdrain()` call, so move it onto the
        // blocking executor and keep polling the spawned task until completion.
        let mut cloned = tty.get_ref().try_clone_native().map_err(io::Error::from)?;
        self.flush_task = Some(unblock(move || std::io::Write::flush(&mut cloned)));
        Ok(())
    }

    fn poll_flush_task(
        &mut self,
        cx: &mut Context<'_>,
        closed_is_ok: bool,
    ) -> Poll<io::Result<()>> {
        if self.flush_task.is_none() {
            match self.start_flush() {
                Ok(()) => {}
                Err(err) if closed_is_ok && err.kind() == io::ErrorKind::NotConnected => {
                    return Poll::Ready(Ok(()));
                }
                Err(err) => return Poll::Ready(Err(err)),
            }
        }

        // Once the task completes, clear it so future flush/drain calls can start a new one.
        let poll = Pin::new(self.flush_task.as_mut().expect("flush task missing")).poll(cx);
        if poll.is_ready() {
            self.flush_task = None;
        }

        poll
    }

    /// Returns the name of this port if it exists.
    pub fn name(&self) -> Option<String> {
        self.port_name.clone()
    }

    /// Returns the current baud rate.
    pub fn baud_rate(&self) -> Result<u32> {
        self.tty_ref()?.baud_rate()
    }

    /// Returns the character size.
    pub fn data_bits(&self) -> Result<DataBits> {
        self.tty_ref()?.data_bits()
    }

    /// Returns the flow control mode.
    pub fn flow_control(&self) -> Result<FlowControl> {
        self.tty_ref()?.flow_control()
    }

    /// Returns the parity-checking mode.
    pub fn parity(&self) -> Result<Parity> {
        self.tty_ref()?.parity()
    }

    /// Returns the number of stop bits.
    pub fn stop_bits(&self) -> Result<StopBits> {
        self.tty_ref()?.stop_bits()
    }

    /// Returns the configured timeout.
    ///
    /// Async reads and writes do not use this value directly. It is retained to
    /// match the native serial API, and blocking operations delegated to the
    /// underlying port such as `flush()` still observe it.
    pub fn timeout(&self) -> Duration {
        self.timeout
    }

    /// Sets the baud rate.
    pub fn set_baud_rate(&mut self, baud_rate: u32) -> Result<()> {
        self.tty_mut()
            .ok_or_else(|| Error::new(ErrorKind::NoDevice, "port is closed"))?
            .set_baud_rate(baud_rate)
    }

    /// Sets the character size.
    pub fn set_data_bits(&mut self, data_bits: DataBits) -> Result<()> {
        self.tty_mut()
            .ok_or_else(|| Error::new(ErrorKind::NoDevice, "port is closed"))?
            .set_data_bits(data_bits)
    }

    /// Sets the flow control mode.
    pub fn set_flow_control(&mut self, flow_control: FlowControl) -> Result<()> {
        self.tty_mut()
            .ok_or_else(|| Error::new(ErrorKind::NoDevice, "port is closed"))?
            .set_flow_control(flow_control)
    }

    /// Sets the parity mode.
    pub fn set_parity(&mut self, parity: Parity) -> Result<()> {
        self.tty_mut()
            .ok_or_else(|| Error::new(ErrorKind::NoDevice, "port is closed"))?
            .set_parity(parity)
    }

    /// Sets the stop bits.
    pub fn set_stop_bits(&mut self, stop_bits: StopBits) -> Result<()> {
        self.tty_mut()
            .ok_or_else(|| Error::new(ErrorKind::NoDevice, "port is closed"))?
            .set_stop_bits(stop_bits)
    }

    /// Sets the underlying serial port timeout.
    pub fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        self.tty_mut()
            .ok_or_else(|| Error::new(ErrorKind::NoDevice, "port is closed"))?
            .set_timeout(timeout)?;
        self.timeout = timeout;
        Ok(())
    }

    /// Sets the state of the RTS control signal.
    pub fn write_request_to_send(&mut self, level: bool) -> Result<()> {
        self.tty_mut()
            .ok_or_else(|| Error::new(ErrorKind::NoDevice, "port is closed"))?
            .write_request_to_send(level)
    }

    /// Sets the state of the DTR control signal.
    pub fn write_data_terminal_ready(&mut self, level: bool) -> Result<()> {
        self.tty_mut()
            .ok_or_else(|| Error::new(ErrorKind::NoDevice, "port is closed"))?
            .write_data_terminal_ready(level)
    }

    /// Reads the state of the CTS control signal.
    pub fn read_clear_to_send(&mut self) -> Result<bool> {
        self.tty_mut()
            .ok_or_else(|| Error::new(ErrorKind::NoDevice, "port is closed"))?
            .read_clear_to_send()
    }

    /// Reads the state of the DSR control signal.
    pub fn read_data_set_ready(&mut self) -> Result<bool> {
        self.tty_mut()
            .ok_or_else(|| Error::new(ErrorKind::NoDevice, "port is closed"))?
            .read_data_set_ready()
    }

    /// Reads the state of the ring indicator signal.
    pub fn read_ring_indicator(&mut self) -> Result<bool> {
        self.tty_mut()
            .ok_or_else(|| Error::new(ErrorKind::NoDevice, "port is closed"))?
            .read_ring_indicator()
    }

    /// Reads the state of the carrier detect signal.
    pub fn read_carrier_detect(&mut self) -> Result<bool> {
        self.tty_mut()
            .ok_or_else(|| Error::new(ErrorKind::NoDevice, "port is closed"))?
            .read_carrier_detect()
    }

    /// Gets the number of bytes available in the input buffer.
    pub fn bytes_to_read(&self) -> Result<u32> {
        self.tty_ref()?.bytes_to_read()
    }

    /// Gets the number of bytes queued for transmission.
    pub fn bytes_to_write(&self) -> Result<u32> {
        self.tty_ref()?.bytes_to_write()
    }

    /// Discards data from the serial driver's input and/or output buffers.
    pub fn clear(&self, buffer_to_clear: ClearBuffer) -> Result<()> {
        self.tty_ref()?.clear(buffer_to_clear)
    }

    /// Starts transmitting a break condition.
    pub fn set_break(&self) -> Result<()> {
        self.tty_ref()?.set_break()
    }

    /// Stops transmitting a break condition.
    pub fn clear_break(&self) -> Result<()> {
        self.tty_ref()?.clear_break()
    }

    /// Waits until all buffered output has been transmitted.
    ///
    /// This uses the same underlying drain path as `flush()`, but exposes it as
    /// an explicit async method.
    pub async fn drain(&mut self) -> io::Result<()> {
        poll_fn(|cx| self.poll_flush_task(cx, false)).await
    }
}

impl AsyncRead for AsyncSerialPort {
    fn poll_read(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &mut [u8],
    ) -> Poll<io::Result<usize>> {
        if buf.is_empty() {
            return Poll::Ready(Ok(0));
        }

        let this = self.as_mut().get_mut();

        loop {
            let result = match this.tty_mut() {
                Some(tty) => tty.read_nonblocking(buf),
                None => {
                    return Poll::Ready(Err(io::Error::new(
                        io::ErrorKind::NotConnected,
                        "port is closed",
                    )))
                }
            };

            match result {
                Ok(len) => return Poll::Ready(Ok(len)),
                Err(err) if err.kind() == io::ErrorKind::Interrupted => continue,
                Err(err) if err.kind() == io::ErrorKind::WouldBlock => {
                    // Retry immediately if readiness was delivered between the failed
                    // read attempt and our registration with the reactor.
                    if this.async_tty()?.poll_readable(cx).is_ready() {
                        continue;
                    }

                    return Poll::Pending;
                }
                Err(err) => return Poll::Ready(Err(err)),
            }
        }
    }
}

impl AsyncWrite for AsyncSerialPort {
    fn poll_write(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &[u8],
    ) -> Poll<io::Result<usize>> {
        if buf.is_empty() {
            return Poll::Ready(Ok(0));
        }

        let this = self.as_mut().get_mut();

        loop {
            let result = match this.tty_mut() {
                Some(tty) => tty.write_nonblocking(buf),
                None => {
                    return Poll::Ready(Err(io::Error::new(
                        io::ErrorKind::NotConnected,
                        "port is closed",
                    )))
                }
            };

            match result {
                Ok(len) => return Poll::Ready(Ok(len)),
                Err(err) if err.kind() == io::ErrorKind::Interrupted => continue,
                Err(err) if err.kind() == io::ErrorKind::WouldBlock => {
                    // Mirror the read side: once writable readiness is observed, try
                    // the write again instead of yielding a spurious pending state.
                    if this.async_tty()?.poll_writable(cx).is_ready() {
                        continue;
                    }

                    return Poll::Pending;
                }
                Err(err) => return Poll::Ready(Err(err)),
            }
        }
    }

    fn poll_flush(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<io::Result<()>> {
        self.as_mut().get_mut().poll_flush_task(cx, true)
    }

    fn poll_close(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<io::Result<()>> {
        match self.as_mut().get_mut().poll_flush_task(cx, true) {
            Poll::Ready(result) => {
                // After close completes, drop the registered port so follow-up
                // operations report a closed/not-connected state.
                self.as_mut().get_mut().close_tty();
                Poll::Ready(result)
            }
            Poll::Pending => Poll::Pending,
        }
    }
}

impl fmt::Debug for AsyncSerialPort {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("AsyncSerialPort")
            .field("name", &self.name())
            .field("baud_rate", &self.baud_rate())
            .field("data_bits", &self.data_bits())
            .field("flow_control", &self.flow_control())
            .field("parity", &self.parity())
            .field("stop_bits", &self.stop_bits())
            .finish()
    }
}
