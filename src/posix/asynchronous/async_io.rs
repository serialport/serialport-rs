//! async-io backend for async serial ports on Unix.

use std::fmt;
use std::future::Future;
use std::io;
use std::pin::Pin;
use std::task::{Context, Poll};

use async_io::Async;
use blocking::{unblock, Task};
use futures_io::{AsyncRead, AsyncWrite};

use crate::async_core::{self, AsyncBackend, PortInfo};
use crate::{Error, ErrorKind, Result, SerialPort, SerialPortBuilder};

use super::super::tty::TTYPort;

/// An asynchronous serial port for Unix targets.
///
/// This type implements [`futures_io::AsyncRead`] and [`futures_io::AsyncWrite`]
/// on top of the crate's native [`TTYPort`].
pub struct AsyncSerialPort {
    /// [`AsyncWrite::poll_close`] needs to drain the kernel buffer before dropping.
    /// Since `poll_close` takes `&mut self`, the executor can re-poll on `Pending`.
    /// Once the drain finishes we `.take()` the inner port and drop it.
    tty: Option<Async<TTYPort>>,
    info: PortInfo,
    flush_task: Option<Task<io::Result<()>>>,
}

impl AsyncSerialPort {
    /// Opens an asynchronous serial port with the given settings.
    pub fn open(builder: &SerialPortBuilder) -> Result<Self> {
        let tty = TTYPort::open_nonblocking(builder)?;
        Self::from_tty(tty)
    }

    /// Wraps an existing non-blocking [`TTYPort`].
    pub fn from_tty(tty: TTYPort) -> Result<Self> {
        let info = PortInfo::new(&tty);
        let async_tty = Async::new_nonblocking(tty)?;
        Ok(Self {
            tty: Some(async_tty),
            info,
            flush_task: None,
        })
    }

    fn async_tty(&self) -> io::Result<&Async<TTYPort>> {
        self.tty
            .as_ref()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotConnected, "port is closed"))
    }

    fn tty_mut(&mut self) -> Option<&mut TTYPort> {
        self.tty.as_mut().map(|a| {
            // SAFETY: `Async::get_mut` requires the inner fd to not be closed or replaced.
            // The fd is dropped via `.take()` in `close()` which sets it to `None`.
            unsafe { a.get_mut() }
        })
    }

    fn close(&mut self) {
        self.flush_task = None;
        self.tty.take();
    }

    fn start_flush(&mut self) -> io::Result<()> {
        if self.flush_task.is_some() {
            return Ok(());
        }

        let tty = self
            .tty
            .as_ref()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotConnected, "port is closed"))?;

        // `tcdrain()` blocks, so clone the fd and run it on the `blocking` thread pool.
        let mut cloned = tty.get_ref().try_clone_native().map_err(io::Error::from)?;
        self.flush_task = Some(unblock(move || std::io::Write::flush(&mut cloned)));
        Ok(())
    }

    fn poll_flush_impl(
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

        let task = self
            .flush_task
            .as_mut()
            .expect("flush task was just created");

        let poll = Pin::new(task).poll(cx);

        if poll.is_ready() {
            self.flush_task = None;
        }

        poll
    }
}

impl AsyncBackend for AsyncSerialPort {
    fn port_info(&self) -> &PortInfo {
        &self.info
    }

    fn port_info_mut(&mut self) -> &mut PortInfo {
        &mut self.info
    }

    fn port_ref(&self) -> Result<&dyn SerialPort> {
        self.tty
            .as_ref()
            .map(|a| a.get_ref() as &dyn SerialPort)
            .ok_or_else(|| Error::new(ErrorKind::NoDevice, "port is closed"))
    }

    fn port_mut(&mut self) -> Option<&mut dyn SerialPort> {
        self.tty_mut().map(|p| p as &mut dyn SerialPort)
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
        self.as_mut().get_mut().poll_flush_impl(cx, true)
    }

    fn poll_close(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<io::Result<()>> {
        let this = self.as_mut().get_mut();
        match this.poll_flush_impl(cx, true) {
            Poll::Ready(result) => {
                this.close();
                Poll::Ready(result)
            }
            Poll::Pending => Poll::Pending,
        }
    }
}

impl fmt::Debug for AsyncSerialPort {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        async_core::debug_fmt(self, "AsyncSerialPort", f)
    }
}
