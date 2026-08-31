//! async-io feature backend for async serial ports on Windows.

use std::fmt;
use std::future::Future;
use std::io;
use std::mem;
use std::os::windows::io::{AsRawHandle, FromRawHandle, OwnedHandle, RawHandle};
use std::pin::Pin;
use std::ptr;
use std::task::{Context, Poll};

use async_io::os::windows::Waitable;
use blocking::{unblock, Task};
use futures_io::{AsyncRead, AsyncWrite};
use windows_sys::Win32::Devices::Communication::{SetCommTimeouts, COMMTIMEOUTS};
use windows_sys::Win32::Foundation::{ERROR_IO_INCOMPLETE, ERROR_IO_PENDING, FALSE, HANDLE, TRUE};
use windows_sys::Win32::System::Threading::CreateEventW;
use windows_sys::Win32::System::IO::{CancelIoEx, GetOverlappedResult, OVERLAPPED};

use crate::async_core::{self, AsyncBackend, PortInfo};
use crate::{Error, ErrorKind, Result, SerialPort, SerialPortBuilder};

use super::super::COMPort;

/// State for one in-flight overlapped read.
struct ReadOp {
    waitable: Waitable<OwnedHandle>,
    overlapped: Box<OVERLAPPED>,
    buffer: Vec<u8>,
}

// SAFETY: The event handle and OVERLAPPED storage stay valid while the OS may use them.
unsafe impl Send for ReadOp {}

impl ReadOp {
    /// Creates a read operation with its event, state, and buffer.
    fn new(capacity: usize) -> io::Result<Self> {
        let waitable = new_waitable_event()?;
        let overlapped = new_overlapped(waitable.as_raw_handle());
        Ok(Self {
            waitable,
            overlapped,
            buffer: vec![0_u8; capacity],
        })
    }

    fn overlapped_ptr(&mut self) -> *mut OVERLAPPED {
        self.overlapped.as_mut() as *mut OVERLAPPED
    }
}

impl WriteOp {
    /// Creates a write operation with its event and owned buffer.
    fn new(buffer: Vec<u8>) -> io::Result<Self> {
        let waitable = new_waitable_event()?;
        let overlapped = new_overlapped(waitable.as_raw_handle());
        Ok(Self {
            waitable,
            overlapped,
            buffer,
        })
    }

    fn overlapped_ptr(&mut self) -> *mut OVERLAPPED {
        self.overlapped.as_mut() as *mut OVERLAPPED
    }
}

/// State for one in-flight overlapped write.
struct WriteOp {
    waitable: Waitable<OwnedHandle>,
    overlapped: Box<OVERLAPPED>,
    buffer: Vec<u8>,
}

// SAFETY: Same reason as `ReadOp`.
unsafe impl Send for WriteOp {}

/// An asynchronous serial port for Windows targets.
pub struct AsyncSerialPort {
    com: Option<COMPort>,
    info: PortInfo,
    read_op: Option<ReadOp>,
    read_buffer: Vec<u8>,
    read_offset: usize,
    write_op: Option<WriteOp>,
    flush_task: Option<Task<io::Result<()>>>,
}

impl AsyncSerialPort {
    /// Opens an asynchronous serial port with the given settings.
    pub fn open(builder: &SerialPortBuilder) -> Result<Self> {
        let com = COMPort::open_overlapped(builder)?;
        let info = PortInfo::new(&com);

        // Keep the requested timeout in `info`, but put the live handle into
        // the async timeout mode used by this backend.
        set_async_timeouts(com.as_raw_handle() as HANDLE)?;
        Ok(Self {
            com: Some(com),
            info,
            read_op: None,
            read_buffer: Vec::new(),
            read_offset: 0,
            write_op: None,
            flush_task: None,
        })
    }

    /// Returns the live COM port or a closed-port error.
    fn com_ref(&self) -> io::Result<&COMPort> {
        self.com
            .as_ref()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotConnected, "port is closed"))
    }

    /// Returns the live COM port when it is still open.
    fn com_mut(&mut self) -> Option<&mut COMPort> {
        self.com.as_mut()
    }

    /// Returns the raw Windows handle for the port.
    fn com_handle(&self) -> io::Result<HANDLE> {
        Ok(self.com_ref()?.as_raw_handle() as HANDLE)
    }

    /// Closes the port and drops all local async state.
    fn close(&mut self) {
        self.cancel_pending_io();
        self.flush_task = None;
        self.read_buffer.clear();
        self.read_offset = 0;
        self.com.take();
    }

    /// Cancels any in-flight read or write operation.
    fn cancel_pending_io(&mut self) {
        if let Ok(handle) = self.com_handle() {
            cancel_read_op(handle, &mut self.read_op);
            cancel_write_op(handle, &mut self.write_op);
        } else {
            self.read_op = None;
            self.write_op = None;
        }
    }

    /// Copies bytes from the internal read buffer into `buf`.
    fn poll_read_buffered(&mut self, buf: &mut [u8]) -> Poll<io::Result<usize>> {
        if self.read_offset >= self.read_buffer.len() {
            self.read_buffer.clear();
            self.read_offset = 0;
            return Poll::Pending;
        }

        let remaining = &self.read_buffer[self.read_offset..];
        let copied = remaining.len().min(buf.len());
        buf[..copied].copy_from_slice(&remaining[..copied]);
        self.read_offset += copied;

        if self.read_offset >= self.read_buffer.len() {
            self.read_buffer.clear();
            self.read_offset = 0;
        }

        Poll::Ready(Ok(copied))
    }

    /// Starts a new overlapped read when none is active.
    fn start_read(&mut self, capacity: usize) -> io::Result<()> {
        if self.read_op.is_some() || !self.read_buffer.is_empty() {
            return Ok(());
        }

        let handle = self.com_handle()?;
        let mut op = ReadOp::new(capacity)?;

        // SAFETY: The handle, buffer, and OVERLAPPED stay valid for the call.
        let result = unsafe {
            windows_sys::Win32::Storage::FileSystem::ReadFile(
                handle,
                op.buffer.as_mut_ptr(),
                op.buffer.len() as u32,
                ptr::null_mut(),
                op.overlapped_ptr(),
            )
        };

        if result == 0 {
            let err = io::Error::last_os_error();
            if err.raw_os_error() == Some(ERROR_IO_PENDING as i32) {
                self.read_op = Some(op);
                return Ok(());
            }
            return Err(err);
        }

        let len = finish_overlapped(handle, op.overlapped_ptr())?;
        op.buffer.truncate(len as usize);
        self.read_buffer = op.buffer;
        self.read_offset = 0;
        Ok(())
    }

    /// Starts a new overlapped write for `buf`.
    fn start_write(&mut self, buf: &[u8]) -> io::Result<Option<usize>> {
        let handle = self.com_handle()?;
        let mut op = WriteOp::new(buf.to_vec())?;

        // SAFETY: The handle, buffer, and OVERLAPPED stay valid for the call.
        let result = unsafe {
            windows_sys::Win32::Storage::FileSystem::WriteFile(
                handle,
                op.buffer.as_ptr(),
                op.buffer.len() as u32,
                ptr::null_mut(),
                op.overlapped_ptr(),
            )
        };

        if result == 0 {
            let err = io::Error::last_os_error();
            if err.raw_os_error() == Some(ERROR_IO_PENDING as i32) {
                self.write_op = Some(op);
                return Ok(None);
            }
            return Err(err);
        }

        let written = finish_overlapped(handle, op.overlapped_ptr())?;
        Ok(Some(written as usize))
    }

    /// Starts a blocking flush on the thread pool.
    fn start_flush(&mut self) -> io::Result<()> {
        if self.flush_task.is_some() {
            return Ok(());
        }

        let mut com = self
            .com_ref()?
            .try_clone_native()
            .map_err(io::Error::from)?;
        self.flush_task = Some(unblock(move || std::io::Write::flush(&mut com)));
        Ok(())
    }

    /// Polls the background flush task until it finishes.
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
        self.com
            .as_ref()
            .map(|com| com as &dyn SerialPort)
            .ok_or_else(|| Error::new(ErrorKind::NoDevice, "port is closed"))
    }

    fn port_mut(&mut self) -> Option<&mut dyn SerialPort> {
        self.com_mut().map(|com| com as &mut dyn SerialPort)
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
            if !this.read_buffer.is_empty() {
                return this.poll_read_buffered(buf);
            }

            if this.read_op.is_none() {
                if let Err(err) = this.start_read(buf.len()) {
                    return Poll::Ready(Err(err));
                }

                if !this.read_buffer.is_empty() {
                    return this.poll_read_buffered(buf);
                }

                if this.read_op.is_none() {
                    // Idle reads stay pending. A sync read with 0 bytes means EOF.
                    return Poll::Ready(Ok(0));
                }
            }

            let poll = {
                let op = this.read_op.as_ref().expect("read op was just created");
                op.waitable.poll_ready(cx)
            };

            match poll {
                Poll::Pending => return Poll::Pending,
                Poll::Ready(Ok(())) => {
                    let mut op = this.read_op.take().expect("read op disappeared");
                    match finish_overlapped(this.com_handle()?, op.overlapped_ptr()) {
                        Ok(len) => {
                            op.buffer.truncate(len as usize);
                            this.read_buffer = op.buffer;
                            this.read_offset = 0;
                            continue;
                        }
                        Err(err) if err.raw_os_error() == Some(ERROR_IO_INCOMPLETE as i32) => {
                            this.read_op = Some(op);
                            return Poll::Pending;
                        }
                        Err(err) => return Poll::Ready(Err(err)),
                    }
                }
                Poll::Ready(Err(err)) => return Poll::Ready(Err(err)),
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
        let this = self.as_mut().get_mut();

        loop {
            if let Some(op) = this.write_op.as_ref() {
                if op.buffer.as_slice() != buf {
                    return Poll::Ready(Err(io::Error::new(
                        io::ErrorKind::InvalidInput,
                        "poll_write called with a different buffer while a write is pending",
                    )));
                }

                let poll = op.waitable.poll_ready(cx);

                match poll {
                    Poll::Pending => return Poll::Pending,
                    Poll::Ready(Ok(())) => {
                        let mut op = this.write_op.take().expect("write op disappeared");
                        match finish_overlapped(this.com_handle()?, op.overlapped_ptr()) {
                            Ok(len) => return Poll::Ready(Ok(len as usize)),
                            Err(err) if err.raw_os_error() == Some(ERROR_IO_INCOMPLETE as i32) => {
                                this.write_op = Some(op);
                                return Poll::Pending;
                            }
                            Err(err) => return Poll::Ready(Err(err)),
                        }
                    }
                    Poll::Ready(Err(err)) => return Poll::Ready(Err(err)),
                }
            }

            if buf.is_empty() {
                return Poll::Ready(Ok(0));
            }

            match this.start_write(buf) {
                Ok(Some(len)) => return Poll::Ready(Ok(len)),
                Ok(None) => continue,
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

impl Drop for AsyncSerialPort {
    /// Cancels pending I/O before the port is dropped.
    fn drop(&mut self) {
        self.cancel_pending_io();
    }
}

impl fmt::Debug for AsyncSerialPort {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        async_core::debug_fmt(self, "AsyncSerialPort", f)
    }
}

/// Applies the timeout profile used by this async backend.
fn set_async_timeouts(handle: HANDLE) -> Result<()> {
    // SAFETY: `handle` is an open COM handle.
    let result = unsafe {
        SetCommTimeouts(
            handle,
            &COMMTIMEOUTS {
                ReadIntervalTimeout: 1,
                ReadTotalTimeoutMultiplier: 0,
                ReadTotalTimeoutConstant: 0,
                WriteTotalTimeoutMultiplier: 0,
                WriteTotalTimeoutConstant: 0,
            },
        )
    };

    if result == 0 {
        Err(super::super::error::last_os_error())
    } else {
        Ok(())
    }
}

/// Creates a waitable event for one overlapped operation.
fn new_waitable_event() -> io::Result<Waitable<OwnedHandle>> {
    // SAFETY: Creates a new manual-reset event.
    let handle = unsafe { CreateEventW(ptr::null(), TRUE, FALSE, ptr::null()) };
    if handle == 0 {
        return Err(io::Error::last_os_error());
    }

    // SAFETY: `handle` is new and moved into `OwnedHandle` once.
    let owned = unsafe { OwnedHandle::from_raw_handle(handle as RawHandle) };
    Waitable::new(owned)
}

/// Creates a zeroed `OVERLAPPED` bound to `event_handle`.
fn new_overlapped(event_handle: RawHandle) -> Box<OVERLAPPED> {
    // SAFETY: Zero-init is valid for `OVERLAPPED`.
    let mut overlapped: OVERLAPPED = unsafe { mem::zeroed() };
    overlapped.hEvent = event_handle as HANDLE;
    Box::new(overlapped)
}

/// Finishes one overlapped operation and returns bytes transferred.
fn finish_overlapped(handle: HANDLE, overlapped: *mut OVERLAPPED) -> io::Result<u32> {
    let mut transferred = 0;
    // SAFETY: `handle` and `overlapped` belong to the same in-flight operation.
    let result = unsafe { GetOverlappedResult(handle, overlapped, &mut transferred, FALSE) };

    if result == 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(transferred)
    }
}

/// Cancels the current read, if one exists.
fn cancel_read_op(handle: HANDLE, op: &mut Option<ReadOp>) {
    if let Some(mut op) = op.take() {
        cancel_overlapped(handle, op.overlapped_ptr());
    }
}

/// Cancels the current write, if one exists.
fn cancel_write_op(handle: HANDLE, op: &mut Option<WriteOp>) {
    if let Some(mut op) = op.take() {
        cancel_overlapped(handle, op.overlapped_ptr());
    }
}

/// Cancels one overlapped operation and waits for completion.
fn cancel_overlapped(handle: HANDLE, overlapped: *mut OVERLAPPED) {
    // SAFETY: Cancels the same operation started with this OVERLAPPED.
    let _ = unsafe { CancelIoEx(handle, overlapped) };
    let mut transferred = 0;
    // SAFETY: Waits for the canceled operation to finish and drains completion.
    let _ = unsafe { GetOverlappedResult(handle, overlapped, &mut transferred, TRUE) };
}
