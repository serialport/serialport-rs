//! Provides a cache for event `HANDLE`s so that we can create event handles for
//! read/write lazily while still keeping them around between reads and writes.
use std::io;
use std::ptr;
use std::sync::atomic::{AtomicUsize, Ordering};

use winapi::shared::minwindef::*;
use winapi::shared::ntdef::NULL;
use winapi::um::handleapi::*;
use winapi::um::synchapi::CreateEventW;
use winapi::um::winnt::HANDLE;

/// Cache for a HANDLE to an event.
#[derive(Debug)]
pub struct EventCache {
    /// A `HANDLE` to an Event, created with `CreateEventW`, or `NULL`. We use
    /// `NULL` to represent a missing handle rather than `INVALID_HANDLE_VALUE`
    /// because `CreateEventW` returns `NULL` rather than `INVALID_HANDLE_VALUE`
    /// on failure.
    handle: AtomicUsize,
}

impl EventCache {
    /// Create a new, empty cache.
    pub fn new() -> Self {
        EventCache {
            handle: AtomicUsize::new(NULL as usize),
        }
    }

    /// Take out the currently contained `HANDLE` if any, or create a new `HANDLE`.
    /// Returns an error only when creating a new event handle fails.
    pub fn take_or_create(&self) -> io::Result<HandleGuard> {
        // Fast path: there is a handle, just take it and return it.
        let existing = self.handle.swap(NULL as usize, Ordering::Relaxed) as HANDLE;
        if existing != NULL {
            return Ok(HandleGuard {
                cache: self,
                handle: existing,
            });
        }

        // We can use auto-reset for both read and write because we'll have a different event
        // handle for every thread that's trying to read or write.
        match unsafe { CreateEventW(ptr::null_mut(), FALSE, FALSE, ptr::null_mut()) } {
            NULL => Err(io::Error::last_os_error()),
            new_handle => Ok(HandleGuard {
                cache: self,
                handle: new_handle,
            }),
        }
    }

    /// Return the given `HANDLE` to the cache or silently deallocate it.
    fn return_or_deallocate(&self, handle: HANDLE) {
        if self
            .handle
            .compare_exchange_weak(
                NULL as usize,
                handle as usize,
                Ordering::Relaxed,
                Ordering::Relaxed,
            )
            .is_err()
        {
            // Already-stored value was not null, so just silently deallocate the returned handle.
            unsafe { CloseHandle(handle) };
        }
    }
}

impl Drop for EventCache {
    fn drop(&mut self) {
        let handle = (*self.handle.get_mut()) as HANDLE;
        if handle != NULL {
            unsafe { CloseHandle(handle) };
        }
    }
}

/// Guard for borrowing the event handle from the `EventCache`. It will return
/// the handle to the cache when dropped, or deallocate it if the cache already
/// contains a handle.
pub struct HandleGuard<'a> {
    /// Event cache to return the handle to when dropped.
    cache: &'a EventCache,
    /// Actual handle value.
    handle: HANDLE,
}

impl<'a> HandleGuard<'a> {
    /// Get the handle from this guard.
    pub fn handle(&self) -> HANDLE {
        self.handle
    }
}

impl<'a> Drop for HandleGuard<'a> {
    fn drop(&mut self) {
        self.cache.return_or_deallocate(self.handle);
    }
}
