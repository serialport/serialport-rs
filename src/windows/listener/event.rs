use std::{
    ffi::{c_void, OsString},
    future::Future,
    io,
    os::windows::{
        ffi::OsStrExt,
        io::{AsRawHandle, HandleOrNull, OwnedHandle, RawHandle},
    },
    pin::Pin,
    sync::{Arc, Mutex},
    task::{Context, Poll, Waker},
    time::Duration,
};

use windows_sys::Win32::{
    Foundation::{FALSE, FILETIME, TRUE, WAIT_ABANDONED, WAIT_FAILED, WAIT_OBJECT_0, WAIT_TIMEOUT},
    System::Threading::{
        CreateEventW, CreateThreadpoolWait, ResetEvent, SetEvent, SetThreadpoolWait,
        WaitForSingleObject, WaitForThreadpoolWaitCallbacks, INFINITE, PTP_CALLBACK_INSTANCE,
        PTP_WAIT,
    },
};

#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(u32)]
pub enum Error {
    Timeout = WAIT_TIMEOUT,
    Abandoned = WAIT_ABANDONED,
    Failed = WAIT_FAILED,
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Timeout => write!(f, "Wait timed out"),
            Error::Abandoned => write!(f, "Wait abandoned"),
            Error::Failed => write!(f, "Wait failed"),
            //Error::Io(e) => write!(f, "Wait io => {e}"),
        }
    }
}

impl std::error::Error for Error {}

/// Windows CreateEvent creation argument
///
/// If this parameter is TRUE, the function creates a manual-reset event object,
/// which requires the use of the ResetEvent function to set the event state to
/// nonsignaled. If this parameter is FALSE, the function creates an auto-reset
/// event object, and the system automatically resets the event state to
/// nonsignaled after a single waiting thread has been released.
#[repr(i32)]
#[derive(PartialEq)]
pub enum EventReset {
    Manual = TRUE,
    Automatic = FALSE,
}

/// Windows CreateEvent creation argument
///
/// If this parameter is TRUE, the initial state of the event object is
/// signaled; otherwise, it is nonsignaled.
#[repr(i32)]
pub enum EventInitialState {
    Set = TRUE,
    Unset = FALSE,
}

#[derive(Debug)]
pub struct Event(OwnedHandle);

impl Event {
    /// Create a system event
    ///
    /// [CreateEventW](https://learn.microsoft.com/en-us/windows/win32/api/synchapi/nf-synchapi-createeventa)
    pub fn named<O>(name: O, reset: EventReset, state: EventInitialState) -> io::Result<Event>
    where
        O: Into<OsString>,
    {
        let kernel_name = name
            .into()
            .encode_wide()
            .chain(Some(0).into_iter())
            .collect::<Vec<_>>();
        Self::new_raw(kernel_name.as_ptr() as _, reset, state)
    }

    /// Create a system event with out a name
    ///
    /// [CreateEventW](https://learn.microsoft.com/en-us/windows/win32/api/synchapi/nf-synchapi-createeventa)
    pub fn anonymous(reset: EventReset, state: EventInitialState) -> io::Result<Event> {
        Self::new_raw(std::ptr::null(), reset, state)
    }

    pub fn new_raw(
        name: *const u16,
        reset: EventReset,
        state: EventInitialState,
    ) -> io::Result<Event> {
        unsafe {
            let raw = CreateEventW(std::ptr::null(), reset as _, state as _, name);
            let handle = HandleOrNull::from_raw_handle(raw as _);
            OwnedHandle::try_from(handle).map_err(|_| io::Error::last_os_error())
        }
        .map(Self)
    }

    pub fn set(&self) -> io::Result<()> {
        match unsafe { SetEvent(self.as_raw_handle() as _) } {
            FALSE => Err(io::Error::last_os_error()),
            _ => Ok(()),
        }
    }

    pub fn reset(&self) -> io::Result<()> {
        match unsafe { ResetEvent(self.as_raw_handle() as _) } {
            FALSE => Err(io::Error::last_os_error()),
            _ => Ok(()),
        }
    }

    pub fn wait(&self, duration: Option<Duration>) -> Result<(), Error> {
        let dur: u32 = duration.map(|d| d.as_millis() as _).unwrap_or(INFINITE);
        match unsafe { WaitForSingleObject(self.as_raw_handle() as _, dur as _) } {
            WAIT_OBJECT_0 => Ok(()),
            WAIT_ABANDONED => Err(Error::Abandoned),
            WAIT_FAILED => Err(Error::Failed),
            WAIT_TIMEOUT => Err(Error::Timeout),
            // https://learn.microsoft.com/en-us/windows/win32/api/synchapi/nf-synchapi-waitforsingleobject#return-value
            _ => unreachable!(),
        }
    }
}

impl AsRawHandle for Event {
    fn as_raw_handle(&self) -> RawHandle {
        self.0.as_raw_handle()
    }
}

/// Callback for which to register Wait handlers
type WaitCallback = unsafe extern "system" fn(PTP_CALLBACK_INSTANCE, *mut c_void, PTP_WAIT, u32);

/// Wait for pending threadpool callbacks, or cancel pending threadpool callbacks
#[repr(u32)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum WaitPending {
    /// Wait for pending threadpool callbacks
    Wait = 0,
    /// Cancel pending threadpool callbacks
    Cancel = 1,
}

#[derive(Default, Debug)]
struct WaitState {
    waker: Option<Waker>,
    result: Option<std::result::Result<(), Error>>,
}

#[derive(Debug)]
pub struct WaitPool(PTP_WAIT);

impl WaitPool {
    /// https://learn.microsoft.com/en-us/windows/win32/api/threadpoolapiset/nf-threadpoolapiset-createthreadpoolwait
    pub fn new(cx: *mut c_void) -> io::Result<Self> {
        let result = unsafe { CreateThreadpoolWait(Some(oneshot_callback), cx, std::ptr::null()) };
        match result {
            0 => Err(io::Error::last_os_error()),
            handle => Ok(WaitPool(handle)),
        }
    }

    /// Set a wait object on a threadpool which will trigger a wait callback
    ///
    /// https://learn.microsoft.com/en-us/windows/win32/api/threadpoolapiset/nf-threadpoolapiset-setthreadpoolwait
    pub fn start<H: AsRawHandle>(&self, waitable: &H, timeout: Option<Duration>) {
        let ft = timeout
            .map(|to| {
                let ms = to.as_millis();
                &FILETIME {
                    dwHighDateTime: (ms >> 32) as u32,
                    dwLowDateTime: (ms & 0xFFFFFFFF) as u32,
                } as *const _
            })
            .unwrap_or_else(std::ptr::null);
        unsafe { SetThreadpoolWait(self.0, waitable.as_raw_handle() as _, ft) };
    }

    /// The wait object will cease to queue new callbacks. Callbacks already queued will still fire
    ///
    /// https://learn.microsoft.com/en-us/windows/win32/api/threadpoolapiset/nf-threadpoolapiset-setthreadpoolwait
    pub fn stop(&self) {
        let ft = FILETIME {
            dwLowDateTime: 0,
            dwHighDateTime: 0,
        };
        // Handle == 0 pool will cease to queue new callbacks. existing callbacks still occur
        unsafe { SetThreadpoolWait(self.0, std::ptr::null_mut(), &ft) };
    }

    /// Waits for outstanding wait callbacks to complete and optionally cancels pending callbacks
    /// that have not yet started to execute.
    ///
    /// https://learn.microsoft.com/en-us/windows/win32/api/threadpoolapiset/nf-threadpoolapiset-waitforthreadpoolwaitcallbacks
    pub fn wait(&self, pending: WaitPending) {
        unsafe { WaitForThreadpoolWaitCallbacks(self.0, pending as _) };
    }
}

#[derive(Debug)]
pub struct Sender {
    #[allow(unused)]
    state: Arc<(Mutex<WaitState>, Event)>,
}

impl Sender {
    pub fn set(self) -> io::Result<()> {
        self.state.1.set()
    }
}

#[derive(Debug)]
pub struct Receiver {
    #[allow(unused)]
    pool: WaitPool,
    state: Arc<(Mutex<WaitState>, Event)>,
}

impl Future for Receiver {
    type Output = std::result::Result<(), Error>;
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let mut state = match self.state.0.lock() {
            Ok(state) => state,
            Err(_) => return Poll::Ready(Err(Error::Failed)),
        };

        match state.result {
            Some(result) => {
                if let Some(waker) = state.waker.take() {
                    waker.wake()
                }
                Poll::Ready(result)
            }
            None => {
                // Update our waker
                let new_waker = cx.waker();
                state.waker = match state.waker.take() {
                    None => Some(new_waker.clone()),
                    Some(old_waker) => match old_waker.will_wake(new_waker) {
                        false => Some(cx.waker().clone()),
                        true => Some(old_waker),
                    },
                };
                Poll::Pending
            }
        }
    }
}

fn oneshot() -> io::Result<(Sender, Receiver)> {
    let event = Event::anonymous(EventReset::Manual, EventInitialState::Unset)?;
    let state = Arc::new((Mutex::new(WaitState::default()), event));
    let pool = WaitPool::new(Arc::as_ptr(&state) as _)?;
    pool.start(&state.1, None);
    let sender = Sender { state };
    let receiver = Receiver {
        state: Arc::clone(&sender.state),
        pool,
    };
    Ok((sender, receiver))
}

unsafe extern "system" fn oneshot_callback(
    _instance: PTP_CALLBACK_INSTANCE,
    context: *mut c_void,
    _wait: PTP_WAIT,
    waitresult: u32,
) {
    let state = &*(context as *const (Mutex<WaitState>, Event));
    if let Ok(mut shared) = state.0.lock() {
        shared.result = match waitresult {
            WAIT_OBJECT_0 => Some(Ok(())),
            WAIT_TIMEOUT => Some(Err(Error::Timeout)),
            _ => panic!("Unsupported kernel argument passed to wait callback!"),
        };
        if let Some(waker) = shared.waker.as_ref() {
            waker.wake_by_ref()
        }
    }
}

#[cfg(test)]
mod test {
    use futures::FutureExt;
    #[test]
    fn test_event_oneshot() {
        // Create a test waker
        let waker = futures::task::noop_waker_ref();
        let mut cx = std::task::Context::from_waker(waker);

        // Create a channel signal
        let (sender, mut receiver) = super::oneshot().unwrap();

        // Make sure we are pending
        let poll = receiver.poll_unpin(&mut cx);
        assert!(poll.is_pending());

        // Make sure we set event and are no longer pending anymore
        // NOTE we set the time delay to allow kernel some time to drive our future
        sender.set().unwrap();
        std::thread::sleep(std::time::Duration::from_millis(10));
        let poll = receiver.poll_unpin(&mut cx);
        assert!(poll.is_ready());
    }
}
use futures::FutureExt;
