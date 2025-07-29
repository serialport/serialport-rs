use crate::guid;

use super::wide::*;
use crossbeam::queue::SegQueue;
use futures::Stream;
use parking_lot::Mutex;
use std::{
    cell::OnceCell,
    ffi::{c_void, OsString},
    io,
    pin::Pin,
    sync::Arc,
    task::{Context, Poll, Waker},
    thread::JoinHandle,
};
use winapi::um::winuser::PostMessageW;
use windows_sys::{
    core::GUID,
    Win32::{
        Foundation::{GetLastError, SetLastError, HMODULE, HWND, LPARAM, LRESULT, WPARAM},
        System::LibraryLoader::GetModuleHandleW,
        UI::WindowsAndMessaging::{
            CreateWindowExW, DefWindowProcW, DispatchMessageW, FindWindowW, GetMessageW,
            GetWindowLongPtrW, RegisterClassExW, RegisterDeviceNotificationW, SetWindowLongPtrW,
            TranslateMessage, CW_USEDEFAULT, DBT_DEVICEARRIVAL, DBT_DEVICEREMOVECOMPLETE,
            DBT_DEVTYP_DEVICEINTERFACE, DBT_DEVTYP_PORT, DEVICE_NOTIFY_WINDOW_HANDLE,
            DEV_BROADCAST_DEVICEINTERFACE_W, DEV_BROADCAST_HDR, DEV_BROADCAST_PORT_W,
            GWLP_USERDATA, MSG, WM_CLOSE, WM_DESTROY, WM_DEVICECHANGE, WNDCLASSEXW,
            WS_EX_APPWINDOW, WS_MINIMIZE,
        },
    },
};

/// Create an instance of a DeviceNotifier window.
///
/// Safety: name must be a null terminated Wide string, and user_data must be a pointer to an
/// Arc<SharedQueue>;
unsafe fn create_window(name: *const u16, user_data: isize) -> io::Result<HWND> {
    let handle = CreateWindowExW(
        WS_EX_APPWINDOW,      // styleEx
        WINDOW_CLASS_NAME,    // class name
        name,                 // window name
        WS_MINIMIZE,          // style
        0,                    // x
        0,                    // y
        CW_USEDEFAULT,        // width
        CW_USEDEFAULT,        // hight
        std::ptr::null_mut(), // parent
        std::ptr::null_mut(), // menu
        hinstance(),          // instance
        std::ptr::null(),     // data
    );
    match handle.is_null() {
        true => Err(io::Error::last_os_error()),
        false => {
            // NOTE a 0 is returned if their is a failure, or if the previous pointer was NULL. To
            // distinguish if a true error has occured we have to clear any errors and test the
            // last_os_error == 0 or not.
            let prev = unsafe {
                SetLastError(0);
                SetWindowLongPtrW(handle, GWLP_USERDATA, user_data)
            };
            match prev {
                0 => match unsafe { GetLastError() } as _ {
                    0 => Ok(handle),
                    raw => Err(io::Error::from_raw_os_error(raw)),
                },
                _ => Ok(handle),
            }
        }
    }
}

/// Window proceedure for responding to windows messages and listening for device notifications
unsafe extern "system" fn window_proceedure(
    hwnd: HWND,
    msg: u32,
    wparam: WPARAM,
    lparam: LPARAM,
) -> LRESULT {
    let ptr = GetWindowLongPtrW(hwnd, GWLP_USERDATA) as *const Queue;
    if !ptr.is_null() {
        let queue = &*ptr;
        match msg {
            WM_DEVICECHANGE => {
                // Safety: lparam is a DEV_BROADCAST_HDR when msg is WM_DEVICECHANGE
                match unsafe { parse_event(wparam, lparam) } {
                    Some(ev) => {
                        queue.push(ev);
                        0
                    }
                    _ => DefWindowProcW(hwnd, msg, wparam, lparam),
                }
            }
            WM_DESTROY => {
                // NOTE we only reconstruct our arc on destroy
                let arc = Arc::from_raw(ptr as *const Queue);
                arc.done();
                0
            }
            _ => DefWindowProcW(hwnd, msg, wparam, lparam),
        }
    } else {
        DefWindowProcW(hwnd, msg, wparam, lparam)
    }
}

unsafe fn parse_event(wparam: WPARAM, lparam: LPARAM) -> Option<Event> {
    match wparam as u32 {
        DBT_DEVICEARRIVAL => Some(Event::Arrival(maybe_serialport(lparam as _)?)),
        DBT_DEVICEREMOVECOMPLETE => Some(Event::RemoveComplete(maybe_serialport(lparam as _)?)),
        _ => None,
    }
}

/// Safety: data must be a DEV_BROADCAST_HDR
unsafe fn maybe_serialport(data: *mut c_void) -> Option<String> {
    let broadcast = &mut *(data as *mut DEV_BROADCAST_HDR);
    match broadcast.dbch_devicetype {
        DBT_DEVTYP_PORT => {
            let data = &*(data as *const DEV_BROADCAST_PORT_W);
            from_wide(data.dbcp_name.as_ptr())
                .to_str()
                .map(|port| port.to_string())
        }
        _ => None,
    }
}

/// Dispatch window messages
///
/// We receive a "name", a list of GUID registrations, and some "user_data" which is an arc.
///
/// Safety: user_data must be a pointer to an Arc<SharedQueue> that was created
/// by Arc::into_raw...
///
/// This method will rebuild the Arc and pass it to the window procedure...
pub unsafe fn window_dispatcher(name: OsString, user_data: isize) -> io::Result<()> {
    const WCEUSBS: GUID =
        guid!(0x25dbce51, 0x6c8f, 0x4a72, 0x8a, 0x6d, 0xb5, 0x4c, 0x2b, 0x4f, 0xc8, 0x35);
    const USBDEVICE: GUID =
        guid!(0x88BAE032, 0x5A81, 0x49f0, 0xBC, 0x3D, 0xA4, 0xFF, 0x13, 0x82, 0x16, 0xD6);
    const PORTS: GUID =
        guid!(0x4d36e978, 0xe325, 0x11ce, 0xbf, 0xc1, 0x08, 0x00, 0x2b, 0xe1, 0x03, 0x18);
    let _atom = get_window_class();
    let unsafe_name = to_wide(name.clone());
    let arc = Arc::from_raw(user_data as *const Arc<Queue>);
    let hwnd = create_window(unsafe_name.as_ptr(), Arc::as_ptr(&arc) as _)?;
    let _registery = [WCEUSBS, USBDEVICE, PORTS]
        .into_iter()
        .map(|guid| {
            let handle = unsafe {
                let mut iface = std::mem::zeroed::<DEV_BROADCAST_DEVICEINTERFACE_W>();
                iface.dbcc_size = std::mem::size_of::<DEV_BROADCAST_DEVICEINTERFACE_W>() as _;
                iface.dbcc_classguid = guid;
                iface.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;
                RegisterDeviceNotificationW(
                    hwnd as _,
                    &iface as *const _ as _,
                    DEVICE_NOTIFY_WINDOW_HANDLE,
                )
            };
            match handle.is_null() {
                false => Ok(handle),
                true => Err(io::Error::last_os_error()),
            }
        })
        .collect::<io::Result<Vec<_>>>()?;

    let mut msg: MSG = std::mem::zeroed();
    loop {
        match GetMessageW(&mut msg as *mut _, std::ptr::null_mut(), 0, 0) {
            0 => {
                break Ok(());
            }
            -1 => {
                let error = Err(io::Error::last_os_error());
                break error;
            }
            _ if msg.message == WM_CLOSE => {
                TranslateMessage(&msg as *const _);
                DispatchMessageW(&msg as *const _);
                break Ok(());
            }
            _ => {
                TranslateMessage(&msg as *const _);
                DispatchMessageW(&msg as *const _);
            }
        }
    }
}

/// The name of our window class.
/// [See also](https://learn.microsoft.com/en-us/windows/win32/winmsg/about-window-classes)
const WINDOW_CLASS_NAME: *const u16 = windows_sys::w!("DeviceNotifier");

/// We register our class only once
const WINDOW_CLASS_ATOM: OnceCell<u16> = OnceCell::new();
fn get_window_class() -> u16 {
    *WINDOW_CLASS_ATOM.get_or_init(|| {
        let class = WNDCLASSEXW {
            style: 0,
            hIcon: std::ptr::null_mut(),
            cbSize: std::mem::size_of::<WNDCLASSEXW>() as _,
            hIconSm: std::ptr::null_mut(),
            hCursor: std::ptr::null_mut(),
            cbClsExtra: 0,
            cbWndExtra: 0,
            hInstance: hinstance(),
            lpszMenuName: std::ptr::null(),
            lpszClassName: WINDOW_CLASS_NAME,
            lpfnWndProc: Some(window_proceedure),
            hbrBackground: std::ptr::null_mut(),
        };
        match unsafe { RegisterClassExW(&class as *const _) } {
            0 => panic!("{:?}", io::Error::last_os_error()),
            atom => atom,
        }
    })
}

/// Creating Windows requires the hinstance prop of the WinMain function. To retreive this
/// parameter use [`windows_sys::Win32::System::LibraryLoader::GetModuleHandleW`];
fn hinstance() -> HMODULE {
    // Safety: If the handle is NULL, GetModuleHandle returns a handle to the file used to create
    // the calling process
    unsafe { GetModuleHandleW(std::ptr::null()) }
}

/// When a user will Plug or Unplug a Usb Port you will receive an [`Event`]
///
/// To get a stream of [`Event`], use [`crate::listen`]
#[derive(Debug)]
pub enum Event {
    /// A Usb has plugged into the system. The string is the name of the Port. IE: COM3
    Arrival(String),
    /// A Usb has unplugged from the system. The string is the name of the Port. IE: COM3
    RemoveComplete(String),
}

#[derive(Default)]
pub struct Queue {
    inner: SegQueue<Option<Event>>,
    waker: Mutex<Option<Waker>>,
}

impl Queue {
    pub fn new() -> Queue {
        Queue {
            inner: SegQueue::new(),
            waker: Mutex::new(None),
        }
    }

    pub fn maybe_wake(&self) {
        if let Some(waker) = &self.waker.lock().as_ref() {
            waker.wake_by_ref();
        }
    }

    pub fn push(&self, ev: Event) {
        self.inner.push(Some(ev));
        self.maybe_wake();
    }

    pub fn done(&self) {
        self.inner.push(None);
        self.maybe_wake();
    }

    pub fn poll_next(&self, cx: &mut Context<'_>) -> Poll<Option<Event>> {
        match self.inner.pop() {
            None => {
                let new_waker = cx.waker();
                let mut waker = self.waker.lock();
                *waker = match waker.take() {
                    None => Some(new_waker.clone()),
                    Some(old_waker) => {
                        if old_waker.will_wake(new_waker) {
                            Some(old_waker)
                        } else {
                            Some(new_waker.clone())
                        }
                    }
                };
                Poll::Pending
            }
            Some(Some(inner)) => Poll::Ready(Some(inner)),
            Some(None) => Poll::Ready(None),
        }
    }
}

/// Stream of [`Event`]
///
/// See also [`crate::listen`]
pub struct Listener {
    pub(crate) window: OsString,
    pub(crate) context: Arc<Queue>,
    pub(crate) join_handle: Option<JoinHandle<io::Result<()>>>,
}

impl std::fmt::Debug for Listener {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Listener")
            .field("window", &self.window)
            .finish()
    }
}

impl Listener {
    /// Stop listening to events
    pub fn close(&mut self) -> io::Result<()> {
        // Find the window so we can close it
        let wide = to_wide(self.window.clone());
        let hwnd = unsafe {
            let result = FindWindowW(WINDOW_CLASS_NAME, wide.as_ptr());
            match result.is_null() {
                true => Err(io::Error::last_os_error()),
                _ => Ok(result),
            }
        }?;

        // Close the window
        let _close = unsafe {
            let result = PostMessageW(hwnd as _, WM_CLOSE, 0, 0);
            match result {
                0 => Err(io::Error::last_os_error()),
                _ => Ok(()),
            }
        }?;

        // Join the thread
        let jh = self
            .join_handle
            .take()
            .ok_or_else(|| io::Error::new(io::ErrorKind::Other, "Already closed WindowEvents"))?;
        jh.join()
            .map_err(|_| io::Error::new(io::ErrorKind::Other, "join error"))?
    }
}

impl Drop for Listener {
    fn drop(&mut self) {
        let _ = self.close();
    }
}

impl Stream for Listener {
    type Item = Event;
    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        self.context.poll_next(cx)
    }
}
