mod guid;
mod wide;
mod wm;

use std::{ffi::OsString, sync::Arc};
pub use wm::Listener;

pub fn listen<N>(n: N) -> Listener
where
    N: Into<OsString>,
{
    let name: OsString = n.into();
    let window = name.clone();
    let ours = Arc::new(wm::Queue::new());
    let theirs = Arc::clone(&ours);
    let jh = std::thread::spawn(move || unsafe {
        wm::window_dispatcher(name, Arc::into_raw(theirs) as _)
    });
    Listener {
        window,
        context: ours,
        join_handle: Some(jh),
    }
}
