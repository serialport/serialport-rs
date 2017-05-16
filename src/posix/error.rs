#[cfg(target_os = "linux")]
use libudev;
use nix;

#[cfg(target_os = "linux")]
impl From<libudev::Error> for ::Error {
    fn from(e: libudev::Error) -> ::Error {
        let description = e.description().to_string();
        match e.kind() {
            libudev::ErrorKind::NoMem => ::Error::new(::ErrorKind::Unknown, description),
            libudev::ErrorKind::InvalidInput => {
                ::Error::new(::ErrorKind::InvalidInput, description)
            }
            libudev::ErrorKind::Io(a) => ::Error::new(::ErrorKind::Io(a), description),
        }
    }
}

impl From<nix::Error> for ::Error {
    fn from(e: nix::Error) -> ::Error {
        let description = e.errno().desc();
        match e {
            nix::Error::InvalidPath => ::Error::new(::ErrorKind::InvalidInput, description),
            _ => ::Error::new(::ErrorKind::Unknown, description),
        }
    }
}
