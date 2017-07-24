#[cfg(target_os = "linux")]
use libudev;

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
