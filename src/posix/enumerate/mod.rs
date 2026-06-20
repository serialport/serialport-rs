use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(target_vendor = "apple")] {
        mod apple;
        pub use apple::available_ports;
    } else if #[cfg(all(target_os = "linux", not(target_env = "musl"), feature = "libudev"))] {
        mod linux_libudev;
        pub use linux_libudev::available_ports;
    } else if #[cfg(target_os = "linux")] {
        mod linux_sysfs;
        pub use linux_sysfs::available_ports;
    } else if #[cfg(target_os = "freebsd")] {
        mod freebsd;
        pub use freebsd::available_ports;
    } else {
        mod unimplemented;
        pub use unimplemented::available_ports;
    }
}
