[![crates.io version badge](https://img.shields.io/crates/v/serialport.svg)](https://crates.io/crates/serialport)
[![Documentation](https://docs.rs/serialport/badge.svg)](https://docs.rs/crate/serialport)
![GitHub Workflow Status](https://img.shields.io/github/workflow/status/jessebraham/serialport-rs/CI?label=CI&logo=github&style=flat-square)

> **Note:** This is a fork of the original [serialport-rs](https://gitlab.com/susurrus/serialport-rs) project on GitLab. Please note there have been some changes to both the supported targets and which Tier some targets are in, and there may be further changes to this made. Additionally, all relevant issues have been migrated to this repository.

Join the discussion on Matrix! [#serialport-rs:matrix.org](https://matrix.to/#/#serialport-rs:matrix.org)

# Introduction

`serialport-rs` is a general-purpose cross-platform serial port library for Rust. It provides a
blocking I/O interface and port enumeration on POSIX and Windows systems.

For async I/O functionality, see the [mio-serial](https://github.com/berkowski/mio-serial) and
[tokio-serial](https://github.com/berkowski/tokio-serial) crates.

# Overview

The library exposes cross-platform serial port functionality through the
`SerialPort` struct. Additional platform-dependent features can be enabled by
importing platform-specific `SerialPortExt` traits. `SerialPort` implements the
standard `Read` and `Write` traits.


Serial enumeration is provided on most platforms. The implementation on Linux using `glibc` relies
on `libudev`, an external dynamic library that will need to be available on the system the final
binary is running on. Enumeration will still be available if this feature is disabled, but won't
expose as much information and may return ports that don't exist physically. However this dependency
can be removed by disabling the default `libudev` feature:

```shell
$ cargo build --no-default-features
```

# Usage

Listing available ports:

```rust
let ports = serialport::available_ports().expect("No ports found!");
for p in ports {
    println!("{}", p.port_name);
}
```

Opening and configuring a port:

```rust
let port = SerialPort::builder()
    .baud_rate(115_200)
    .read_timeout(Duration::from_millis(10))
    .open("/dev/ttyUSB0")
    .expect("Failed to open port");
```

Writing to a port:

```rust
use std::io::Write;

let output = "This is a test. This is only a test.".as_bytes();
port.write(output).expect("Write failed!");
```

Reading from a port:

```rust
use std::io::Read;

let mut serial_buf: Vec<u8> = vec![0; 32];
port.read(serial_buf.as_mut_slice()).expect("Read failed");
```

Some platforms expose additional functionality, which is accessed by importing the platform-specific extension trait.

```rust
let port = SerialPort::builder()
    .baud_rate(115_200)
    .read_timeout(Duration::from_millis(10))
    .open("/dev/ttyUSB0")
    .expect("Failed to open port");

#[cfg(windows)]
use serialport::windows::SerialPortExt;

#[cfg(unix)]
use serialport::posix::SerialPortExt;
```

Closing a port:

`serialport-rs` uses the Resource Acquisition Is Initialization (RAII) paradigm and so closing a
port is done when the `SerialPort` object is `Drop`ed either implicitly or explicitly using
`std::mem::drop` (`std::mem::drop(port)`).

# Migrating to Version 5

Prior to version 5 of this library, the `SerialPort` type was a trait, and
cross-platform functionality was provided by using `Box<dyn SerialPort>`.
Platform-specific functionality required using the platform-specific structs,
`COMPort` and `TTYPort`.

In version 5, these types have been unified, with a single `SerialPort` struct
as the only serial port type exposed by the library. Platform-specific
functionality is implemented through extension traits, which can be imported
when needed on a particular platform, to allow you to call extra functions on
the `SerialPort` struct. Using a struct instead of a trait means you no longer
need to `Box` `SerialPort` instances, and the extension traits should make it
easier to write cross-platform code that only occasionally needs access to
platform-specific features.

For example, to send a break on a TTY port, in version 4 and earlier, you would
have to use the `TTYPort` struct instead of the cross-platform `dyn SerialPort`:

```rust
use serialport::BreakDuration;

let port = serialport::new("/dev/ttyUSB0", 9600).open_native()?;
port.send_break(BreakDuration::Short)?;
```

In version 5, you can now use the common `SerialPort` type everywhere, and to
gain access to the platform-specific `send_break` method, you just have to
import the platform-specific trait.

```rust
use serialport::posix::{SerialPortExt, BreakDuration};
use serialport::SerialPort;

let port = SerialPort::builder().open("/dev/ttyUSB0")?;
port.send_break(BreakDuration::Short)?;
```

One other consequence of the switch to a having `SerialPort` as a struct rather
than a trait is that you will now need to import `std::io::Read` and
`std::io::Write` traits explicitly. Previously, the `SerialPort` trait inherited
from `Read` and `Write` so you could call read and write without importing them
whenever the `SerialPort` trait was in scope. With `SerialPort` as a struct, you
now need to explicitly import `Read` and `Write`.

# Examples

There are several included examples, which help demonstrate the functionality of this library and
can help debug software or hardware errors.

- _clear_input_buffer_ - Demonstrates querying and clearing the driver input buffer
- _clear_output_buffer_ - Demonstrates querying and clearing the driver output buffer
- _duplex_ - Tests that a port can be successfully cloned.
- _hardware_check_ - Checks port/driver functionality for a single port or a pair of ports connected
  to each other.
- _list_ports_ - Lists available serial ports.
- _pseudo_terminal_ - Unix only. Tests that a pseudo-terminal pair can be created.
- _receive_data_ - Output data received on a port.
- _transmit_ - Transmits data regularly on a port with various port configurations. Useful for debugging.

# Dependencies

Rust versions 1.46.0 and higher are supported.

For GNU Linux `pkg-config` headers are required:

- Ubuntu: `sudo apt install pkg-config`
- Fedora: `sudo dnf install pkgconf-pkg-config`

For other distros they may provide `pkg-config` through the `pkgconf` package instead.

For GNU Linux `libudev` headers are required as well (unless you disable the default `libudev` feature):

- Ubuntu: `sudo apt install libudev-dev`
- Fedora: `sudo dnf install systemd-devel`

# Platform Support

Platform support is broken into two tiers:

- Tier 1 - Builds and tests for this target are run in CI. Failures of either block the inclusion of new code.
- Tier 2 - Builds for this target are run in CI. Tests are not run in CI.

**Tier 1:**

- Linux
  - `i686-unknown-linux-gnu`
  - `i686-unknown-linux-musl`
  - `x86_64-unknown-linux-gnu`
  - `x86_64-unknown-linux-musl`
- MacOS/iOS
  - `aarch64-apple-darwin`
  - `x86_64-apple-darwin`
- Windows
  - `i686-pc-windows-gnu`
  - `i686-pc-windows-msvc`
  - `x86_64-pc-windows-gnu`
  - `x86_64-pc-windows-msvc`

**Tier 2:**

- Android
  - `aarch64-linux-android` (no serial enumeration)
  - `arm-linux-androideabi` (no serial enumeration)
  - `armv7-linux-androideabi` (no serial enumeration)
  - `i686-linux-android` (no serial enumeration)
  - `x86_64-linux-android` (no serial enumeration)
- FreeBSD
  - `i686-unknown-freebsd`
  - `x86_64-unknown-freebsd`
- Linux
  - `aarch64-unknown-linux-gnu`
  - `aarch64-unknown-linux-musl`
  - `arm-unknown-linux-gnueabi`
  - `arm-unknown-linux-gnueabihf`
  - `arm-unknown-linux-musleabi`
  - `armv5te-unknown-linux-gnueabi`
  - `armv5te-unknown-linux-musleabi`
  - `armv7-unknown-linux-gnueabihf`
  - `armv7-unknown-linux-musleabihf`
  - `i586-unknown-linux-gnu`
  - `i586-unknown-linux-musl`
  - `mips-unknown-linux-gnu`
  - `mips-unknown-linux-musl`
  - `mips64-unknown-linux-gnuabi64`
  - `mips64el-unknown-linux-gnuabi64`
  - `mipsel-unknown-linux-gnu`
  - `mipsel-unknown-linux-musl`
  - `powerpc-unknown-linux-gnu`
  - `powerpc64-unknown-linux-gnu`
  - `powerpc64le-unknown-linux-gnu`
  - `s390x-unknown-linux-gnu`
  - `sparc64-unknown-linux-gnu`
- MacOS/iOS
  - `aarch64-apple-ios`
  - `x86_64-apple-ios`
- NetBSD
  - `x86_64-unknown-netbsd` (no serial enumeration)

# Hardware Support

This library has been developed to support all serial port devices across all
supported platforms. To determine how well your platform is supported, please
run the `hardware_check` example provided with this library. It will test the
driver to confirm that all possible settings are supported for a port.
Additionally, it will test that data transmission is correct for those settings
if you have two ports physically configured to communicate. If you experience
problems with your devices, please file a bug and identify the hardware, OS,
and driver in use.

Known issues:

| Hardware      | OS    | Driver                  | Issues                                                                             |
| ------------- | ----- | ----------------------- | ---------------------------------------------------------------------------------- |
| FTDI TTL-232R | Linux | ftdi_sio, Linux 4.14.11 | Hardware doesn't support 5 or 6 data bits, but the driver lies about supporting 5. |

# Licensing

Licensed under the [Mozilla Public License, version 2.0](https://www.mozilla.org/en-US/MPL/2.0/).

# Contributing

Please open an issue or merge request on GitLab to contibute. Code contributions submitted for
inclusion in the work by you, as defined in the MPL2.0 license, shall be licensed as the above
without any additional terms or conditions.

# Acknowledgments

Special thanks to dcuddeback, willem66745, and apoloval who wrote the original serial-rs library
which this library heavily borrows from.

Additional thanks to susurrus and all other contributors to the original [serialport-rs](https://gitlab.com/susurrus/serialport-rs) project on GitLab.
