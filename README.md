[![crates.io version badge](https://img.shields.io/crates/v/serialport.svg)](https://crates.io/crates/serialport)
[![Documentation](https://docs.rs/serialport/badge.svg)](https://docs.rs/crate/serialport)
![GitHub Workflow Status](https://img.shields.io/github/workflow/status/serialport/serialport-rs/CI?label=CI&logo=github&style=flat-square)

> **Note:** This is a fork of the original [serialport-rs](https://gitlab.com/susurrus/serialport-rs) project on GitLab. Please note there have been some changes to both the supported targets and which Tier some targets are in, and there may be further changes to this made. Additionally, all relevant issues have been migrated to this repository.

Join the discussion on Matrix! [#serialport-rs:matrix.org](https://matrix.to/#/#serialport-rs:matrix.org)

**This project is looking for maintainers! If you are interested please let us know on Matrix, or by [creating a Discussion](https://github.com/serialport/serialport-rs/discussions/new).**

# Introduction

`serialport-rs` is a general-purpose cross-platform serial port library for Rust. It provides a
blocking I/O interface and port enumeration on POSIX and Windows systems.

For async I/O functionality, see the [mio-serial](https://github.com/berkowski/mio-serial) and
[tokio-serial](https://github.com/berkowski/tokio-serial) crates.

# Overview

The library exposes cross-platform serial port functionality through the `SerialPort` trait. This
library is structured to make this the simplest API to use to encourate cross-platform development
by default. Working with the resultant `Box<dyn SerialPort>` type is therefore recommended. To
expose additional platform-specific functionality use the platform-specific structs directly:
`TTYPort` for POSIX systems and `COMPort` for Windows.

Serial enumeration is provided on most platforms. The implementation on Linux using `glibc` relies
on `libudev`, an external dynamic library that will need to be available on the system the final
binary is running on. Enumeration will still be available if this feature is disabled, but won't
expose as much information and may return ports that don't exist physically. However this dependency
can be removed by disabling the default `libudev` feature:

```shell
$ cargo build --no-default-features
```

It should also be noted that on macOS, both the Callout (`/dev/cu.*`) and Dial-in ports (`/dev/tty.*`) ports are enumerated, resulting in two available ports per connected serial device.

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
let port = serialport::new("/dev/ttyUSB0", 115_200)
    .timeout(Duration::from_millis(10))
    .open().expect("Failed to open port");
```

Writing to a port:

```rust
let output = "This is a test. This is only a test.".as_bytes();
port.write(output).expect("Write failed!");
```

Reading from a port (default is blocking with a 0ms timeout):

```rust
let mut serial_buf: Vec<u8> = vec![0; 32];
port.read(serial_buf.as_mut_slice()).expect("Found no data!");
```

Some platforms expose additional functionality, which is opened using the `open_native()` method:

```rust
let port = serialport::new("/dev/ttyUSB0", 115_200)
    .open_native().expect("Failed to open port");
```

Closing a port:

`serialport-rs` uses the Resource Acquisition Is Initialization (RAII) paradigm and so closing a
port is done when the `SerialPort` object is `Drop`ed either implicitly or explicitly using
`std::mem::drop` (`std::mem::drop(port)`).

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

Builds and tests for all supported targets are run in CI. Failures of either block the inclusion of new code. This library should be compatible with additional targets not listed below, but no guarantees are made. Additional platforms may be added in the future if there is a need and/or demand.

- Android
  - `arm-linux-androideabi` (no serial enumeration)
  - `armv7-linux-androideabi` (no serial enumeration)
- FreeBSD
  - `x86_64-unknown-freebsd`
- Linux
  - `aarch64-unknown-linux-gnu`
  - `aarch64-unknown-linux-musl`
  - `i686-unknown-linux-gnu`
  - `i686-unknown-linux-musl`
  - `x86_64-unknown-linux-gnu`
  - `x86_64-unknown-linux-musl`
- MacOS/iOS
  - `aarch64-apple-darwin`
  - `aarch64-apple-ios`
  - `x86_64-apple-darwin`
- NetBSD
  - `x86_64-unknown-netbsd` (no serial enumeration)
- Windows
  - `i686-pc-windows-gnu`
  - `i686-pc-windows-msvc`
  - `x86_64-pc-windows-gnu`
  - `x86_64-pc-windows-msvc`

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
