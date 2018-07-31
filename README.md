[![crates.io version badge](https://img.shields.io/crates/v/serialport.svg)](https://crates.io/crates/serialport)
[![Documentation](https://docs.rs/serialport/badge.svg)](https://docs.rs/crate/serialport)


[![GitLab CI status](https://gitlab.com/susurrus/serialport-rs/badges/master/build.svg)](https://gitlab.com/susurrus/serialport-rs/pipelines)
[![Appveyor CI status](https://ci.appveyor.com/api/projects/status/gitlab/Susurrus/serialport-rs?svg=true&branch=master)](https://ci.appveyor.com/project/Susurrus/serialport-rs)
[![Travis CI status](https://travis-ci.org/Susurrus/serialport-rs.svg?branch=master)](https://travis-ci.org/Susurrus/serialport-rs)

Overview
========

`serialport-rs` is a general-purpose cross-platform serial port library for Rust. It provides a
simple blocking I/O interface and port enumeration on POSIX and Windows systems.

For async I/O functionality, see the [mio-serial](https://github.com/berkowski/mio-serial) and
[tokio-serial](https://github.com/berkowski/tokio-serial) crates.

The canonical repository for this crate is on [GitLab](https://gitlab.com/susurrus/serialport-rs),
but it is mirrored on GitHub purely for testing via Travis CI. To report any issues or contribute
code, please do so using through the GitLab repository.

Features
========

The library has been organized such that there is a high-level `SerialPort` trait that provides
a cross-platform API for accessing serial ports. This is the preferred method of interacting
with ports and as such is part of the `prelude`. The `open*()` and `available_ports()` functions
in the root are also cross-platform.

For platform-specific functionality, this crate is split into a `posix` and `windows` API with
corresponding `TTYPort` and `COMPort` structs, both of which implement the `SerialPort` trait. Using
the platform-specific `open*()` functions will return the platform-specific port object which
allows access to platform-specific functionality.

Examples
========

There are several included examples, which both demonstrate the functionality of this library and
serve to help debug software or hardware errors.

 * *clear_input_buffer* - Demonstrates querying and clearing the driver input buffer
 * *clear_output_buffer* - Demonstrates querying and clearing the driver output buffer
 * *duplex* - Tests that a port can be successfully cloned.
 * *hardware\_check* - Checks port/driver functionality for a single port or a pair of ports connected
   to each other.
 * *heartbeat* - Transmits data regularly on a port.
 * *list_ports* - Lists available serial ports.
 * *pseudo_terminal* - Unix only. Tests that a pseudo-terminal pair can be created.
 * *receive_data* - Print data received on a port.

Dependencies
============

Rust versions 1.20 and higher are supported.

For GNU Linux `pkg-config` and `libudev` headers are required (`pkg-config` and `libudev-dev` on Ubuntu respectively).

Platform Support
================

Platform support is broken into three tiers:

 * Tier 1 - Builds and tests for this target are run in CI. Failures of either block the inclusion of new code.
 * Tier 2 - Builds for this target are run in CI. Failures during the build blocks the inclusion of new code. Tests may be run, but failures in tests don't block the inclusion of new code.
 * Tier 3 - Builds for this target are run in CI. Failures during the build do not block the inclusion of new code. Testing may be run, but failures in tests don't block the inclusion of new code.


Tier 1:

 * Linux
   * `i586-unknown-linux-musl`
   * `i686-unknown-linux-gnu`
   * `i686-unknown-linux-musl`
   * `x86_64-unknown-linux-gnu`
   * `x86_64-unknown-linux-musl`
 * MacOS/iOS
   * `i686-apple-darwin`
   * `x86_64-apple-darwin`
 * Windows
   * `i686-pc-windows-gnu`
   * `i686-pc-windows-msvc`
   * `x86_64-pc-windows-gnu`
   * `x86_64-pc-windows-msvc`

Tier 2:

 * Android
   * `arm-linux-androideabi`
   * `armv7-linux-androideabi`
   * `i686-linux-android`
 * FreeBSD
   * `i686-unknown-freebsd`
   * `x86_64-unknown-freebsd`
 * Linux
   * `aarch64-unknown-linux-gnu`
   * `arm-unknown-linux-musleabi`
   * `armv7-unknown-linux-musleabihf`
   * `mips64-unknown-linux-gnuabi64`
   * `mips64el-unknown-linux-gnuabi64`
   * `mips-unknown-linux-gnu`
   * `mips-unknown-linux-musl`
   * `mipsel-unknown-linux-gnu`
   * `mipsel-unknown-linux-musl`
   * `powerpc64-unknown-linux-gnu`
   * `powerpc64le-unknown-linux-gnu`
   * `powerpc-unknown-linux-gnu`
   * `s390x-unknown-linux-gnu`
 * NetBSD
   * `x86_64-unknown-netbsd`

Tier 3:

 * Android
   * `aarch64-linux-android`
   * `x86_64-linux-android`
 * Linux
   * `aarch64-unknown-linux-musl`
   * `sparc64-unknown-linux-gnu`
   * `x86_64-unknown-linux-gnux32`

Hardware Support
================

This library has been developed to support all serial port devices across all
supported platforms. To determine how well your platform is supported, please
run the `hardware_check` example provided with this library. It will test the
driver to confirm that all possible settings are supported for a port.
Additionally, it will test that data transmission is correct for those settings
if you have two ports physically configured to communicate. If you experience
problems with your devices, please file a bug and identify the hardware, OS,
and driver in use.

Known issues:

|    Hardware   |   OS  |        Driver           |                                      Issues                                        |
| ------------- | ----- | ----------------------- | ---------------------------------------------------------------------------------- |
| FTDI TTL-232R | Linux | ftdi_sio, Linux 4.14.11 | Hardware doesn't support 5 or 6 data bits, but the driver lies about supporting 5. |

Licensing
=========

Licensed under the [Mozilla Public License, version 2.0](https://www.mozilla.org/en-US/MPL/2.0/).


Contributing
============

Please open an issue or merge request on GitLab to contibute. Code contributions submitted for
inclusion in the work by you, as defined in the MPL2.0 license, shall be licensed as the above
without any additional terms or conditions.

Acknowledgments
===============

Special thanks to dcuddeback, willem66745, and apoloval who wrote the original serial-rs library
which this library heavily borrows from.
