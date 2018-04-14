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

Rationale
=========

This library started as a fork of the [serial-rs](https://github.com/dcuddeback/serial-rs)
library because `serial-rs` was slow to include enumeration support. Additionally I found its API a
bit cumbersome as I was more familiar with `QSerialPort`'s API.

Dependencies
============

Rust versions 1.20 and higher are supported.

Platform Support
================

Operating system support is as follows:

Tier 1:

  * Linux, i686 & x86_64

Tier 2:

  * Windows 7+, i686 & x86_64
  * Mac OS X

Tier 1 supports means this library is build- and run-tested and is regularly used so this should
be pretty bug-free. Tier 2 means it's only build- and run-tested, but no developer uses this as
their primary development environment.

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

Additionally I'd like to acknowledge the following people who have made contributions:

  * Dave Hylands
  * Zac Berkowitz
