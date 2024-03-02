# Change log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/) and this
project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]
### Added
* Added conversions between `DataBits`, `StopBits` types and their numeric representations
* Added `FromStr` implementation for `FlowControl`
### Changed
### Fixed
* Fixes a bug where `available_ports()` returned disabled devices on Windows.
  [#144](https://github.com/serialport/serialport-rs/pull/144)
* Fixes a bug on Windows where the `WriteTotalTimeoutConstant` field hasn't been
  configured properly when the `set_timeout` method is called.
  [#124](https://github.com/serialport/serialport-rs/issues/124)

### Removed

## [4.3.0] - 2023-12-11

### Changed

* Raise MSRV from 1.56.1 to 1.59.0 and Rust edition from 2018 to 2021.
  [#137](https://github.com/serialport/serialport-rs/pull/137)
* Update `bitflags` dependency to 2.4.0.
  [#127](https://github.com/serialport/serialport-rs/pull/127)
* Open serial devices with `O_CLOEXEC` (Posix). This will close the device
  handles when starting a child process. In particular this means that a serial
  device can be reopened after making SW update of a Tauri application.
  [#130](https://github.com/serialport/serialport-rs/pull/130)
* Prefer USB device manufacturer and model information from the actual USB
  device over the information from udev's database.
  [#137](https://github.com/serialport/serialport-rs/pull/137)

### Fixed
* Fixes a bug on Windows where composite devices would show a incorrect serial
  number.
  [#141](https://github.com/serialport/serialport-rs/pull/141)
* Fixes a bug on Linux without udev where `available_ports()` returned wrong
  device file paths.
  [#122](https://github.com/serialport/serialport-rs/pull/122)
* Fixes a bug on Windows where some USB device serial numbers were truncated.
  [#131](https://github.com/serialport/serialport-rs/pull/131)
* Switches to maintained sys crates for CoreFoundation and IOKit on macOS.
  [#112](https://github.com/serialport/serialport-rs/issues/112),
  [#136](https://github.com/serialport/serialport-rs/pull/136)

## [4.2.2] - 2023-08-03
### Fixed
* Fixes a bug on the Raspberry Pi 4, which results in USB-devices being detected as PCI-devices.
  [#113](https://github.com/serialport/serialport-rs/pull/113)



## [4.2.1] - 2023-05-21
### Added
* Add support for reporting the USB device interface (feature-gated by
  _usbserialinfo-interface_).
  [#47](https://github.com/serialport/serialport-rs/pull/47),
  [#101](https://github.com/serialport/serialport-rs/pull/101)
* Add example for loopback testing with real hardware.
  [#69](https://github.com/serialport/serialport-rs/pull/69)
* Implement `fmt::Debug` and `fmt::Display` for `SerialPort` and related enums.
  [#91](https://github.com/serialport/serialport-rs/pull/91)
### Changed
* Migrated from unmaintainted dependency `mach` to `mach2`.
* Update dependency `nix` from 0.24.1 to 0.26.0 and raise MSRV to 1.56.1.
  [#67](https://github.com/serialport/serialport-rs/pull/67),
  [#75](https://github.com/serialport/serialport-rs/pull/75),
  [#78](https://github.com/serialport/serialport-rs/pull/78)
### Fixed
* Skip attempts to set baud rate 0 on macOS.
  [#58](https://github.com/serialport/serialport-rs/pull/58)
* Fix getting actual result value from `tiocmget`.
  [#61](https://github.com/serialport/serialport-rs/pull/61/files)
* Fix serial number retrieval procedure on macOS.
  [#65](https://github.com/serialport/serialport-rs/pull/65)
* Fix port name retrieval procedure for Unicode names on Windows.
  [#63](https://github.com/serialport/serialport-rs/pull/63)
* Fix compilation for OpenBSD due to missing use declaration.
  [#68](https://github.com/serialport/serialport-rs/pull/68)
* A number of memory leaks have been addressed when using serialport-rs.
  [#98](https://github.com/serialport/serialport-rs/pull/98)

## [4.2.0] - 2022-06-02
### Added
* Add `serde` support behind a feature flag.
  [#51](https://github.com/serialport/serialport-rs/pull/51)
### Changed
* Request exclusive access when opening a POSIX serial port by default.
  [#44](https://github.com/serialport/serialport-rs/pull/44)
* Updated `nix` dependency to 0.24.1 and limited features.
  [#46](https://github.com/serialport/serialport-rs/pull/46)
* Derive the `Clone` trait for `Error`.
  [#53](https://github.com/serialport/serialport-rs/pull/53)
* Enumerate callout devices in addition to dial-in devices on macOS.
  [#54](https://github.com/serialport/serialport-rs/pull/54)
* Revert to edition 2018 to allow for use with older compiler versions.
### Fixed
* Set port timeout to a non-zero value before performing loopback test.
  [#45](https://github.com/serialport/serialport-rs/pull/45)

## [4.1.0] - 2022-04-04
### Added
* impl `SerialPort` for `&mut T`. This allows a `&mut T (where T: SerialPort)`
  to be used in a context where `impl SerialPort` is expected.
  [!114](https://gitlab.com/susurrus/serialport-rs/-/merge_requests/114)
### Changed
* Updated `nix` dependency to 0.23.1.
* Remove useless call to tcflush on open.
  [#40](https://github.com/serialport/serialport-rs/pull/40)
### Fixed
* Improved support for recent versions of macOS.
  [!104](https://gitlab.com/susurrus/serialport-rs/-/merge_requests/104)
* Fix filehandle leak in open() on Windows.
  [#36](https://github.com/serialport/serialport-rs/pull/36)
* Make sure fd is properly closed if initialization fails.
  [#39](https://github.com/serialport/serialport-rs/pull/39)
  [#41](https://github.com/serialport/serialport-rs/pull/41)

## [4.0.1] - 2021-04-17
### Changed
* Update maintenance status to looking for a new maintainer.
### Fixed
* Properly initialize DCB structure on Windows. This fixes some non-functional
  devices.
  [!97](https://gitlab.com/susurrus/serialport-rs/-/merge_requests/97)

## [4.0.0] - 2020-12-17
### Added
* Added `send_break()` to `TTYPort`.
  [!69](https://gitlab.com/susurrus/serialport-rs/merge_requests/69)
* Enable `available_ports()` for Linux musl targets and those without the
  `libudev` feature enabled by scanning `/sys/` for ports.
  [!72](https://gitlab.com/susurrus/serialport-rs/merge_requests/72)
* `ENOENT` and `EACCES` errors are now exposed as `NotFound` and
  `PermissionDenied` errors on Linux.
  [!80](https://gitlab.com/susurrus/serialport-rs/merge_requests/80)
* `try_clone_native()` was added to `COMPort` and `TTYPort` to complement
  `SerialPort::try_clone()` but returning the concrete type instead.
  [!85](https://gitlab.com/susurrus/serialport-rs/merge_requests/85)
* Added `set_break()` and `clear_break()` to `SerialPort`.
  [!70](https://gitlab.com/susurrus/serialport-rs/merge_requests/70)

### Changed
* Minimum supported Rust version is now 1.36.0 to support the `mem::MaybeUninit`
  feature.
* The platform-specific `TTYPort`/`BreakDuration` and `COMPort` are now at the
  root level rather than under the `posix` and `windows` submodules
  respectively.
* Opening `SerialPort` s now uses the builder pattern through
  `serialport::new()`. See the README for concrete examples.
  [!73](https://gitlab.com/susurrus/serialport-rs/merge_requests/73)
* `SerialPorts`s are no longer opened with a default timeout of 1ms.
* Under linux, the `manufacturer` and `product` fields of `UsbPortInfo` now take
  their values from the `ID_VENDOR_FROM_DATABASE` and `ID_MODEL_FROM_DATABASE`
  udev properties respectively, instead of the `ID_VENDOR` and `ID_MODEL`
  properties that were used before. When the `_FROM_DATABASE` values are not
  available, it falls back to the old behavior.
  [!86](https://gitlab.com/susurrus/serialport-rs/merge_requests/86)
* POSIX ports are no longer opened in exclusive mode. After opening they can be
  made exclusive via `TTYPort::set_exclusive()`.
  [!98](https://gitlab.com/susurrus/serialport-rs/merge_requests/98)

### Fixed
* Raised the version specification for `bitflags` to 1.0.4. Previously it was
  set to 1.0.0, but this version of `bitflags` is actually incompatible with
  Rust 2018 style macro imports that `serialport-rs` requires.
  [!83](https://gitlab.com/susurrus/serialport-rs/merge_requests/83)

### Removed
* Removed the `serialport::prelude` module. Types should be explicitly imported
  or can be glob-imported from the root like `use serialport::*`.
  [!82](https://gitlab.com/susurrus/serialport-rs/merge_requests/82)

## [3.3.0] - 2019-06-12
### Added
* Added support for arbitrary baud rates on macOS and iOS.

### Changed
* Minimum supported Rust version is now 1.31 to support using the 2018 edition
  of Rust.

### Fixed
* Upgraded `sparc64-unknown-linux-gnu` to Tier 2 support.

## [3.2.0] - 2019-01-01
### Added
* Port enumeration is now supported on FreeBSD.

### Changed
* Minimum supported Rust version changed to 1.24.1.
* Made `aarch64-unknown-linux-musl` a Tier-2 supported target.

### Fixed
* Fixed software flow control for POSIX systems.
  [!54](https://gitlab.com/susurrus/serialport-rs/merge_requests/54)

### Removed
* Removed support for `x86_64-unknown-linux-gnux32`.

## [3.1.0] - 2018-11-02
### Added
* Added `bytes_to_read()`, `bytes_to_write()`, and `clear()` to `SerialPort`.
  Also added example scripts for using them.
* Added Tier 2 support for:
  * `armv5te-unknown-linux-musleabi`
* Added "libudev" feature to allow for disabling linking to `libudev` on Linux.

## [3.0.0] - 2018-07-14
### Added
* Arbitrary baud rates are now supported on BSDs, Linux, and Windows.
* Added Tier 1 support for `{i586|i686|x86_64}-unknown-linux-musl`.
* Added Tier 2 support for:
  * `{arm|armv7}-linux-androideabi`
  * `i686-linux-android`
  * `{i686|x86_64}-unknown-freebsd`
  * `arm-unknown-linux-musleabi`
  * `armv7-unknown-linux-musleabihf`
  * `{mips64|mips64el}-unknown-linux-gnuabi64`
  * `armv5te-unknown-linux-gnueabi`
  * `{aarch64|mips|mipsel|powerpc64|powerpc64le|powerpc|s390x}-unknown-linux-gnu`
  * `{mips|mipsel}-unknown-linux-musl`
  * `x86_64-unknown-netbsd`
* Added Tier 3 support for:
  * `{aarch64|x86_64}-linux-android`
  * `aarch64-unknown-linux-musl`
  * `sparc64-unknown-linux-gnu`,
  * `x86_64-unknown-linux-gnux32`

### Changed
* Most port configuration methods now return a `Result<()>`.
* Renamed `SerialPort::port_name()` to `name()`.

### Fixed
* On Windows, the `port_name` field on `SerialPortInfo` included an extraneous
  trailing nul byte character.

### Removed
* The `BaudRate` enum was removed in favor of a `u32`.

## [2.3.0] - 2018-03-13
### Added
* Added `examples/hardware_check.rs` for use in debugging library or driver
  issues when using physical serial ports.
* Added `SerialPort::try_clone` which allows for cloning a port for full-duplex
  reading and writing.

### Changed
* Removed configuration caching for serial ports. The underlying implementations
  for all platforms cached a configuration struct so that modifying the port
  settings involved a single switch into kernel space. This has been removed so
  now two system calls are needed for every configuration change. This is
  probably a slight performance regression, but should allow the new
  `SerialPort::try_clone` interface to work as people expect.

### Fixed
* `TTYPort::into_raw_fd` will now work as expected. It previously closed the
  port so the returned file descriptor would be invalid.
* 921600 baud is now supported on NetBSD and FreeBSD.

## 2.2.0 - 2018-03-13
Unreleased, happened due to a user error using `cargo-release`.

## [2.1.0] - 2018-02-14
### Added
* `impl FromRawHandle` for `COMPort`.

### Changed
* Specific IO-related errors are now returned instead of mapping every IO error
  to Unknown. This makes it possible to catch things like time-out errors.
* Changed all baud rates to be reported as the discrete `BaudRate::Baud*` types
  rather than as the `BaudRate::BaudOther(*)` type.

### Fixed
* Modem-type USB serial devices are now enumerated on macOS. This now allows
  connected Arduinos to be detected.
* Compilation on FreeBSD and NetBSD was fixed by removing the 921600 baud rates.
  These will be re-added in a future release.

## [2.0.0] - 2017-12-18
### Added
* USB device information is now returned in calls to `available_ports()`.
* Serial port enumeration is now supported on Mac.
* Serial port enumeration now attempts to return the interface used for the port
  (USB, PCI, Bluetooth, Unknown).
* `BaudRate::standard_rates()` provides a vector of cross-platform baud rates.
* `SerialPort` trait is now `Send`.

### Changed
* Software license has changed from LGPLv3+ to MPL-2.0. This makes it possible
  to use this library in any Rust project if it's unmodified.
* Mac is now a Tier 2 supported platform.
* Removed `BaudRate::from_speed(usize)` and `BaudRate::speed -> usize` in favor
  of the `From<u32>` and `Into<u32>` traits.
* Removed `available_baud_rates` in favor of `BaudRate::platform_rates()` as
  this has a more clear semantic meaning. The returned list of baud rates is now
  also correct for all supported platforms.
* Removed `termios` dependency in favor of `nix`. This is a big step towards
  supporting additional platforms.

### Fixed
* Stop bits are now specified properly (had been reversed). Thanks to
  @serviushack. (MR#9)
* `TTYPort::pair()` is now thread-safe.
* `TTYPort::open()` no longer leaks file descriptors if it errors. Thanks to
  @daniel. (MR#12)
* Fixed compilation when targeting Android.

## [1.0.1] - 2017-02-20
### Fixed
* `read()` now properly blocks for at least one character.
* Compilation now works on Mac.

## [1.0.0] - 2017-02-13
### Changed
* Various documentation/README updates.
* Minor formatting fixes (from rustfmt).

### Fixed
* Platform-specific examples are now only built on appropriate platforms.

## [0.9.0] - 2017-02-09
### Added
* `impl Debug` for `COMPort`.
* `exclusive()` and `set_exclusive()` for `TTYPort`.
* `port_name()` for `SerialPort`.
* `impl FromRawFd` and `impl IntoRawFd` for `TTYPort`.
* `pair()` for `TTYPort`.

## [0.3.0] - 2017-01-28
### Added
* `open_with_settings()` to support initializing the port with custom settings.
* `SerialPortSettings` is now publically usable being exported in the prelude,
  having all public and commented fields, and a `Default` impl.

### Changed
* `TTYPort/COMPort::open()` now take a `SerialPortSettings` argument and return
  concrete types.
* `serialport::open()` now initializes the port to reasonable defaults.
* Removed all instances of `try!()` for `?`.
* `SerialPort::set_all()` now borrows `SerialPortSettings`.

## [0.2.4] - 2017-01-26
### Added
* Report an Unimplemented error for unsupported unix targets.

### Changed
* Minor changes suggested by Clippy.
* Reworked Cargo.toml to more easily support additional targets.

### Fixed
* AppVeyor badge should now be properly displayed.

## [0.2.3] - 2017-01-21
### Added
* Specify AppVeyor build status badge for crates.io.

## [0.2.2] - 2017-01-21
* No changes, purely a version increment to push new crate metadata to
  crates.io.

## [0.2.1] - 2017-01-21
### Added
* Specify category for crates.io.

## [0.2.0] - 2017-01-07
### Added
* Added a changelog.
* Added a getter/setter pair for all settings at once.
* An error is thrown if settings weren't correctly applied on POSIX.

## [0.1.1] - 2016-12-23
### Changed
* Fixed compilation on x86_64-pc-windows-gnu target.
* Added contributors to README.
* Clarified license terms in the README.

## [0.1.0] - 2016-12-22
### Added
* Initial release.


[Unreleased]: https://github.com/serialport/serialport-rs/compare/v4.3.0...HEAD
[4.3.0]: https://github.com/serialport/serialport-rs/compare/v4.2.2...v4.3.0
[4.2.2]: https://github.com/serialport/serialport-rs/compare/v4.2.1...v4.2.2
[4.2.1]: https://github.com/serialport/serialport-rs/compare/v4.2.0...v4.2.1
[4.2.0]: https://github.com/serialport/serialport-rs/compare/v4.1.0...v4.2.0
[4.1.0]: https://github.com/serialport/serialport-rs/compare/v4.0.1...v4.1.0
[4.0.1]: https://github.com/serialport/serialport-rs/compare/v4.0.0...v4.0.1
[4.0.0]: https://github.com/serialport/serialport-rs/compare/v3.3.0...v4.0.0
[3.3.0]: https://github.com/serialport/serialport-rs/compare/v3.2.0...v3.3.0
[3.2.0]: https://github.com/serialport/serialport-rs/compare/v3.1.0...v3.2.0
[3.1.0]: https://github.com/serialport/serialport-rs/compare/v3.0.0...v3.1.0
[3.0.0]: https://github.com/serialport/serialport-rs/compare/v2.3.0...v3.0.0
[2.3.0]: https://github.com/serialport/serialport-rs/compare/v2.1.0...v2.3.0
[2.1.0]: https://github.com/serialport/serialport-rs/compare/v2.0.0...v2.1.0
[2.0.0]: https://github.com/serialport/serialport-rs/compare/v1.0.1...v2.0.0
[1.0.1]: https://github.com/serialport/serialport-rs/compare/v1.0.0...v1.0.1
[1.0.0]: https://github.com/serialport/serialport-rs/compare/v0.9.0...v1.0.0
[0.9.0]: https://github.com/serialport/serialport-rs/compare/v0.3.0...v0.9.0
[0.3.0]: https://github.com/serialport/serialport-rs/compare/v0.2.4...v0.3.0
[0.2.4]: https://github.com/serialport/serialport-rs/compare/v0.2.3...v0.2.4
[0.2.3]: https://github.com/serialport/serialport-rs/compare/v0.2.2...v0.2.3
[0.2.2]: https://github.com/serialport/serialport-rs/compare/v0.2.1...v0.2.2
[0.2.1]: https://github.com/serialport/serialport-rs/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/serialport/serialport-rs/compare/v0.1.1...v0.2.0
[0.1.1]: https://github.com/serialport/serialport-rs/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/serialport/serialport-rs/releases/tag/v0.1.0
