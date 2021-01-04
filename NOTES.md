This document outlines some of the fundamental design decisions behind this library. Especially
those that are steeped in low-level system API archana.

# Cross-platform API

This library focuses on a cross-platform API that allows for trivial reading and writing of data
along UART and RS232 connections. The focus is on optimizing for the 90% case, however steps and
workarounds are included to support edge cases where they're found. Serial ports are a tricky
business, so it's not guaranteed the approach here is the best one, nor is it the goal of this
library to support 100% of all communication paradigms. Instead this library focuses on enabling
reading and writing binary data over UART/RS232 ports. Some of the additional control signals and
settings are supported in addition to that.

While APIs on platforms vary, the most common approaches rely on reading using any of the following:

 * Return immediately with up to X characters
 * Wait until at least X characters are available and then return (possibly waiting forever)
 * Return up to X characters timing out after Y milliseconds

Writing is even simpler, as only two approaches are used by applications:

 * Return immediately writing up to X characters
 * Write up to X characters timing out after Y milliseconds

However not all of these approaches are natively supported by system serial port APIs and so they
are manually implemented on top of them.

# Windows

Windows has two different APIs: COM and winrt. Rust targets the former in most *-windows-* targets
while it targets winrt in *-uwp-windows-*. Support for UWP and WinRT is being tracked in #48.

Port configuration is done via the [DCB][1] struct.

COM uses the [COMMTIMEOUTS][2] struct to configure timeouts for both read and write modes. These can
be used to implement the following modes:

| COMMTIMEOUTS                                                                                                  | Behavior                                                              |
|---------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------|
| `ReadIntervalTimeout = MAXDWORD`, `ReadTotalTimeoutMultiplier = 0`, and `ReadTotalTimeoutConstant = 0`        | Return immediately with 0 <= chars <= X                               |
| `ReadIntervalTimeout = MAXDWORD`, `ReadTotalTimeoutMultiplier = MAXDWORD`, and `ReadTotalTimeoutConstant > 0` | Return 0 <= chars <= X timing out after `ReadTotalTimeoutConstant` ms |
| `ReadIntervalTimeout = 0`, `ReadTotalTimeoutMultiplier = 0`, and `ReadTotalTimeoutConstant = 0`               | Block until X chars are received                                      |

[1]: https://docs.microsoft.com/en-us/windows/win32/api/winbase/ns-winbase-dcb
[2]: https://docs.microsoft.com/en-us/windows/win32/api/winbase/ns-winbase-commtimeouts

This API matches the cross-platform one used by this library, so it's used directly for managing
timeouts.

# POSIX (Linux, BSDs, and Mac)

Blocking and non-blocking reads can be implemented in three major ways:

* Rely on the POSIX `termios` API
* Use `select`/`pselect``poll`/`ppoll`
* Implement it manually

## Timeouts

Timeouts for Linux are...complicated. There are several ways they can be implemented. `termios` supports
various timeouts and there are the `poll`/`ppoll` APIs as well.

This library ignores newlines in that data isn't processed "by line". Instead data is only processed
in sequences of bytes. An application is expected to do any buffering or parsing of the data
including newlines. This is referred to as "non-canonical mode" in POSIX documentation. We'll only
discuss non-canonical mode here.

## Reading

To configure a non-blocking interface, set the `O_NONBLOCK` flag. According to the [termios man 
pages](https://manpages.debian.org/buster/manpages-dev/termios.3.en.html#Canonical_and_noncanonical_mode):

> POSIX does not specify whether the setting of the O_NONBLOCK file status flag takes precedence
> over the MIN and TIME settings. If O_NONBLOCK is set, a read(2) in noncanonical mode may return
> immediately, regardless of the setting of MIN or TIME. Furthermore, if no data is available,
> POSIX permits a read(2) in noncanonical mode to return either 0, or -1 with errno set to EAGAIN.

Therefore this library chooses to not set these parameters and instead sets VMIN and VTIME to 0.
`read(2)` should return immediately with whatever bytes are available ≤ X. ***TODO***

## `poll()` loop

To implement reading and writing and ensuring that all data is written, it'd be possible to do this
by being in a `while()` loop until the timeout, or number of characters, are properly written.
However, on POSIX systems, there is the `select/pselect` and `poll/ppoll` interfaces. These let you
block on availability of a file descriptor until it's available for reading and writing. And since
this uses a kernel API, it prevents the application from burning CPU cycles attempting `read`s or
`write`s.

`select` and `pselect` are older versions of this kind of API and have limitations as to how many
events you can wait on. By many people's opinions on POSIX APIs, they're effectively deprecated now
that the `poll`/`ppoll` APIs exist. We don't do any `signet` masking, don't need better than ms
timing granularity, and `poll` is supported on all POSIX platforms, so this library uses `poll`.
The algorithm is roughly:

```
input: characters to be read c,
       timeout in ms t
output: characters that were read

chars_left_to_read ← chars_to_read
time_left ← timeout
while chars_left_to_read > 0 && timeout && time_left > 0 do
    poll(time_left)
    chars_left_to_read = chars_left_to_read - read(chars_left_to_read)
    time_left = time_left - (now() - realtime_clock())
    realtime_clock = now()

return chars_read
```

## References

* [Understanding UNIX termios VMIN and VTIME](http://unixwiz.net/techtips/termios-vmin-vtime.html)
* [Serial Programming Guide for POSIX Operating Systems](https://www.cmrr.umn.edu/~strupp/serial.html)

# Arbitrary baud rates on Mac OS X & iOS

Macs use the usual `termios` TTY implementation that other POSIXes use, but support non-standard
baud rates through the `iossiospeed` ioctl (as of OS X 10.4). To support non-standard baud rates on
Mac, there are three main approaches:

1. Always use `iossiospeed`
2. Use `iossiospeed` for non-standard bauds, but `termios` with standard bauds
3. Use `iossiospeed` always by default and fail-over to the termios approach

## Implementation notes

This library uses the first approach. Given that OS X as far back as 10.4 supports it (2005), there
seem to be no downsides. Internally, baud rates within the `termios` struct are kept at 9600 when
that struct is read & written. This means that anytime the `termios` struct is written back (using
`tcsetattr` a call to `iossiospeed` follows it. Additionally, the `termios` struct is not cached and
instead retrieved on every settings adjustment. While this can increase the number of system calls
when changing port settings, it removes the need to keep state consistent and instead the kernel's
state can always be considered the canonical source.

## Platform notes

`iossiospeed` has no official documentation that can be found by searching
https://developer.apple.com. However [IOSerialTestLib.c](https://opensource.apple.com/source/IOSerialFamily/IOSerialFamily-93/tests/IOSerialTestLib.c.auto.html)
can be found on Apple's open source code repository and has some example code for using this API.

Experimentation has shown that there are a few key features to using `iossiospeed`:

* `iossiospeed` should be called after setting the `termios` struct via `tcsetattr` as that resets
  the baud rate and you cannot put custom baud rates in the `termios` struct.
* Calling `iossiospeed` will modify the `termios` struct in the kernel such that you can no longer
  round-trip the `termios` struct. The following code will fail:
  
  ```C
  struct termios t;
  tcgetattr(fd, &t);
  tcsetattr(fd, TCSANOW, &t)
  ```

# Reference Implementations

Cross-platform serial libraries:

 * [pyserial](https://github.com/pyserial/pyserial) for Python
 * [libserialport](https://sigrok.org/wiki/Libserialport) for C
 * [QSerialPort](https://doc.qt.io/qt-5/qserialport.html) for C++ (part of Qt)

Serial programs:

 * [picocom](https://github.com/npat-efault/picocom) follows the second approach. However they also
  cache the existing `termios` struct.
