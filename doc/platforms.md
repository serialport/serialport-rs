# Platform API Overview

The three primary platforms have considerable differences in even their basic blocking APIs that are worth outlining in more detail. Many aspects of the cross-platform API came about because of these differences.

## Windows

Surprisingly enough, Windows has the most sane out of all of the interfaces. There is a singular `DCB` struct that contains all configuration necessary for the port. Once a `DCB` struct is created and configured, it's quite easy to configure the port and call `SetCommState()`. Note that this struct supports arbitrary baud rates by default.

The `DCB` struct for a given `HANDLE` can also be retrieved from the `GetCommState()` function.

## Linux & Android

Linux and Android provide both the Termios and Termios2 APIs. The Termios API allows for all configuration necessary, but does not support arbitrary baud rates. In this API the speed members of the struct are not accessible and the `cfsetXspeed()` functions must be used to configure them. And the only appropriate values are the `B*` constants.

The Termios2 API, on the other hand, supports arbitrary baud rates. Instead of using `cfsetXspeed` and the `B*` constants, you can modify the `c_ispeed` and `c_ospeed` fields of the `termios2` struct directly.

## The BSDs (DragonFlyBSD, FreeBSD, NetBSD, OpenBSD)

The BSDs basically **only** have the Termios2 API, but they call it Termios. It supports arbitrary baud rates out of the gate as the `termios2.c_ispeed` and `termios2.c_ospeed` fields are directly settable to the desired baud rate.

### FreeBSD

 * https://www.freebsd.org/doc/handbook/serial.html

### NetBSD

 * http://netbsd.gw.com/cgi-bin/man-cgi?tty+4.i386+NetBSD-8.0
 * http://netbsd.gw.com/cgi-bin/man-cgi?com+4.i386+NetBSD-8.0
 * https://www.netbsd.org/docs/Hardware/Misc/serial.html
 * https://www.netbsd.org/ports/hp300/faq.html

### OpenBSD

 * https://man.openbsd.org/tty.4

## Mac OS X and iOS

While OS X and iOS have the heritage of a BSD, their support is slightly different. In theory, they support arbitrary baud rates in their Termios API much like the BSDs, but in practice this doesn't work with many hardware devices, as it's dependent on driver support. Instead, Apple added the `IOSSIOSPEED` ioctl in OS X 10.4, which can set the baud rate to an arbitrary value. As the oldest OS X version supported by Rust is 10.7, it's available on all Mac platforms.

This API requires the port to be set into raw mode with `cfmakeraw`, and must be done after every call to `tcsetattr`, as that will reset the baud rate. Additionally, there is no way to retrieve the actual baud rate from the OS. This is therefore the clunkiest API of any platform.
