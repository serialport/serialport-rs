# Mac OS X / iOS

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

## Reference implementations

 * [picocom](https://github.com/npat-efault/picocom) follows the second approach. However they also
cache the existing `termios` struct.

# Additional References

 * [Understanding UNIX termios VMIN and VTIME](http://unixwiz.net/techtips/termios-vmin-vtime.html)
