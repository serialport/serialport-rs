# serialport-rs

Serial ports are some of the oldest external interfaces exposed by system kernels and their interfaces are quite clumsy. Additionally there are plenty of caveats in each platform's interface that we attempt to hide through the safe cross-platform interface. Many details of these interfaces are not well documented and so the following resources are an attempt to remedy that situation. They also serve as helpful developer documentation for `serialport-rs`.

## Resources

[Platform API overview](./platforms.md)

[Developer notes](./dev_notes.md)

## References

 * <https://github.com/pyserial/pyserial/blob/master/serial/serialposix.py#L354>
 * <https://github.com/Fazecast/jSerialComm/blob/v2.0.2/src/main/c/OSX/SerialPort_OSX.c#L241>
 * <https://github.com/node-serialport/node-serialport/blob/a31078f054f23c47e9de7cc8b1c7e79d6e4e6c0c/src/serialport_unix.cpp#L137>
 * <https://github.com/jacobsa/go-serial/blob/master/serial/open_darwin.go#L239>
 * <https://github.com/nyholku/purejavacomm/blob/master/src/jtermios/macosx/JTermiosImpl.java#L434>
 * <https://github.com/npat-efault/picocom/blob/master/termios2.txt>
