How to test `serialport-rs` for development.

Without hardware:

 1. Compilation
 2. `cargo test`

With a single unconnected device:

`cargo run --example hardware_check <DEVICE>`

And when wired in a physical loopback mode:

`cargo run --example hardware_check <DEVICE> --loopback`

With two devices connected to each other:

 * `cargo run --example hardware_check <DEVICE1> --loopback-port <DEVICE2>`
 * Also `cargo run --example heartbeat <DEVICE1> <BAUD>` in one terminal and
   `cargo run --example receive_data <DEVICE2> <BAUD>` in another
 * Running tests with test cases requiring hardware devices enabled:
     ```
     $ export SERIALPORT_TEST_PORT_1=$(realpath /dev/ttyX)
     $ export SERIALPORT_TEST_PORT_2=$(realpath /dev/ttyY)
     $ cargo test --features hardware-tests
     ```

Can also verify trickier settings (like non-standard baud rates) using serial terminal programs
like:

  * `screen` (POSIX)
  * [CoolTerm](http://freeware.the-meiers.org/) (macOS)
  * [RealTerm](https://sourceforge.net/projects/realterm/) (Windows)
