//! Duplex example
//!
//! This example tests the ability to clone a serial port. It works by creating
//! a new file descriptor, and therefore a new `SerialPort` object that's safe
//! to send to a new thread.
//!
//! This example selects the first port on the system, clones the port into a child
//! thread that writes data to the port every second. While this is running the parent
//! thread continually reads from the port.
//!
//! To test this, have a physical or virtual loopback device connected as the
//! only port in the system.

use std::io::Write;
use std::time::Duration;
use std::{io, thread};

fn main() {
    // Open the first serialport available.
    let port_name = &serialport::available_ports().expect("No serial port")[0].port_name;
    let mut port = serialport::new(port_name, 9600)
        .open()
        .expect("Failed to open serial port");

    // Clone the port
    let mut clone = port.try_clone().expect("Failed to clone");

    // Send out 4 bytes every second
    thread::spawn(move || loop {
        clone
            .write_all(&[5, 6, 7, 8])
            .expect("Failed to write to serial port");
        thread::sleep(Duration::from_millis(1000));
    });

    // Read the four bytes back from the cloned port
    let mut buffer: [u8; 1] = [0; 1];
    loop {
        match port.read(&mut buffer) {
            Ok(bytes) => {
                if bytes == 1 {
                    println!("Received: {:?}", buffer);
                }
            }
            Err(ref e) if e.kind() == io::ErrorKind::TimedOut => (),
            Err(e) => eprintln!("{:?}", e),
        }
    }
}
