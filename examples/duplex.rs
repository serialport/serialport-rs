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

extern crate serialport;

use serialport::{available_ports, open};
use std::io;
use std::io::Write;
use std::{mem, thread};
use std::time::Duration;

fn main() {

    // Open the first serialport available.
    let mut serialport = open(&available_ports().expect("No serial port")[0].port_name)
        .expect("Failed to open serial port");

    // Clone the port
    let mut clone = serialport.try_clone().expect("Failed to clone");

    // Send out 4 bytes every second
    thread::spawn(move || {
        loop {
            clone.write(&[5, 6, 7, 8]).expect("Failed to write to serial port");
            thread::sleep(Duration::from_millis(1000));
        }
    });

    // Read the four bytes back from the cloned port
    let mut buffer: [u8; 1] = unsafe { mem::uninitialized() };
    loop {
        match serialport.read(&mut buffer) {
            Ok(bytes) => if bytes == 1 { println!("Received: {:?}",buffer); },
            Err(ref e) if e.kind() == io::ErrorKind::TimedOut => (),
            Err(e) => eprintln!("{:?}",e),
        }
    }
}

