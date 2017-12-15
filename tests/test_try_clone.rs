#![cfg(unix)]
extern crate serialport;

use std::io::{Read,Write,ErrorKind};
use serialport::posix::TTYPort;
use serialport::SerialPort;

#[test]
fn test_try_clone() {
    use std::{mem, thread};

    let (mut master, mut slave) = TTYPort::pair().expect("Unable to create ptty pair");

    // Slave tty is just used to forward what the master tty sends back to it.
    // So we can clone master tty and test that we can simultaneously read and write
    // to it and check that the values are correct.
    let loopback = thread::spawn(move || {
        let mut buffer: [u8; 10] = unsafe { mem::uninitialized() };

        // loops until the serial tty is closed
        loop {
            match slave.read(&mut buffer) {
                Ok(bytes) => if bytes > 0 {slave.write(&buffer[0..bytes]).unwrap();} else { break; },
                Err(ref e)  => {assert_eq!(e.kind(),ErrorKind::ConnectionAborted);},
            }
        }
    });

    // Create clone in an inner scope to test that dropping a clone do not
    // shuts off the serial device.
    {
        let mut clone = master.try_clone().expect("Failed to clone");
        let writes = thread::spawn(move || {
            let bytes = [b'a', b'b', b'c', b'd', b'e', b'f'];
            clone.write(&bytes).unwrap();
        });
        let mut buffer = [0; 6];
        master.read_exact(&mut buffer).unwrap();
        writes.join().unwrap();
        assert_eq!(buffer, [b'a', b'b', b'c', b'd', b'e', b'f']);
    }

    // Second try to check that the serial device is not closed
    {
        let mut clone = master.try_clone().expect("Failed to clone");
        let writes = thread::spawn(move || {
            let bytes = [b'g', b'h', b'i', b'j', b'k', b'l'];
            clone.write(&bytes).unwrap();
        });
        let mut buffer = [0; 6];
        master.read_exact(&mut buffer).unwrap();
        writes.join().unwrap();
        assert_eq!(buffer, [b'g', b'h', b'i', b'j', b'k', b'l']);
    }

    // Check that once all reference to the master tty are gone, the
    // serial device is closed and the loopback thread should finish
    mem::drop(master);
    loopback.join().unwrap();
}

