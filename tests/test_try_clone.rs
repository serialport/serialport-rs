#![cfg(unix)]
extern crate serialport;

use serialport::{SerialPort, TTYPort};
use std::io::{Read, Write};

// Test that cloning a port works as expected
#[test]
fn test_try_clone() {
    let (master, mut slave) = TTYPort::pair().expect("Unable to create ptty pair");

    // Create the clone in an inner scope to test that dropping a clone doesn't close the original
    // port
    {
        let mut clone = master.try_clone().expect("Failed to clone");
        let bytes = [b'a', b'b', b'c', b'd', b'e', b'f'];
        clone.write_all(&bytes).unwrap();
        let mut buffer = [0; 6];
        slave.read_exact(&mut buffer).unwrap();
        assert_eq!(buffer, [b'a', b'b', b'c', b'd', b'e', b'f']);
    }

    // Second try to check that the serial device is not closed
    {
        let mut clone = master.try_clone().expect("Failed to clone");
        let bytes = [b'g', b'h', b'i', b'j', b'k', b'l'];
        clone.write_all(&bytes).unwrap();
        let mut buffer = [0; 6];
        slave.read_exact(&mut buffer).unwrap();
        assert_eq!(buffer, [b'g', b'h', b'i', b'j', b'k', b'l']);
    }
}

// Test moving a cloned port into a thread
#[test]
fn test_try_clone_move() {
    use std::thread;

    let (master, mut slave) = TTYPort::pair().expect("Unable to create ptty pair");

    let mut clone = master.try_clone().expect("Failed to clone the slave");
    let loopback = thread::spawn(move || {
        let bytes = [b'a', b'b', b'c', b'd', b'e', b'f'];
        clone.write_all(&bytes).unwrap();
    });

    let mut buffer = [0; 6];
    slave.read_exact(&mut buffer).unwrap();
    assert_eq!(buffer, [b'a', b'b', b'c', b'd', b'e', b'f']);

    // The thread should have already ended, but we'll make sure here anyways.
    loopback.join().unwrap();
}
