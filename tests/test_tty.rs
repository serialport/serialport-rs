//! Tests for the `posix::TTYPort` struct.
#![cfg(unix)]

extern crate serialport;

use std::os::unix::prelude::*;
use std::io::{Read, Write};
use std::str;

use serialport::posix::TTYPort;

#[test]
fn test_ttyport_pair() {
    let (mut master, mut slave) = TTYPort::pair().expect("Unable to create ptty pair");

    // Test file descriptors.
    assert!(master.as_raw_fd() > 0,
            "Invalid file descriptor on master ptty");
    assert!(slave.as_raw_fd() > 0,
            "Invalid file descriptor on slae ptty");
    assert!(master.as_raw_fd() != slave.as_raw_fd(),
            "master and slave ptty's share the same file descriptor.");

    let msg = "Test Message";
    let mut buf = [0u8; 128];

    // Write the string on the master
    assert!(master.write(msg.as_bytes()).unwrap() == msg.len(),
            "Unable to write message on master.");

    // Read it on the slave
    let nbytes = slave.read(&mut buf).expect("Unable to read bytes.");
    assert!(nbytes == msg.len(),
            "Read message length differs from sent message.");

    assert!(str::from_utf8(&buf[..nbytes]).unwrap() == msg,
            "Received message does not match sent");
}
