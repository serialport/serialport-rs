//! Tests for the `posix::TTYPort` struct.
#![cfg(unix)]

extern crate serialport;

use std::os::unix::prelude::*;
use std::io::{Read, Write};
use std::str;

use serialport::{BaudRate, SerialPort};
use serialport::posix::TTYPort;

#[test]
fn test_ttyport_pair() {
    let (mut master, mut slave) = TTYPort::pair().expect("Unable to create ptty pair");

    // Test file descriptors.
    assert!(master.as_raw_fd() > 0,
            "Invalid file descriptor on master ptty");
    assert!(slave.as_raw_fd() > 0,
            "Invalid file descriptor on slae ptty");
    assert_ne!(master.as_raw_fd(),
               slave.as_raw_fd(),
               "master and slave ptty's share the same file descriptor.");

    let msg = "Test Message";
    let mut buf = [0u8; 128];

    // Write the string on the master
    assert_eq!(master.write(msg.as_bytes()).unwrap(),
               msg.len(),
               "Unable to write message on master.");

    // Read it on the slave
    let nbytes = slave.read(&mut buf).expect("Unable to read bytes.");
    assert_eq!(nbytes,
               msg.len(),
               "Read message length differs from sent message.");

    assert_eq!(str::from_utf8(&buf[..nbytes]).unwrap(),
               msg,
               "Received message does not match sent");
}

#[test]
fn test_ttyport_set_standard_baud() {
    // `master` must be used here as Dropping it causes slave to be deleted by the OS.
    // TODO: Convert this to a statement-level attribute once https://github.com/rust-lang/rust/issues/15701
    //       is on stable.
    #![allow(unused_variable)]
    let (master, mut slave) = TTYPort::pair().expect("Unable to create ptty pair");

    slave.set_baud_rate(BaudRate::Baud9600).unwrap();
    assert_eq!(slave.baud_rate().unwrap(), BaudRate::Baud9600);
    slave.set_baud_rate(BaudRate::Baud57600).unwrap();
    assert_eq!(slave.baud_rate().unwrap(), BaudRate::Baud57600);
    slave.set_baud_rate(BaudRate::Baud115200).unwrap();
    assert_eq!(slave.baud_rate().unwrap(), BaudRate::Baud115200);
}
