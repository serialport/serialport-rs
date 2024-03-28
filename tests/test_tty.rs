//! Tests for the `posix::TTYPort` struct.
#![cfg(unix)]

extern crate serialport;

use std::io::{Read, Write};
use std::os::unix::prelude::*;
use std::str;
use std::time::Duration;

use serialport::{SerialPort, TTYPort};

#[test]
fn test_ttyport_pair() {
    // FIXME: Create a mutex across all tests for using `TTYPort::pair()` as it's not threadsafe
    let (mut master, mut slave) = TTYPort::pair().expect("Unable to create ptty pair");
    master
        .set_timeout(Duration::from_millis(10))
        .expect("Unable to set timeout on the master");
    slave
        .set_timeout(Duration::from_millis(10))
        .expect("Unable to set timeout on the slave");

    // Test file descriptors.
    assert!(
        master.as_raw_fd() > 0,
        "Invalid file descriptor on master ptty"
    );
    assert!(
        slave.as_raw_fd() > 0,
        "Invalid file descriptor on slae ptty"
    );
    assert_ne!(
        master.as_raw_fd(),
        slave.as_raw_fd(),
        "master and slave ptty's share the same file descriptor."
    );

    let msg = "Test Message";
    let mut buf = [0u8; 128];

    // Write the string on the master
    let nbytes = master
        .write(msg.as_bytes())
        .expect("Unable to write bytes.");
    assert_eq!(
        nbytes,
        msg.len(),
        "Write message length differs from sent message."
    );

    // Read it on the slave
    let nbytes = slave.read(&mut buf).expect("Unable to read bytes.");
    assert_eq!(
        nbytes,
        msg.len(),
        "Read message length differs from sent message."
    );

    assert_eq!(
        str::from_utf8(&buf[..nbytes]).unwrap(),
        msg,
        "Received message does not match sent"
    );
}

#[test]
fn test_ttyport_timeout() {
    let result = std::sync::Arc::new(std::sync::Mutex::new(None));
    let result_thread = result.clone();

    std::thread::spawn(move || {
        // FIXME: Create a mutex across all tests for using `TTYPort::pair()` as it's not threadsafe
        let (mut master, _slave) = TTYPort::pair().expect("Unable to create ptty pair");
        master.set_timeout(Duration::new(1, 0)).unwrap();

        let mut buffer = [0u8];
        let read_res = master.read(&mut buffer);

        *result_thread.lock().unwrap() = Some(read_res);
    });

    std::thread::sleep(std::time::Duration::new(2, 0));

    let read_res = result.lock().unwrap();
    match *read_res {
        Some(Ok(_)) => panic!("Received data without sending"),
        Some(Err(ref e)) => assert_eq!(e.kind(), std::io::ErrorKind::TimedOut),
        None => panic!("Read did not time out"),
    }
}

#[test]
#[cfg(any(target_os = "ios", target_os = "macos"))]
fn test_osx_pty_pair() {
    #![allow(unused_variables)]
    let (mut master, slave) = TTYPort::pair().expect("Unable to create ptty pair");
    let (output_sink, output_stream) = std::sync::mpsc::sync_channel(1);
    let name = slave.name().unwrap();

    master.write_all("12".as_bytes()).expect("");

    let reader_thread = std::thread::spawn(move || {
        let mut port = TTYPort::open(&serialport::new(&name, 0)).expect("unable to open");
        let mut buffer = [0u8; 2];
        let amount = port.read_exact(&mut buffer);
        output_sink
            .send(String::from_utf8(buffer.to_vec()).expect("buffer not read as valid utf-8"))
            .expect("unable to send from thread");
    });

    reader_thread.join().expect("unable to join sink thread");
    assert_eq!(output_stream.recv().unwrap(), "12");
}

// On Mac this should work (in fact used to in b77768a) but now fails. It's not functionality that
// should be required, and the ptys work otherwise. So going to just disable this test instead.
#[test]
#[cfg_attr(any(target_os = "ios", target_os = "macos"), ignore)]
fn test_ttyport_set_standard_baud() {
    // `master` must be used here as Dropping it causes slave to be deleted by the OS.
    // TODO: Convert this to a statement-level attribute once
    //       https://github.com/rust-lang/rust/issues/15701 is on stable.
    // FIXME: Create a mutex across all tests for using `TTYPort::pair()` as it's not threadsafe
    #![allow(unused_variables)]
    let (master, mut slave) = TTYPort::pair().expect("Unable to create ptty pair");

    slave.set_baud_rate(9600).unwrap();
    assert_eq!(slave.baud_rate().unwrap(), 9600);
    slave.set_baud_rate(57600).unwrap();
    assert_eq!(slave.baud_rate().unwrap(), 57600);
    slave.set_baud_rate(115_200).unwrap();
    assert_eq!(slave.baud_rate().unwrap(), 115_200);
}

// On mac this fails because you can't set nonstandard baud rates for these virtual ports
#[test]
#[cfg_attr(
    any(
        target_os = "ios",
        all(target_os = "linux", target_env = "musl"),
        target_os = "macos"
    ),
    ignore
)]
fn test_ttyport_set_nonstandard_baud() {
    // `master` must be used here as Dropping it causes slave to be deleted by the OS.
    // TODO: Convert this to a statement-level attribute once
    //       https://github.com/rust-lang/rust/issues/15701 is on stable.
    // FIXME: Create a mutex across all tests for using `TTYPort::pair()` as it's not threadsafe
    #![allow(unused_variables)]
    let (master, mut slave) = TTYPort::pair().expect("Unable to create ptty pair");

    slave.set_baud_rate(10000).unwrap();
    assert_eq!(slave.baud_rate().unwrap(), 10000);
    slave.set_baud_rate(60000).unwrap();
    assert_eq!(slave.baud_rate().unwrap(), 60000);
    slave.set_baud_rate(1_200_000).unwrap();
    assert_eq!(slave.baud_rate().unwrap(), 1_200_000);
}
