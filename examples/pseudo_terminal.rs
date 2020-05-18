//! Pseudo terminal example.

#[cfg(unix)]
fn main() {
    use std::io::{Read, Write};
    use std::os::unix::prelude::*;
    use std::str;
    use std::thread;
    use std::time;

    use serialport::{SerialPort, TTYPort};

    let (mut master, mut slave) = TTYPort::pair().expect("Unable to create pseudo-terminal pair");

    // Master ptty has no associated path on the filesystem.
    println!(
        "Master ptty fd: {}, path: {:?}",
        master.as_raw_fd(),
        master.name()
    );
    println!(
        "Slave  ptty fd: {}, path: {:?}",
        slave.as_raw_fd(),
        slave.name()
    );

    // Receive buffer.
    let mut buf = [0u8; 512];

    println!("Sending 5 messages from master to slave.");

    // Send 5 messages.
    for x in 1..6 {
        let msg = format!("Message #{}", x);

        // Send the message on the master
        assert_eq!(master.write(msg.as_bytes()).unwrap(), msg.len());

        // Receive on the slave
        let bytes_recvd = slave.read(&mut buf).unwrap();
        assert_eq!(bytes_recvd, msg.len());

        let msg_recvd = str::from_utf8(&buf[..bytes_recvd]).unwrap();
        assert_eq!(msg_recvd, msg);

        println!("Slave Rx:  {}", msg_recvd);
        thread::sleep(time::Duration::from_secs(1));
    }
}

#[cfg(not(unix))]
fn main() {}
