// Shows a low-level approach where VMIN, VTIME and raw reads are used for blocking reads.
#[cfg(unix)]
fn main() {
    use serialport::{SerialPort, TTYPort};
    use std::io::{self, Write};
    use std::os::unix::prelude::*;
    use std::time::Duration;

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

    std::thread::scope(|s| {
        let jh_master = s.spawn(move || {
            std::thread::sleep(Duration::from_millis(100));
            master.write(b"hello slave\n").unwrap();
            let mut serial_buf: Vec<u8> = vec![0; 1000];
            loop {
                match master.read_raw(serial_buf.as_mut_slice()) {
                    Ok(t) => {
                        if t == 0 {
                            std::thread::sleep(Duration::from_millis(100));
                            continue;
                        }

                        println!("received {t} bytes on master:");
                        io::stdout().write_all(&serial_buf[..t]).unwrap();
                        io::stdout().flush().unwrap();
                        assert_eq!(&serial_buf[..t], b"hello master\n");
                        break;
                    }
                    Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {
                        panic!("unexpected timeout error");
                    }
                    Err(e) => eprintln!("{:?}", e),
                }
            }
        });
        let jh_slave = s.spawn(move || {
            slave
                .set_read_mode(serialport::ReadMode::Blocking {
                    inter_byte_timeout: 10,
                    minimal_bytes: 1,
                })
                .unwrap();
            let mut serial_buf: Vec<u8> = vec![0; 1000];
            let now = std::time::Instant::now();
            loop {
                match slave.read_raw(serial_buf.as_mut_slice()) {
                    Ok(t) => {
                        let elapsed = now.elapsed();
                        // The read is delayed by at least 100ms.
                        assert!(
                            elapsed >= Duration::from_millis(100),
                            "read returned too quickly: {:?}",
                            elapsed
                        );
                        println!("received {t} bytes on slave:");
                        io::stdout().write_all(&serial_buf[..t]).unwrap();
                        io::stdout().flush().unwrap();
                        assert_eq!(&serial_buf[..t], b"hello slave\n");
                        slave.write(b"hello master\n").unwrap();
                        slave.flush().ok();
                        break;
                    }
                    Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {
                        panic!("unexpected timeout error");
                    }
                    Err(e) => eprintln!("{:?}", e),
                }
            }
        });
        jh_master.join().unwrap();
        jh_slave.join().unwrap();
    });
}
