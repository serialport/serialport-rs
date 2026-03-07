// Shows a low-level approach where VMIN, VTIME and raw reads are used for non-blocking reads.
#[cfg(unix)]
fn main() {
    use serialport::{SerialPort, TTYPort};
    use std::io::{self, Write};
    use std::os::unix::prelude::*;
    use std::sync::atomic::AtomicBool;
    use std::time::Duration;

    static MASTER_DONE: AtomicBool = AtomicBool::new(false);

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
            let mut serial_buf: Vec<u8> = vec![0; 1000];
            std::thread::sleep(Duration::from_millis(50));
            master.write(b"hello slave\n").unwrap();
            master.flush().ok();
            loop {
                match master.read(&mut serial_buf) {
                    Ok(t) => {
                        println!("received {t} bytes on master:");
                        assert_eq!(&serial_buf[..t], b"hello master\n");
                        io::stdout().write_all(&serial_buf[..t]).unwrap();
                        io::stdout().flush().unwrap();
                        MASTER_DONE.store(true, std::sync::atomic::Ordering::Relaxed);
                        break;
                    }
                    Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {
                        std::thread::sleep(Duration::from_millis(20));
                    },
                    Err(e) => {
                        panic!("master error: {:?}", e);
                    }
                }
            }
        });
        let jh_slave = s.spawn(move || {
            slave
                .set_read_mode(serialport::ReadMode::Immediate)
                .unwrap();
            let now = std::time::Instant::now();
            let mut serial_buf: Vec<u8> = vec![0; 1000];

            // This is non-blocking.
            let read_result = slave.read_raw(&mut serial_buf);
            assert!(read_result.is_ok(), "non-blocking read should not throw error");
            assert!(now.elapsed() < Duration::from_millis(1), "slave read has blocked");
            assert_eq!(read_result.unwrap(), 0, "no data should be available");

            let mut current_write_index = 0;
            let now = std::time::Instant::now();
            loop {
                match slave.read_raw(&mut serial_buf[current_write_index..]) {
                    Ok(t) => {
                        if t > 0 {
                            current_write_index += t;
                        } else {
                            std::thread::sleep(Duration::from_millis(20));
                        }
                    }
                    Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {
                        // This is the difference to a regular read: Reads will return Ok(0)
                        // instead of this error.
                        panic!("unexpected timeout error");
                    }
                    Err(e) => {
                        panic!("slave error: {:?}", e);
                    }
                }
                if now.elapsed() > Duration::from_millis(100) {
                    println!("received {current_write_index} bytes on slave:");
                    io::stdout()
                        .write_all(&serial_buf[..current_write_index])
                        .unwrap();
                    io::stdout().flush().unwrap();
                    assert_eq!(
                        &serial_buf[0..current_write_index],
                        "hello slave\n".as_bytes()
                    );
                    slave.write(b"hello master\n").unwrap();
                    slave.flush().ok();
                    break;
                }
            }
            // Need to keep the slave alive to avoid pipe errors.
            while !MASTER_DONE.load(std::sync::atomic::Ordering::Relaxed) {
                std::thread::sleep(Duration::from_millis(10));
            }
        });
        jh_master.join().unwrap();
        jh_slave.join().unwrap();
    });
}
