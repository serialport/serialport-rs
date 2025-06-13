mod config;

use config::{hw_config, HardwareConfig};
use rstest::rstest;
use serialport::*;
use std::io::Read;
use std::thread;
use std::time::{Duration, Instant};

#[rstest]
#[case(1, Vec::from(b"abcdef"))]
#[case(
    20,
    Vec::from(b"0123456789:;<=>?@abcdefghijklmnopqrstuvwxyz[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~")
)]
#[cfg_attr(not(feature = "hardware-tests"), ignore)]
fn test_read_returns_available_data_before_timeout(
    hw_config: HardwareConfig,
    #[case] chunk_size: usize,
    #[case] message: Vec<u8>,
) {
    let send_period = Duration::from_millis(500);
    let receive_timeout = Duration::from_millis(3000);
    let marign = Duration::from_millis(100);

    let mut sender = serialport::new(hw_config.port_1, 115200).open().unwrap();
    let mut receiver = serialport::new(hw_config.port_2, 115200)
        .timeout(receive_timeout)
        .open()
        .unwrap();

    sender.clear(ClearBuffer::All).unwrap();
    receiver.clear(ClearBuffer::All).unwrap();

    let expected_message = message.clone();
    let receiver_thread = thread::spawn(move || {
        let chunk_timeout = send_period + marign;
        assert!(receiver.timeout() > 3 * chunk_timeout);

        let mut received_message = Vec::with_capacity(expected_message.len());

        loop {
            let chunk_start = Instant::now();
            let expected_chunk_until = chunk_start + chunk_timeout;

            let mut buffer = [0u8; 1024];
            assert!(buffer.len() > expected_message.len());

            // Try to read more data than we are expecting and expect some data to be available
            // after the send period (plus some margin).
            match receiver.read(&mut buffer) {
                Ok(read) => {
                    assert!(expected_chunk_until > Instant::now());
                    assert!(read > 0);
                    println!(
                        "receive: {} bytes after waiting {} ms",
                        read,
                        (Instant::now() - chunk_start).as_millis()
                    );
                    received_message.extend_from_slice(&buffer[..read]);
                }
                e => panic!("unexpected error {:?}", e),
            }

            if received_message.len() >= expected_message.len() {
                break;
            }
        }

        assert_eq!(expected_message, received_message);
    });

    let sender_thread = thread::spawn(move || {
        let mut next = Instant::now();

        for chunk in message.chunks(chunk_size) {
            sender.write_all(chunk).unwrap();
            sender.flush().unwrap();

            println!("send: {} bytes", chunk.len());

            next += send_period;
            thread::sleep(next - Instant::now());
        }
    });

    sender_thread.join().unwrap();
    receiver_thread.join().unwrap();
}

#[rstest]
#[case(b"a")]
#[case(b"0123456789")]
#[case(b"0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~")]
#[cfg_attr(not(feature = "hardware-tests"), ignore)]
fn test_timeout_zero(hw_config: HardwareConfig, #[case] message: &[u8]) {
    let timeout = Duration::ZERO;
    let margin = Duration::from_millis(100);

    let mut sender = serialport::new(hw_config.port_1, 115200).open().unwrap();
    let mut receiver = serialport::new(hw_config.port_2, 115200)
        .timeout(timeout)
        .open()
        .unwrap();
    let mut buffer: [u8; 1024] = [0xff; 1024];

    sender.clear(ClearBuffer::All).unwrap();
    receiver.clear(ClearBuffer::All).unwrap();

    sender.write_all(message).unwrap();
    sender.flush().unwrap();
    let flushed_at = Instant::now();

    let expected_until = flushed_at + timeout + margin;
    let mut timeouts = 0usize;

    loop {
        match receiver.read(&mut buffer) {
            Ok(read) => {
                assert!(read > 0);
                println!(
                    "read: {} bytes of {} after {} timeouts/{} ms",
                    read,
                    message.len(),
                    timeouts,
                    (Instant::now() - flushed_at).as_millis()
                );
                assert_eq!(message[..read], buffer[..read]);
                break;
            }
            Err(e) => {
                assert_eq!(e.kind(), std::io::ErrorKind::TimedOut);
                timeouts += 1;
            }
        }

        assert!(expected_until > Instant::now());
    }
}

#[rstest]
#[case(Duration::from_millis(10))]
#[case(Duration::from_millis(100))]
#[case(Duration::from_millis(1000))]
#[cfg_attr(not(feature = "hardware-tests"), ignore)]
fn test_timeout_greater_zero(hw_config: HardwareConfig, #[case] timeout: Duration) {
    let margin = Duration::from_millis(100);

    let mut sender = serialport::new(hw_config.port_1, 115200).open().unwrap();
    let mut receiver = serialport::new(hw_config.port_2, 115200)
        .timeout(timeout)
        .open()
        .unwrap();

    let message =
        b"0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~";
    let mut buffer: [u8; 1024] = [0xff; 1024];

    sender.clear(ClearBuffer::All).unwrap();
    receiver.clear(ClearBuffer::All).unwrap();

    sender.write_all(message).unwrap();
    sender.flush().unwrap();

    let flushed_at = Instant::now();

    let read = receiver.read(&mut buffer).unwrap();
    let read_at = Instant::now();

    println!(
        "read: {} bytes of {} after {} ms",
        read,
        message.len(),
        (Instant::now() - flushed_at).as_millis()
    );

    assert!(read > 0);
    assert!(flushed_at + timeout + margin > read_at);
    assert_eq!(buffer[..read], message[..read]);
}

/// Checks that reading data with a timeout of `Duration::MAX` returns some data and no error. It
/// does not check the actual timeout for obvious reason.
#[rstest]
#[cfg_attr(not(feature = "hardware-tests"), ignore)]
fn test_timeout_max(hw_config: HardwareConfig) {
    let sleep = Duration::from_millis(3000);
    let margin = Duration::from_millis(500);
    let mut sender = serialport::new(hw_config.port_1, 115200).open().unwrap();
    let mut receiver = serialport::new(hw_config.port_2, 115200)
        .timeout(Duration::MAX)
        .open()
        .unwrap();

    let message =
        b"0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~";
    let mut buffer: [u8; 1024] = [0xff; 1024];

    sender.clear(ClearBuffer::All).unwrap();
    receiver.clear(ClearBuffer::All).unwrap();

    let started_at = Instant::now();

    let sender_thread = thread::spawn(move || {
        thread::sleep(sleep);

        sender.write_all(message).unwrap();
        sender.flush().unwrap();
    });

    let read = receiver.read(&mut buffer).unwrap();
    let read_at = Instant::now();

    println!(
        "read: {} bytes of {} after {} ms",
        read,
        message.len(),
        (Instant::now() - started_at).as_millis()
    );

    assert!(read > 0);
    assert!(read_at > started_at + sleep);
    assert!(read_at < started_at + sleep + margin);
    assert_eq!(buffer[..read], message[..read]);

    sender_thread.join().unwrap();
}
