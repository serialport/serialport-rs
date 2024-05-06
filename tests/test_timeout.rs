mod config;

use config::{hw_config, HardwareConfig};
use rstest::rstest;
use serialport::*;
use std::io::Read;
use std::time::{Duration, Instant};

#[rstest]
#[case(b"a")]
#[case(b"0123456789")]
#[case(b"0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~")]
#[cfg_attr(feature = "ignore-hardware-tests", ignore)]
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
#[cfg_attr(feature = "ignore-hardware-tests", ignore)]
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
