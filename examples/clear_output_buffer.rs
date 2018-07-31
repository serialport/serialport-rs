//
// Provides a way to test clearing and querying the size of the serial output buffer.
//
// USAGE:
//
// 1. Connect a serial device to your host computer. E.g. an Arduino could be used. It will be able
//    to receive data without any specific sketch loaded.
// 2. Run this example
// 3. Observe the output - it reports how many bytes are waiting to be sent to the connected device
// 4. Press the Return key to make the example clear the output buffer. You should see the number
//    of bytes queued to send momentarily drop to 0
// 5. Try passing different values for the buffer-size argument to see how that affects the speed
//    and saturation point of the output buffer
// 6. Press Ctrl+D (Unix) or Ctrl+Z (Win) to quit
//

extern crate argparse;
extern crate serialport;

use std::error::Error;
use std::io::{self, Read};
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use argparse::{ArgumentParser, Store};
use serialport::prelude::*;

const DEFAULT_BLOCK_SIZE: usize = 128;

fn main() {
    let mut port_name = "".to_string();
    let mut baud_rate = "".to_string();
    let mut block_size = DEFAULT_BLOCK_SIZE;
    {
        let block_size_help = format!(
            "The size in bytes of the block of data to write to the port (default: {} bytes)",
            DEFAULT_BLOCK_SIZE
        );

        let mut ap = ArgumentParser::new();
        ap.set_description(
            "Reports how many bytes are waiting to be sent and allows the user to clear the output buffer"
        );
        ap.refer(&mut port_name)
            .add_argument("port", Store, "Port name")
            .required();
        ap.refer(&mut baud_rate)
            .add_argument("baud", Store, "Baud rate")
            .required();
        ap.refer(&mut block_size)
            .add_argument("block-size", Store, &block_size_help);
        ap.parse_args_or_exit();
    }

    let exit_code = match run(&port_name, &baud_rate, block_size) {
        Ok(_) => 0,
        Err(e) => {
            println!("Error: {}", e);
            1
        }
    };

    std::process::exit(exit_code);
}

fn run(port_name: &str, baud_rate: &str, block_size: usize) -> Result<(), Box<Error>> {
    let mut settings: SerialPortSettings = Default::default();
    settings.timeout = Duration::from_millis(10);

    let rate = baud_rate
        .parse::<u32>()
        .map_err(|_| format!("Invalid baud rate '{}' specified", baud_rate))?;
    settings.baud_rate = rate.into();

    let mut port = serialport::open_with_settings(&port_name, &settings)
        .map_err(|ref e| format!("Port '{}' not available: {}", &port_name, e))?;

    let chan_clear_buf = input_service();

    println!("Connected to {} at {} baud", &port_name, &baud_rate);
    println!("Ctrl+D (Unix) or Ctrl+Z (Win) to stop. Press Return to clear the buffer.");

    let block = vec![0; block_size];

    // This loop writes the block repeatedly, as fast as possible, to try to saturate the
    // output buffer. If you don't see much data queued to send, try changing the block size.
    loop {
        match port.write(&block) {
            Ok(_) => (),
            Err(ref e) if e.kind() == io::ErrorKind::TimedOut => (),
            Err(e) => panic!("Error while writing data to the port: {}", e),
        };

        match chan_clear_buf.try_recv() {
            Ok(_) => {
                println!("------------------------- Discarding buffer ------------------------- ");
                port.clear(ClearBuffer::Output)
                    .expect("Failed to discard output buffer")
            }
            Err(mpsc::TryRecvError::Empty) => (),
            Err(mpsc::TryRecvError::Disconnected) => {
                println!("Stopping.");
                break;
            }
        }

        println!(
            "Bytes queued to send: {}",
            port.bytes_to_write().expect("Error calling bytes_to_write")
        );
    }

    Ok(())
}

fn input_service() -> mpsc::Receiver<()> {
    let (tx, rx) = mpsc::channel();

    thread::spawn(move || {
        let mut buffer = [0; 32];
        loop {
            // Block awaiting any user input
            match io::stdin().read(&mut buffer) {
                Ok(0) => {
                    drop(tx); // EOF, drop the channel and stop the thread
                    break;
                }
                Ok(_) => tx.send(()).unwrap(), // Signal main to clear the buffer
                Err(e) => panic!(e),
            }
        }
    });

    rx
}
