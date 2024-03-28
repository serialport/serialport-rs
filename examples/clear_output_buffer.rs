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

use std::error::Error;
use std::io::{self, Read};
use std::panic::panic_any;
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use clap::{Arg, ArgMatches, Command};

use serialport::ClearBuffer;

const DEFAULT_BLOCK_SIZE: &str = "128";

fn main() {
    let block_size_help = format!(
        "The size in bytes of the block of data to write to the port (default: {} bytes)",
        DEFAULT_BLOCK_SIZE
    );

    let matches = Command::new("Serialport Example - Clear Output Buffer")
        .about("Reports how many bytes are waiting to be read and allows the user to clear the output buffer")
        .disable_version_flag(true)
        .arg(Arg::new("port")
             .help("The device path to a serial port")
             .use_value_delimiter(false)
             .required(true))
        .arg(Arg::new("baud")
             .help("The baud rate to connect at")
             .use_value_delimiter(false)
             .required(true))
        .arg(Arg::new("block-size")
             .help(Some(block_size_help.as_str()))
             .use_value_delimiter(false)
             .default_value(DEFAULT_BLOCK_SIZE))
        .get_matches();

    let port_name = matches.value_of("port").unwrap();
    let baud_rate = matches.value_of("baud").unwrap();
    let block_size = ArgMatches::value_of_t(&matches, "block-size").unwrap_or_else(|e| e.exit());

    let exit_code = match run(port_name, baud_rate, block_size) {
        Ok(_) => 0,
        Err(e) => {
            println!("Error: {}", e);
            1
        }
    };

    std::process::exit(exit_code);
}

fn run(port_name: &str, baud_rate: &str, block_size: usize) -> Result<(), Box<dyn Error>> {
    let rate = baud_rate
        .parse::<u32>()
        .map_err(|_| format!("Invalid baud rate '{}' specified", baud_rate))?;

    let mut port = serialport::new(port_name, rate)
        .timeout(Duration::from_millis(10))
        .open()
        .map_err(|ref e| format!("Port '{}' not available: {}", &port_name, e))?;

    let chan_clear_buf = input_service();

    println!("Connected to {} at {} baud", &port_name, &baud_rate);
    println!("Ctrl+D (Unix) or Ctrl+Z (Win) to stop. Press Return to clear the buffer.");

    let block = vec![0; block_size];

    // This loop writes the block repeatedly, as fast as possible, to try to saturate the
    // output buffer. If you don't see much data queued to send, try changing the block size.
    loop {
        match port.write_all(&block) {
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
                Ok(_bytes_read) => tx.send(()).unwrap(), // Signal main to clear the buffer
                Err(e) => panic_any(e),
            }
        }
    });

    rx
}
