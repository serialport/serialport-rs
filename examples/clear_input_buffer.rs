//
// Provides a way to test clearing and querying the size of the serial input buffer.
//
// USAGE:
//
// 1. Connect a serial device to your host computer. E.g. an Arduino loaded with the example sketch
//    given below
// 2. Run this example
// 3. Observe the output - it reports how many bytes have been received from the device and are
//    waiting to be read
// 4. Press the Return key to make the example clear the input buffer. You should see the number of
//    bytes queued to read momentarily drop to 0
// 5. Press Ctrl+D (Unix) or Ctrl+Z (Win) to quit
//
// The following Arduino firmware could be used to generate input:
//
// ```
// #include <Arduino.h>
//
// void setup() {
//     Serial.begin(9600);
//     while (!Serial); // wait for serial port to connect. Needed for native USB
// }
//
// int iter = 0;
//
// void loop() {
//     Serial.print(iter);
//     if (++iter == 10) {
//         Serial.println();
//         iter = 0;
//     }
//     delay(1000 / 20);
// }
// ```

use std::error::Error;
use std::io::{self, Read};
use std::panic::panic_any;
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use clap::{Arg, Command};

use serialport::ClearBuffer;

fn main() {
    let matches = Command::new("Serialport Example - Clear Input Buffer")
        .about("Reports how many bytes are waiting to be read and allows the user to clear the input buffer")
        .disable_version_flag(true)
        .arg(Arg::new("port")
             .help("The device path to a serial port")
             .use_value_delimiter(false)
             .required(true))
        .arg(Arg::new("baud")
             .help("The baud rate to connect at")
             .use_value_delimiter(false)
             .required(true))
        .get_matches();

    let port_name = matches.value_of("port").unwrap();
    let baud_rate = matches.value_of("baud").unwrap();

    let exit_code = match run(port_name, baud_rate) {
        Ok(_) => 0,
        Err(e) => {
            println!("Error: {}", e);
            1
        }
    };

    std::process::exit(exit_code);
}

fn run(port_name: &str, baud_rate: &str) -> Result<(), Box<dyn Error>> {
    let rate = baud_rate
        .parse::<u32>()
        .map_err(|_| format!("Invalid baud rate '{}' specified", baud_rate))?;

    let port = serialport::new(port_name, rate)
        .timeout(Duration::from_millis(10))
        .open()
        .map_err(|ref e| format!("Port '{}' not available: {}", &port_name, e))?;

    let chan_clear_buf = input_service();

    println!("Connected to {} at {} baud", &port_name, &baud_rate);
    println!("Ctrl+D (Unix) or Ctrl+Z (Win) to stop. Press Return to clear the buffer.");

    loop {
        println!(
            "Bytes available to read: {}",
            port.bytes_to_read().expect("Error calling bytes_to_read")
        );

        match chan_clear_buf.try_recv() {
            Ok(_) => {
                println!("------------------------- Discarding buffer ------------------------- ");
                port.clear(ClearBuffer::Input)
                    .expect("Failed to discard input buffer")
            }
            Err(mpsc::TryRecvError::Empty) => (),
            Err(mpsc::TryRecvError::Disconnected) => {
                println!("Stopping.");
                break;
            }
        }

        thread::sleep(Duration::from_millis(100));
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
