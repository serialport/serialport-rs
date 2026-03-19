// Provides a very simple async example using the smol executor.
//
// USAGE:
//
// `cargo run --features async --example async_smol <device> <baudrate>`

use futures::io::AsyncReadExt;
use smol_macros::main;
use std::env;
use std::process;

main! {
    async fn main() {
        let args: Vec<String> = env::args().collect();
        if args.len() != 3 {
            eprintln!("Usage: {} <device> <baudrate>", args[0]);
            process::exit(1);
        }

        let port_name = &args[1];
        let baud_rate = args[2].parse::<u32>().expect("Invalid baud rate");

        let mut port = serialport::new(port_name, baud_rate)
            .open_async()
            .expect("Failed to open port");

        println!("Receiving data on {} at {} baud:", &port_name, &baud_rate);

        let mut buf = [0; 1024];
        loop {
            match port.read(&mut buf).await {
                Ok(0) => break,
                Ok(n) => println!("Bytes read: {:?}", &buf[..n]),
                Err(e) => {
                    eprintln!("Error reading: {}", e);
                    break;
                }
            }
        }
    }
}
