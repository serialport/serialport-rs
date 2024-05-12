//! This example performs a loopback test using real hardware ports
//!
//! Additionally, some data will be collected and logged during the test to provide some
//! rudimentary benchmarking information. When 'split-port' is specified, the serial port will
//! be split into two channels that read/write "simultaneously" from multiple threads.
//!
//! You can also provide the length (in bytes) of data to test with, and the number of iterations to perform or
//! a list of raw bytes to transmit.
//!
//! To run this example:
//!
//! 1) `cargo run --example loopback /dev/ttyUSB0`
//!
//! 2) `cargo run --example loopback /dev/ttyUSB0 --split-port`
//!
//! 3) `cargo run --example loopback /dev/ttyUSB0 -i 100 -l 32 -b 9600`
//!
//! 4) `cargo run --example loopback /dev/ttyUSB8 --bytes 222,173,190,239`

use std::time::{Duration, Instant};

use clap::Parser;
use serialport::SerialPort;

/// Serialport Example - Loopback
#[derive(Parser)]
struct Args {
    /// The device path to a serialport
    port: String,

    /// The number of read/write iterations to perform
    #[clap(short, long, default_value = "100")]
    iterations: usize,

    /// The number of bytes written per transaction
    ///
    /// Ignored when bytes are passed directly from the command-line
    #[clap(short, long, default_value = "8")]
    length: usize,

    /// The baudrate to open the port with
    #[clap(short, long, default_value = "115200")]
    baudrate: u32,

    /// Bytes to write to the serial port
    ///
    /// When not specified, the bytes transmitted count up
    #[clap(long, use_value_delimiter = true)]
    bytes: Option<Vec<u8>>,

    /// Split the port to read/write from multiple threads
    #[clap(long)]
    split_port: bool,
}

fn main() {
    let args = Args::parse();

    // Open the serial port
    let mut port = match serialport::new(&args.port, args.baudrate)
        .timeout(Duration::MAX)
        .open()
    {
        Err(e) => {
            eprintln!("Failed to open \"{}\". Error: {}", args.port, e);
            ::std::process::exit(1);
        }
        Ok(p) => p,
    };

    // Setup stat-tracking
    let length = args.length;
    let data: Vec<u8> = args
        .bytes
        .unwrap_or_else(|| (0..length).map(|i| i as u8).collect());

    let (mut read_stats, mut write_stats) = Stats::new(args.iterations, &data);

    // Run the tests
    if args.split_port {
        loopback_split(&mut port, &mut read_stats, &mut write_stats);
    } else {
        loopback_standard(&mut port, &mut read_stats, &mut write_stats);
    }

    // Print the results
    println!("Loopback {}:", args.port);
    println!("  data-length: {} bytes", read_stats.data.len());
    println!("  iterations: {}", read_stats.iterations);
    println!("  read:");
    println!("    total: {:.6}s", read_stats.total());
    println!("    average: {:.6}s", read_stats.average());
    println!("    max: {:.6}s", read_stats.max());
    println!("  write:");
    println!("    total: {:.6}s", write_stats.total());
    println!("    average: {:.6}s", write_stats.average());
    println!("    max: {:.6}s", write_stats.max());
    println!("  total: {:.6}s", read_stats.total() + write_stats.total());
    println!(
        "  bytes/s: {:.6}",
        (read_stats.data.len() as f32) / (read_stats.average() + write_stats.average())
    )
}

/// Capture read/write times to calculate average durations
#[derive(Clone)]
struct Stats<'a> {
    pub data: &'a [u8],
    pub times: Vec<Duration>,
    pub iterations: usize,
    now: Instant,
}

impl<'a> Stats<'a> {
    /// Create new read/write stats
    fn new(iterations: usize, data: &'a [u8]) -> (Self, Self) {
        (
            Self {
                data,
                times: Vec::with_capacity(iterations),
                iterations,
                now: Instant::now(),
            },
            Self {
                data,
                times: Vec::with_capacity(iterations),
                iterations,
                now: Instant::now(),
            },
        )
    }

    /// Start a duration timer
    fn start(&mut self) {
        self.now = Instant::now();
    }

    /// Store a duration
    fn stop(&mut self) {
        self.times.push(self.now.elapsed());
    }

    /// Provides the total time elapsed
    fn total(&self) -> f32 {
        self.times.iter().map(|d| d.as_secs_f32()).sum()
    }

    /// Provides average time per transaction
    fn average(&self) -> f32 {
        self.total() / (self.times.len() as f32)
    }

    /// Provides the maximum transaction time
    fn max(&self) -> f32 {
        self.times
            .iter()
            .max()
            .map(|d| d.as_secs_f32())
            .unwrap_or(0.0)
    }
}

fn loopback_standard<'a>(
    port: &mut Box<dyn SerialPort>,
    read_stats: &mut Stats<'a>,
    write_stats: &mut Stats<'a>,
) {
    let mut buf = vec![0u8; read_stats.data.len()];

    for _ in 0..read_stats.iterations {
        // Write data to the port
        write_stats.start();
        port.write_all(write_stats.data)
            .expect("failed to write to serialport");
        write_stats.stop();

        // Read data back from the port
        read_stats.start();
        port.read_exact(&mut buf)
            .expect("failed to read from serialport");
        read_stats.stop();

        // Crash on error
        for (i, x) in buf.iter().enumerate() {
            if read_stats.data[i] != *x {
                eprintln!(
                    "Expected byte '{:02X}' but got '{:02X}'",
                    read_stats.data[i], x
                );
                ::std::process::exit(2);
            }
        }
    }
}

#[rustversion::before(1.63)]
fn loopback_split<'a>(
    _port: &mut Box<dyn SerialPort>,
    _read_stats: &mut Stats<'a>,
    _write_stats: &mut Stats<'a>,
) {
    unimplemented!("requires Rust 1.63 or later");
}

#[rustversion::since(1.63)]
fn loopback_split<'a>(
    port: &mut Box<dyn SerialPort>,
    read_stats: &mut Stats<'a>,
    write_stats: &mut Stats<'a>,
) {
    let mut buf = vec![0u8; read_stats.data.len()];
    let mut rport = match port.try_clone() {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Failed to clone port: {}", e);
            ::std::process::exit(3);
        }
    };

    // Manage threads for read/writing; port usage is not async, so threads can easily deadlock:
    //
    // 1. Read Thread: Park -> Read -> Unpark Write ──────┐
    //                 └──────────────────────────────────┘
    // 2. Write Thread: Write -> Unpark Read -> Park ──────┐
    //                  └──────────────────────────────────┘
    std::thread::scope(|scope| {
        // Get handle for writing thread
        let wr_thread = std::thread::current();

        // Spawn a thread that reads data for n iterations
        let handle = scope.spawn(move || {
            for _ in 0..read_stats.iterations {
                // Wait for the write to complete
                std::thread::park();

                read_stats.start();
                rport
                    .read_exact(&mut buf)
                    .expect("failed to read from serialport");
                read_stats.stop();

                // Crash on error
                for (i, x) in buf.iter().enumerate() {
                    if read_stats.data[i] != *x {
                        eprintln!(
                            "Expected byte '{:02X}' but got '{:02X}'",
                            read_stats.data[i], x
                        );
                        ::std::process::exit(2);
                    }
                }

                // Allow the writing thread to start
                wr_thread.unpark();
            }
        });

        // Write data to the port for n iterations
        for _ in 0..write_stats.iterations {
            write_stats.start();
            port.write_all(write_stats.data)
                .expect("failed to write to serialport");
            write_stats.stop();

            // Notify that the write completed
            handle.thread().unpark();

            // Wait for read to complete
            std::thread::park();
        }
    });
}
