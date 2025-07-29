use clap::{Arg, Command};
use futures::StreamExt;
fn main() {
    let matches = Command::new("Serialport Example - Listen")
        .about("listen to Usb Plug and Unplug events")
        .disable_version_flag(true)
        .arg(Arg::new("count").help("The number of Usb Plug and Unplug events to listen to"))
        .get_matches();
    let count = match matches.value_of("count") {
        Some(c) => c.parse::<usize>().unwrap(),
        _ => 2,
    };
    futures::executor::block_on(log_events(count));
}

async fn log_events(count: usize) {
    println!("listening to {count} events");
    let mut n = 0;
    let mut events = serialport::listen("SERIALPORT_DEMO");
    while let Some(ev) = events.next().await {
        println!("{:?}", ev);
        n += 1;
        if n == count {
            break;
        }
    }
    println!("demo over");
}
