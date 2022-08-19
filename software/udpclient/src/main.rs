use std::net::UdpSocket;
use std::time::Instant;
use std::{thread, time::Duration};

fn main() {
    // 10.0.0.1 is the gateway and *.2 is the DHCP server
    // 0.0.0.0 -> broadcast
    let dst_addr = "169.254.1.229:8052";

    // Bind a port
    let socket = UdpSocket::bind("0.0.0.0:8053").unwrap();
    socket.set_nonblocking(true).unwrap();
    let _ = match socket.connect(dst_addr) {
        Ok(_) => {
            println!("Socket connection success");
            true
        }
        Err(x) => {
            println!("Socket connection error: {x:?}");
            false
        }
    };

    // Send a packet ahead to get the ARP machinery moving
    // so that we do not lose the first few packets later
    let _ = socket.send_to(b"sup, nerd", dst_addr);
    thread::sleep(Duration::from_millis(250));

    // Try to send and receive
    let mut buf: [u8; 1522] = [0; 1522];
    let mut i = 0;
    let mut sent: u64 = 0;
    let mut recvd: u64 = 0;
    let timeout = Duration::from_micros(500);
    let spam_interval = Duration::from_millis(250);
    let mut last_spam = Instant::now();
    // let mut latency: Vec<Duration>;
    let start_of_loop = Instant::now();
    let mut elapsed: u64;
    let mut rate: f64;
    let mut loss_rate: f64;
    loop {
        // Send a unique message to the device
        let msg = format!("greetings greetings {i}");
        let msg_bytes = msg.as_bytes();
        match socket.send_to(&msg_bytes, dst_addr) {
            Ok(_) => {
                sent += 1;
            }
            Err(x) => println!("{i} Data send failure: {x:?}"),
        }

        // Wait until we have heard back or time out
        let start_of_recv = Instant::now();
        'outer: while start_of_recv.elapsed() < timeout {
            if let Ok((amt, _)) = socket.recv_from(&mut buf) {
                if &buf[..amt] == msg_bytes {
                    recvd += 1;
                    break 'outer;
                }
            };
        }

        if last_spam.elapsed() > spam_interval {
            last_spam = Instant::now();
            elapsed = start_of_loop.elapsed().as_secs();
            rate = (recvd as f64) / (elapsed as f64);
            loss_rate = ((sent - recvd) as f64) / (sent as f64) * 100.0;
            println!("{i} Sent: {sent} [packets], Received: {recvd} [packets], Elapsed: {elapsed:?} [s], Round-Trip Rate: {rate:.1} [packets/sec], Loss rate: {loss_rate:.4} [percent]");
        }

        i += 1;
    }
}
