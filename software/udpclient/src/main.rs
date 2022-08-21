use std::net::UdpSocket;
use std::time::Instant;
use std::{thread, time::Duration};
use chrono::Utc;
use hyper::{Body, Method, Request, Uri, Client};
use rpassword::prompt_password;

const INFLUXDB_PORT: u16 = 8886;

struct Influxdb {
    pub port: u16,
    pub bucket: &'static str,
    pub org: &'static str,
    pub token: String
}

// #[tokio::main]
fn main() {
    // // Set up influxdb
    // let client = Client::new();
    // let idb: Influxdb = Influxdb {
    //     port: INFLUXDB_PORT,
    //     bucket: "test",
    //     org: "isthmus",
    //     token: prompt_password("InfluxDB access token: ").unwrap()
    // };
    // let idb_uri = format!("http://localhost:{}/api/v2/write?org={}&bucket={}&precision=ns", idb.port, idb.org, idb.bucket);

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
    for _ in 0..10 {
        let _ = socket.send_to(b"sup, nerd", dst_addr);
        thread::sleep(Duration::from_millis(250));
    }

    thread::sleep(Duration::from_millis(1000));

    // Try to send and receive
    let mut buf: [u8; 1522] = [0; 1522];
    let mut i = 0;
    let mut sent: u64 = 0;
    let mut recvd: u64 = 0;
    let min_timeout = Duration::from_micros(300).as_secs_f64();
    let mut timeout = Duration::from_micros(300).as_secs_f64();
    let spam_interval = Duration::from_millis(1000);
    let mut last_spam = Instant::now();
    let start_of_loop = Instant::now();
    let mut elapsed: u64;
    let mut rate: f64;
    let mut loss_rate: f64;
    let mut latency: f64;
    let mut mean_latency_us: f64 = 0.0;
    loop {
        // Do some formatting out of the loop
        // let token_str = format!("Token {}", &idb.token);

        // Send a unique message to the device
        let msg = format!("greetings my dude. it is wednesday {i}");
        let msg_bytes = msg.as_bytes();
        match socket.send_to(&msg_bytes, dst_addr) {
            Ok(_) => {}
            Err(x) => println!("{i} Data send failure: {x:?}"),
        }
        sent += 1;

        // Wait until we have heard back or time out
        let start_of_recv = Instant::now();
        'outer: loop {
            latency = start_of_recv.elapsed().as_secs_f64();
            if latency > timeout {
                break 'outer;
            } else if let Ok((amt, _)) = socket.recv_from(&mut buf) {
                if &buf[..amt] == msg_bytes {
                    recvd += 1;
                    // Keep running average of roundtrip latency
                    mean_latency_us = ((recvd - 1) as f64) / (recvd as f64) * mean_latency_us
                        + 1.0 / (recvd as f64) * latency * 1e6;
                    timeout = (4.0 * mean_latency_us / 1e6).max(min_timeout);

                    // Send to influxdb
                    // let t = Utc::now().timestamp_nanos();
                    // let latency_ns = (latency * 1e9) as u32;
                    // let line = format!("board,units=s latency={latency} {t}");
                    // let req = Request::builder()
                    // .method(Method::POST)
                    // .uri(&idb_uri)
                    // .header("Authorization", &token_str)
                    // .header("Content-Type", "text/plain; charset=utf-8")
                    // .header("Accept", "application/json")
                    // .body(Body::from(line)).unwrap();
                    // let _ = client.request(req).await;
                    // println!("{:?}", resp);

                    break 'outer;
                }
            } else {
                continue;
            }
        }

        if last_spam.elapsed() > spam_interval {
            last_spam = Instant::now();
            elapsed = start_of_loop.elapsed().as_secs();
            rate = (recvd as f64) / (elapsed as f64);
            loss_rate = ((sent - recvd) as f64) / (sent as f64) * 100.0;
            println!("{i} Sent: {sent} [packets], Received: {recvd} [packets], Elapsed: {elapsed:?} [s], Round-Trip Rate: {rate:.1} [packets/sec], Loss Rate: {loss_rate:.4} [percent], Mean Latency: {mean_latency_us:.0} [us]");
        }

        i += 1;
    }
}
