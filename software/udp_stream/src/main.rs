//! Desktop counterpart to the udp_echo board example.
//!
//! Optionally sends metrics to an InfluxDB instance which is assumed to be
//! running locally but could easily be pointed to another IP address.
//!
//! The program will prompt for a login token for the database during startup;
//! to run without sending metrics, just leave the token blank.

use chrono::Utc;
use core_affinity;
use hyper::{Body, Client, Method, Request};
use rpassword::prompt_password;
use std::net::UdpSocket;
use std::time::Instant;
use std::{thread, time::Duration};

const INFLUXDB_PORT: u16 = 8886; // 8086 is standard, but it can be configured for others

struct Influxdb {
    pub port: u16,
    pub bucket: &'static str,
    pub org: &'static str,
    pub token: String,
}

#[tokio::main]
async fn main() {
    // Set core affinity; otherwise, we drop a bunch of packets every time the thread relocates
    let core_ids = core_affinity::get_core_ids().unwrap();
    core_affinity::set_for_current(*core_ids.last().unwrap()); // Set affinity to last core

    // Set up influxdb
    let client = Client::new();
    let idb: Influxdb = Influxdb {
        port: INFLUXDB_PORT,
        bucket: "test",
        org: "isthmus",
        token: prompt_password("InfluxDB access token (leave blank to skip): ").unwrap(),
    };
    let idb_uri = format!(
        "http://localhost:{}/api/v2/write?org={}&bucket={}&precision=ns",
        idb.port, idb.org, idb.bucket
    );
    let token_str = format!("Token {}", &idb.token);

    // Board's IP address
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

    // Send a few packets ahead to get the ARP machinery moving
    // so that we do not lose the first few packets later
    for _ in 0..20 {
        let _ = socket.send_to(b"sup, nerd", dst_addr);
        thread::sleep(Duration::from_millis(50));
    }

    // Try to send and receive
    let mut buf: [u8; 1522] = [0; 1522];
    let mut recvd: u64 = 0;
    let mut bytes_recvd: u64 = 0;
    let spam_interval = Duration::from_millis(1000).as_secs_f64();
    let mut last_spam = Instant::now();
    let mut data_size: usize = 0;
    loop {
        // Wait until we hear from the device or time out
        if let Ok((amt, _)) = socket.recv_from(&mut buf) {
            recvd += 1;
            bytes_recvd += amt as u64;
            data_size = amt;
        }

        // Once per spam interval, print outputs to terminal and send data to influxdb
        let elapsed = last_spam.elapsed().as_secs_f64();
        if elapsed > spam_interval {
            //
            // Keep running average of roundtrip latency and drop rate
            //
            let rate = (recvd as f64) / elapsed;
            let throughput = (bytes_recvd as f64) / elapsed;
            println!("Data Size: {data_size} [B], Throughput: {throughput:.0} [B/s], Received: {recvd} [packets], Elapsed: {elapsed:.2} [s], Stream Rate: {rate:.1} [packets/sec]");
            // Reset counters
            recvd = 0;
            bytes_recvd = 0;

            //
            // Send stored values to influxdb and clear buffers
            //
            let mut lines: Vec<String> = Vec::new();
            //    Log aggregate metrics
            let t = Utc::now().timestamp_nanos();
            lines.push(format!("board,units=Hz stream_rate={rate} {t}"));
            lines.push(format!(
                "board,units=B/s stream_throughput={throughput} {t}"
            ));
            lines.push(format!("board,units=B stream_data_size={data_size} {t}"));
            //    Send data via HTTP request
            if token_str != "Token " {
                let req = Request::builder()
                    .method(Method::POST)
                    .uri(&idb_uri)
                    .header("Authorization", &token_str)
                    .header("Content-Type", "text/plain; charset=utf-8")
                    .header("Accept", "application/json")
                    .body(Body::from(lines.join("\n")))
                    .unwrap();
                // We need to block on this request to avoid clogging the wire while the test is in progress
                // Check the response but only print if there was an error
                let resp = client.request(req).await;
                match resp {
                    Ok(x) => match x.status().is_success() {
                        true => {}
                        false => println!("{:?}", x),
                    },
                    Err(x) => {
                        println!("{:?}", x);
                    }
                }
            }

            // Reset timer at end so that it doesn't absorb timing from database comms
            last_spam = Instant::now();
        }
    }
}
