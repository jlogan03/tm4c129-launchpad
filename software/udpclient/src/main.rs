use chrono::Utc;
use hyper::{Body, Client, Method, Request};
use rpassword::prompt_password;
use std::net::UdpSocket;
use std::time::Instant;
use std::{thread, time::Duration};

const INFLUXDB_PORT: u16 = 8886;

struct Influxdb {
    pub port: u16,
    pub bucket: &'static str,
    pub org: &'static str,
    pub token: String,
}

#[tokio::main]
async fn main() {
    // Set up influxdb
    let client = Client::new();
    let idb: Influxdb = Influxdb {
        port: INFLUXDB_PORT,
        bucket: "test",
        org: "isthmus",
        token: prompt_password("InfluxDB access token: ").unwrap(),
    };
    let idb_uri = format!(
        "http://localhost:{}/api/v2/write?org={}&bucket={}&precision=ns",
        idb.port, idb.org, idb.bucket
    );
    let token_str = format!("Token {}", &idb.token);

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

    // Send a few packets ahead to get the ARP machinery moving
    // so that we do not lose the first few packets later
    for _ in 0..20 {
        let _ = socket.send_to(b"sup, nerd", dst_addr);
        thread::sleep(Duration::from_millis(50));
    }

    // Metrics
    let mut latencies: Vec<f64> = Vec::new();
    let mut times: Vec<i64> = Vec::new();

    // Try to send and receive
    let mut buf: [u8; 1522] = [0; 1522];
    let mut i = 0;
    let mut sent: u64 = 0;
    let mut recvd: u64 = 0;
    let timeout = Duration::from_micros(2000).as_secs_f64();
    let spam_interval = Duration::from_millis(500).as_secs_f64();
    let mut last_spam = Instant::now();
    let mut latency: f64;
    let mut msg_bytes = [0_u8; 160];
    let mut num_to_send = 1;
    loop {
        // Send a unique message to the device, varying the length on each
        // experiment to examine the effect of packet size
        let bytes = (i as u64).to_be_bytes();
        for j in 0..msg_bytes.len() {
            msg_bytes[j] = bytes[j % 8];
        }
        match socket.send_to(&msg_bytes[..num_to_send], dst_addr) {
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
                if &buf[..amt] == &msg_bytes[..num_to_send] {
                    recvd += 1;
                    // Store values to send to influxdb
                    latencies.push(latency);
                    times.push(Utc::now().timestamp_nanos());

                    break 'outer;
                }
            } else {
                continue;
            }
        }

        // Once per spam interval, print outputs to terminal and send data to influxdb
        let elapsed = last_spam.elapsed().as_secs_f64();
        if elapsed > spam_interval {
            // Keep running average of roundtrip latency and drop rate
            let mean_latency_us = latencies.iter().sum::<f64>() / (latencies.len() as f64) * 1e6;
            let rate = (recvd as f64) / elapsed;
            let loss_rate = ((sent - recvd) as f64) / (sent as f64) * 100.0;
            let throughput = (recvd as f64) * (num_to_send as f64) / elapsed;
            println!("Data Size: {num_to_send} [B], Throughput: {throughput:.0} [B/s], Sent: {sent} [packets], Received: {recvd} [packets], Elapsed: {elapsed:.2} [s], Round-Trip Rate: {rate:.1} [packets/sec], Loss Rate: {loss_rate:.4} [percent], Mean Latency: {mean_latency_us:.0} [us]");
            // Reset counters
            sent = 0;
            recvd = 0;

            // Send stored values to influxdb and clear buffers
            let mut lines: Vec<String> = Vec::new();
            //    Log metrics
            while times.len() >= 1 {
                let t = times.pop().unwrap_or_default();
                let l = latencies.pop().unwrap_or_default();
                lines.push(format!("board,units=s latency={l} {t}"))
            }
            //    Log aggregate metrics
            let t = Utc::now().timestamp_nanos();
            lines.push(format!("board,units=Hz roundtrip_rate={rate} {t}"));
            lines.push(format!("board,units=percent roundtrip_loss_rate={loss_rate} {t}"));
            lines.push(format!("board,units=B/s roundtrip_throughput={throughput} {t}"));
            lines.push(format!("board,units=B roundtrip_data_size={num_to_send} {t}"));
            //    Send data via HTTP request
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
            // Increment number of bytes to send in each message
            num_to_send = ((num_to_send + 5) % msg_bytes.len()).max(1);

            // Reset timer at end so that it doesn't absorb timing from database comms
            last_spam = Instant::now();
        }

        i += 1;
    }
}
