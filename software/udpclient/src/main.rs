use std::net::UdpSocket;
use std::{thread, time::Duration};

fn main() {
    // 10.0.0.1 is the gateway and *.2 is the DHCP server
    // 0.0.0.0 -> broadcast
    let dst_addr = "10.0.0.229:8053";

    // Bind a port
    let socket = UdpSocket::bind("0.0.0.0:8053").unwrap();
    socket
        .set_read_timeout(Some(Duration::from_nanos(1)))
        .unwrap();
    // Tell the socket to listen for 
    let connected = match socket.connect(dst_addr) {
        Ok(_) => {
            println!("Socket connection success");
            true
        }
        Err(x) => {
            println!("Socket connection error: {x:?}");
            false
        }
    };
    socket.set_broadcast(true).unwrap(); // Enable sending to broadcast address

    // Try to send and receive
    let mut buf: [u8; 1522] = [0; 1522];
    let mut i = 0;
    loop {
        println!("{i} Loop Start");

        // Receive all buffered frames
        if connected {
            while let Ok((amt, src)) = socket.recv_from(&mut buf) {
                println!("{i} Received {amt} bytes from {src} : {:?}", unsafe {
                    String::from_utf8_unchecked(buf[0..amt].to_vec())
                });
            }
        } else {
            println!("{i} Skipping recv due to connection failure")
        }

        // Send specifically to device
        match socket.send_to(b"greetings", dst_addr) {
            Ok(_) => println!("{i} Data send success"),
            Err(x) => println!("{i} Data send failure: {x:?}"),
        }

        // Broadcast
        match socket.send_to(b"greetings", "0.0.0.0:8053") {
            Ok(_) => println!("{i} Data send success"),
            Err(x) => println!("{i} Data send failure: {x:?}"),
        }

        i += 1;

        thread::sleep(Duration::from_millis(250));
    }
}
