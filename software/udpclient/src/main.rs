use std::net::UdpSocket;
use std::{thread, time::Duration};

fn main() {
    // 10.0.0.1 is the gateway and *.2 is the DHCP server
    // 0.0.0.0 -> broadcast
    let dst_addr = "0.0.0.0:8053";

    // Bind a socket and tell it to listen for input from a specific address
    let socket = UdpSocket::bind("127.0.0.1:8053").unwrap();
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

    // Try to send and receive
    let mut buf: [u8; 1500] = [0; 1500];
    let mut i = 0;
    loop {
        println!("{i} Loop Start");
        if connected {
            match socket.recv_from(&mut buf) {
                Ok((amt, src)) => {
                    println!("Received {amt} bytes from {src} : {:?}", buf);
                }
                Err(x) => {
                    println!("{i} Receive error: {x:?}");
                }
            }
        } else {
            println!("{i} Skipping recv due to connection failure")
        }

        match socket.send_to(&[0; 10], dst_addr) {
            Ok(_) => println!("{i} Data send success"),
            Err(x) => println!("{i} Data send failure: {x:?}"),
        }

        i += 1;

        thread::sleep(Duration::from_millis(500));
    }
}
