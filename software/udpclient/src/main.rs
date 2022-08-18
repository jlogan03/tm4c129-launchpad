use std::net::UdpSocket;
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

    // Try to send and receive
    let mut buf: [u8; 1522] = [0; 1522];
    let mut i = 0;
    loop {
        thread::sleep(Duration::from_micros(1000));

        // Receive all buffered frames
        match socket.recv_from(&mut buf) {
            Ok((amt, src)) => {
                println!("{i} Received {amt} bytes from {src} : {:?}", unsafe {
                    String::from_utf8_unchecked(buf[..amt].to_vec())
                });
            }
            Err(x) => {} //{println!("{i} recv error {x}");}
        };

        // Send specifically to device
        if i % 1 == 0 {
            let msg = format!("{} {i}", "greetings");
            match socket.send_to(msg.as_bytes(), dst_addr) {
                Ok(_) => println!("{i} Sent packet with message \"{msg}\""), //println!("{i} Data send success"),
                Err(x) => println!("{i} Data send failure: {x:?}"),
            }
        }

        i += 1;
    }
}
