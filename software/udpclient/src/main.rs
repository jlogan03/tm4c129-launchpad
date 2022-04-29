use std::net::UdpSocket;

fn main() {
    let socket = UdpSocket::bind("127.0.0.1:8053").unwrap();

    // Receives a single datagram message on the socket. If `buf` is too small to hold
    // the message, it will be cut off.
    let mut buf = [0; 10];
    loop {
        match socket.recv_from(&mut buf) {
            Ok((amt, src)) => {println!("{:?}", buf);},
            _ => {}
        }
        
        socket.send_to(&[0; 10], "10.0.0.2:8053").expect("couldn't send data");
    }
}
