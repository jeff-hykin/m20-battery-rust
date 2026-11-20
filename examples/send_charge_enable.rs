//! Send charge-enable requests (0xb3) on the M20's motor bus, and nothing else. For a live test with rl_deploy
//! stopped and the robot lying on its dock. Every byte handed to the socket comes from `charge_enable_request`, and
//! `checked` refuses anything that is not a bare 10-byte 0xb3 header, so no joint command can go out.
//! usage: send_charge_enable --yes-send [interface (eth1)] [requests (1)]

#[cfg(target_os = "linux")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    use m20_battery::charge::charge_enable_request;
    use m20_ecan::transport::RawSocket;

    /// The only gate to the socket: a 10-byte header, type 0xb3, code 0x11, length 0.
    fn checked(body: &[u8]) -> Result<&[u8], String> {
        if body.len() == 10 && body[5] == 0xb3 && body[4] == 0x11 && body[6..10] == [0, 0, 0, 0] {
            Ok(body)
        } else {
            Err(format!("refusing to send a non-charge-enable frame: {body:02x?}"))
        }
    }

    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) != Some("--yes-send") {
        return Err("pass --yes-send: this transmits on the motor bus".into());
    }
    let interface = args.get(1).map(String::as_str).unwrap_or("eth1");
    let requests: usize = args.get(2).map(|s| s.parse()).transpose()?.unwrap_or(1);
    for entry in std::fs::read_dir("/proc")?.flatten() {
        if std::fs::read_to_string(entry.path().join("comm")).map(|c| c.trim() == "rl_deploy").unwrap_or(false) {
            return Err("rl_deploy is running; it owns the bus".into());
        }
    }
    let socket = RawSocket::open(interface)?;
    let mut seq: u8 = 0;
    for request in 0..requests {
        for body in charge_enable_request(seq) {
            let sent = socket.send(checked(&body)?)?;
            let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH)?.as_nanos();
            println!("{now} request {request} seq {seq} sent {sent} bytes {body:02x?}");
            seq = seq.wrapping_add(1);
            // rl_deploy's spacing: one frame per 1 ms control cycle
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }
    Ok(())
}

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("send_charge_enable needs Linux (AF_PACKET)");
}
