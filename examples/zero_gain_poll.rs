//! Zero-gain poll of the M20 motor board, for the question "does the board answer someone other than rl_deploy?".
//! Sends rl_deploy's sitting frame byte for byte (all 16 joints: kp = kd = 0, torque = velocity = 0, control word 4,
//! position = the vendor calibration's URDF zero, as rl_deploy sends while passive), changing only the sequence. It
//! stops at the first reply showing torque, motion or a status other than 1. Optionally sends one 0xb3 request.
//! `--handshake FILE` first replays rl_deploy's startup frames from FILE (lines "<t_ns> tx <frame hex>" as the
//! capture logger writes them), byte for byte apart from the sequence, 1 ms apart, through the same gate.
//! usage: zero_gain_poll --yes-send [--handshake FILE] [--normal-path] [--comm-rl-deploy] [--enable-at N] <frames> <interval_ms>
//!        [--max-torque 1.0] [--max-velocity 0.5] [--max-drift 0.05]

/// rl_deploy's sitting payload (16 joint units), copied from its own frame on the dock (2026-11-20 12:55:57 CST):
/// kp = kd = 0, velocity = torque = 0 (signed zeros as it writes them), control word 4, position = the URDF zero.
const SITTING_PAYLOAD: [u8; 384] = [
    0x79, 0x00, 0x02, 0x00, 0xf4, 0x66, 0xdf, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x04, 0x00, 0x00, 0x00, 0x79, 0x00, 0x03, 0x00, 0x12, 0x54, 0x12, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x79, 0x00, 0x04, 0x00, 0xc2, 0xb8, 0x32, 0x40, 0x00,
    0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x79, 0x00, 0x05, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
    0x00, 0x79, 0x01, 0x02, 0x00, 0xf4, 0x66, 0xdf, 0xbe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x79, 0x01, 0x03, 0x00, 0x12, 0x54, 0x12, 0xc0, 0x00, 0x00, 0x00, 0x80, 0x00,
    0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x79, 0x01, 0x04, 0x00, 0xc2, 0xb8, 0x32, 0xc0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x79, 0x01, 0x05,
    0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00,
    0x00, 0x00, 0x79, 0x02, 0x02, 0x00, 0xf4, 0x66, 0xdf, 0xbe, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x79, 0x02, 0x03, 0x00, 0x12, 0x54, 0x12, 0xc0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x79, 0x02, 0x04, 0x00, 0xc2, 0xb8, 0x32,
    0xc0, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x79, 0x02,
    0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
    0x00, 0x00, 0x00, 0x79, 0x03, 0x02, 0x00, 0xf4, 0x66, 0xdf, 0x3e, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80,
    0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x79, 0x03, 0x03, 0x00, 0x12, 0x54, 0x12, 0x40, 0x00, 0x00, 0x00,
    0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x79, 0x03, 0x04, 0x00, 0xc2, 0xb8,
    0x32, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x79,
    0x03, 0x05, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00,
    0x04, 0x00, 0x00, 0x00,
];

/// rl_deploy's sitting frame body with sequence `seq`: the only joint frame this tool can send.
pub fn passive_body(seq: u8) -> Vec<u8> {
    m20_ecan::frame::encode(
        seq,
        m20_ecan::frame::FLAGS_JOINT_COMMAND,
        m20_ecan::frame::CODE_JOINT_COMMAND,
        m20_ecan::FrameType::Joints,
        &SITTING_PAYLOAD,
    )
    .expect("384 bytes fit")
}

/// Control words a zero-gain frame may carry: rl_deploy's startup handshake (1 Disable, 17 ErrorReset, 2 Enable,
/// 23 GetStatusWord) and 4, the motor-control word it sends with zero gains while sitting.
pub const ALLOWED_WORDS: [u16; 5] = [1, 17, 2, 23, 4];

/// rl_deploy's 0xb2 startup block, as captured.
const SETUP_B2: [u8; 64] = [
    0x01, 0x01, 0x01, 0x05, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
];

/// rl_deploy's per-joint setup frames (0x81): 16 units, no velocity, torque or gains; ErrorReset, Disable, Enable,
/// GetStatusWord with a zero value, or SetMotorTemperature (14) with its value 1300.
fn setup_81_ok(payload: &[u8]) -> bool {
    payload.len() == 16 * 24
        && payload.chunks(24).all(|u| {
            let value = u32::from_le_bytes([u[4], u[5], u[6], u[7]]);
            let word = u16::from_le_bytes([u[20], u[21]]);
            u[8..20].iter().all(|&b| b == 0)
                && match word {
                    1 | 2 | 17 | 23 => value == 0,
                    14 => value == 1300,
                    _ => false,
                }
        })
}

/// The gate in front of the socket: a 16-joint 0x89 command with every gain, torque and velocity zero and an
/// allowed control word, or a bare 0xb3 header. Anything else is refused.
pub fn checked(body: &[u8]) -> Result<&[u8], String> {
    let frame = m20_ecan::Frame::parse(body).map_err(|e| e.to_string())?;
    match (frame.header.frame_type, &m20_ecan::Message::decode(&frame, m20_ecan::Direction::ToBoard)) {
        (m20_ecan::FrameType::Joints, Ok(m20_ecan::Message::JointCommands(commands)))
            if commands.len() == 16
                && commands.iter().all(|c| {
                    c.kp == 0.0
                        && c.kd == 0.0
                        && c.torque == 0.0
                        && c.velocity == 0.0
                        && ALLOWED_WORDS.contains(&c.control_word)
                }) =>
        {
            Ok(body)
        }
        (m20_ecan::FrameType::ChargeEnable, _) if body.len() == 10 => Ok(body),
        // rl_deploy's startup setup, exactly as it sends it: version request, the 0xb2 setup block, per-joint 0x81
        (m20_ecan::FrameType::Version, _) if body.len() == 10 && body[4] == 0x11 => Ok(body),
        (m20_ecan::FrameType::Other(0xb2), _) if frame.payload == SETUP_B2 && body[4] == 0x11 => Ok(body),
        (m20_ecan::FrameType::Type81, _) if body[4] == 0x84 && setup_81_ok(frame.payload) => Ok(body),
        _ => Err(format!("refusing to send {:02x?}", &body[..body.len().min(24)])),
    }
}

#[cfg(target_os = "linux")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    use std::os::fd::AsRawFd;
    use std::time::{Duration, Instant};

    use m20_battery::BatteryReport;
    use m20_ecan::transport::{send_address, RawSocket, MSG_YSC};
    use m20_ecan::{Direction, Frame, FrameType, Message};

    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let mut flag = |name: &str| match args.iter().position(|a| a == name) {
        Some(i) => {
            args.remove(i);
            true
        }
        None => false,
    };
    if !flag("--yes-send") {
        return Err("pass --yes-send: this transmits on the motor bus".into());
    }
    let normal = flag("--normal-path");
    let comm = flag("--comm-rl-deploy");
    let handshake = match args.iter().position(|a| a == "--handshake") {
        Some(i) => {
            let path = args.get(i + 1).ok_or("--handshake FILE")?.clone();
            args.drain(i..i + 2);
            Some(path)
        }
        None => None,
    };
    let mut value = |name: &str, default: f64| -> Result<f64, Box<dyn std::error::Error>> {
        match args.iter().position(|a| a == name) {
            Some(i) => {
                let v = args.get(i + 1).ok_or("missing value")?.parse()?;
                args.drain(i..i + 2);
                Ok(v)
            }
            None => Ok(default),
        }
    };
    let enable_at = value("--enable-at", -1.0)?;
    let max_torque = value("--max-torque", 1.0)? as f32;
    let max_velocity = value("--max-velocity", 0.5)? as f32;
    let max_drift = value("--max-drift", 0.05)? as f32;
    let frames: usize = args.first().ok_or("frames?")?.parse()?;
    let interval = Duration::from_secs_f64(args.get(1).ok_or("interval_ms?")?.parse::<f64>()? / 1000.0);
    for entry in std::fs::read_dir("/proc")?.flatten() {
        if std::fs::read_to_string(entry.path().join("comm")).map(|c| c.trim() == "rl_deploy").unwrap_or(false)
            && entry.file_name() != std::process::id().to_string().as_str()
        {
            return Err("rl_deploy is running; it owns the bus".into());
        }
    }
    if comm {
        // test whether the vendor path is keyed on the process name
        let name = std::ffi::CString::new("rl_deploy")?;
        unsafe { libc::prctl(libc::PR_SET_NAME, name.as_ptr()) };
    }
    let socket = RawSocket::open("eth1")?;
    let flags = if normal { 0 } else { MSG_YSC };
    let send = |body: &[u8]| -> std::io::Result<usize> {
        let frame = m20_ecan::ethernet::wrap(socket.mac(), body);
        let address = send_address(socket.ifindex());
        let n = unsafe {
            libc::sendto(
                socket.as_raw_fd(),
                frame.as_ptr().cast(),
                frame.len(),
                flags,
                (&address as *const libc::sockaddr_ll).cast(),
                std::mem::size_of::<libc::sockaddr_ll>() as u32,
            )
        };
        if n < 0 {
            Err(std::io::Error::last_os_error())
        } else {
            Ok(n as usize)
        }
    };
    let recv = |buf: &mut [u8]| -> std::io::Result<usize> {
        let mut address = m20_ecan::transport::recv_address(socket.ifindex(), socket.mac());
        let mut len = 20u32;
        let n = unsafe {
            libc::recvfrom(
                socket.as_raw_fd(),
                buf.as_mut_ptr().cast(),
                buf.len(),
                flags,
                (&mut address as *mut libc::sockaddr_ll).cast(),
                &mut len,
            )
        };
        if n < 0 {
            Err(std::io::Error::last_os_error())
        } else {
            Ok(n as usize)
        }
    };
    let start = Instant::now();
    let mut buf = [0u8; 1514];
    let (mut states, mut batteries, mut others) = (0usize, 0usize, 0usize);
    let mut first_q: Option<Vec<f32>> = None;
    let mut seq = 0u8;
    if let Some(path) = &handshake {
        let mut sent = 0;
        let mut last_t: Option<u64> = None;
        for line in std::fs::read_to_string(path)?.lines() {
            let mut parts = line.split_whitespace();
            let (Some(t), Some("tx"), Some(hex)) = (parts.next(), parts.next(), parts.next()) else { continue };
            // the captured spacing between frames (rl_deploy waits 100 / 500 ms between setup steps), at most 1 s
            let t: u64 = t.parse()?;
            if let Some(prev) = last_t {
                let gap = Duration::from_nanos(t.saturating_sub(prev)).min(Duration::from_secs(1));
                if gap > Duration::from_millis(2) {
                    std::thread::sleep(gap);
                }
            }
            last_t = Some(t);
            let bytes: Vec<u8> =
                (0..hex.len()).step_by(2).map(|i| u8::from_str_radix(&hex[i..i + 2], 16)).collect::<Result<_, _>>()?;
            let mut body = bytes.get(14..).ok_or("short frame")?.to_vec();
            body[2] = seq;
            send(checked(&body)?)?;
            seq = seq.wrapping_add(1);
            sent += 1;
            if body.len() < 10 + 16 * 24 {
                println!("{:.4} setup frame {sent}: type {:#04x}", start.elapsed().as_secs_f64(), body[5]);
            } else {
                let words: std::collections::BTreeSet<u16> =
                    (0..16).map(|k| u16::from_le_bytes([body[10 + k * 24 + 20], body[10 + k * 24 + 21]])).collect();
                if words != [4].into() {
                    println!(
                        "{:.4} frame {sent} type {:#04x}: words {words:?}",
                        start.elapsed().as_secs_f64(),
                        body[5]
                    );
                }
            }
            // drain replies so the polling below starts fresh; report any status
            let until = Instant::now() + Duration::from_millis(1);
            while Instant::now() < until {
                if let Ok(n) = recv(&mut buf) {
                    if let Ok(frame) = Frame::parse_ethernet(&buf[..n]) {
                        if let Ok(Message::JointStates { joints, .. }) = Message::decode(&frame, Direction::FromBoard) {
                            let st: std::collections::BTreeSet<u16> = joints.iter().map(|j| j.status_word).collect();
                            let tau = joints.iter().map(|j| j.torque.abs()).fold(0.0f32, f32::max);
                            let vel = joints.iter().map(|j| j.velocity.abs()).fold(0.0f32, f32::max);
                            if tau > max_torque || vel > max_velocity {
                                println!(
                                    "{:.4} STOP during handshake: tau {tau:.3} dq {vel:.3}",
                                    start.elapsed().as_secs_f64()
                                );
                                return Ok(());
                            }
                            if sent % 100 == 1 || st != [1].into() && sent % 10 == 1 {
                                println!(
                                    "{:.4} handshake reply after frame {sent}: status {st:?} tau {tau:.3} dq {vel:.3}",
                                    start.elapsed().as_secs_f64()
                                );
                            }
                        }
                    }
                }
            }
        }
        println!("{:.4} handshake done: {sent} frames", start.elapsed().as_secs_f64());
    }
    for i in 0..frames {
        if enable_at >= 0.0 && i == enable_at as usize {
            for body in m20_battery::charge::charge_enable_request(seq) {
                send(checked(&body)?)?;
                seq = seq.wrapping_add(1);
                println!("{:.4} sent 0xb3", start.elapsed().as_secs_f64());
            }
        }
        let body = passive_body(seq);
        let sent = send(checked(&body)?);
        seq = seq.wrapping_add(1);
        if let Err(e) = &sent {
            println!("{:.4} frame {i}: send error {e}", start.elapsed().as_secs_f64());
            return Ok(());
        }
        let deadline = Instant::now() + interval;
        while Instant::now() < deadline {
            let n = match recv(&mut buf) {
                Ok(n) => n,
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
                Err(e) => {
                    println!("{:.4} recv error {e}", start.elapsed().as_secs_f64());
                    std::thread::sleep(deadline.saturating_duration_since(Instant::now()));
                    break;
                }
            };
            let t = start.elapsed().as_secs_f64();
            let Ok(frame) = Frame::parse_ethernet(&buf[..n]) else { continue };
            if buf[6..12] == socket.mac() {
                continue;
            }
            match Message::decode(&frame, Direction::FromBoard) {
                Ok(Message::JointStates { joints, .. }) => {
                    states += 1;
                    let q: Vec<f32> = joints.iter().map(|j| j.position).collect();
                    let first = first_q.get_or_insert_with(|| q.clone());
                    let drift = q
                        .iter()
                        .zip(first.iter())
                        .zip(&joints)
                        .filter(|(_, j)| j.joint.kind != m20_ecan::JointKind::Wheel)
                        .map(|((a, b), _)| (a - b).abs())
                        .fold(0.0f32, f32::max);
                    let tau = joints.iter().map(|j| j.torque.abs()).fold(0.0f32, f32::max);
                    let vel = joints.iter().map(|j| j.velocity.abs()).fold(0.0f32, f32::max);
                    let status: Vec<u16> = joints.iter().map(|j| j.status_word).filter(|&s| s != 1).collect();
                    if states == 1 || states % 100 == 0 {
                        println!("{t:.4} state #{states} seq {} max|tau| {tau:.3} max|dq| {vel:.3} drift {drift:.4} status!=1 {status:?}", frame.header.seq);
                    }
                    if tau > max_torque || vel > max_velocity || drift > max_drift || !status.is_empty() {
                        println!("{t:.4} STOP: tau {tau:.3} dq {vel:.3} drift {drift:.4} status {status:?}");
                        return Ok(());
                    }
                }
                Ok(m @ Message::Battery(_)) => {
                    batteries += 1;
                    if let Ok(r) = BatteryReport::from_message(&m) {
                        let p = |p: &m20_battery::BatteryPack| {
                            format!(
                                "{:.2} V {:+.2} A {}% mos {} contact {:.2} V detect {} maxT {:.1}",
                                p.voltage_v,
                                p.current_a,
                                p.level_percent,
                                p.mos_state,
                                p.charger_input_v,
                                p.charge_detect,
                                p.temperatures().iter().copied().fold(f32::MIN, f32::max)
                            )
                        };
                        println!("{t:.4} battery #{batteries}: L {} | R {}", p(&r.left), p(&r.right));
                    }
                }
                Ok(other) => {
                    others += 1;
                    println!(
                        "{t:.4} other {:?} {:?}",
                        frame.header.frame_type,
                        format!("{other:?}").chars().take(80).collect::<String>()
                    );
                }
                Err(e) => println!("{t:.4} undecodable {e}"),
            }
        }
        let _ = FrameType::Joints;
    }
    println!(
        "done: {frames} frames sent over {:.2} s; replies: {states} joint states, {batteries} battery, {others} other",
        start.elapsed().as_secs_f64()
    );
    Ok(())
}

#[cfg(not(target_os = "linux"))]
fn main() {
    let _ = (passive_body(0), checked(&[]));
    eprintln!("zero_gain_poll needs Linux (AF_PACKET)");
}

#[cfg(test)]
mod tests {}
