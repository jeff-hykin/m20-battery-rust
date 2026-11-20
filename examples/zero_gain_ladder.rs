//! One-process zero-gain test at rl_deploy's pace: replay its startup setup frames (captured timing), then the
//! zero-gain handshake and word-4 polling at 1 kHz, then a poll-rate ladder and one 0xb3 request. Sends only what
//! zero_gain_poll's gate admits; a receiver thread does all receiving so sends never wait.
//! Stops on torque / velocity / position drift above the limits, any status other than 1 or 0x21, or no status 1
//! within `--settle-ms` of 1 kHz polling. 0x21 at a slower rate ends the ladder (that rate is too slow).
//! usage: zero_gain_ladder --yes-send REPLAY_FILE [--max-torque 1.0] [--max-velocity 0.5] [--max-drift 0.05]
//!        [--settle-ms 500]

#[path = "zero_gain_poll.rs"]
#[allow(dead_code)]
mod gate;

#[cfg(target_os = "linux")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    use std::os::fd::AsRawFd;
    use std::sync::mpsc;
    use std::sync::Arc;
    use std::time::{Duration, Instant};

    use m20_battery::BatteryReport;
    use m20_ecan::transport::{recv_address, send_address, RawSocket, MSG_YSC};
    use m20_ecan::{Direction, Frame, JointKind, Message};

    let mut args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) != Some("--yes-send") {
        return Err("pass --yes-send: this transmits on the motor bus".into());
    }
    args.remove(0);
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
    let max_torque = value("--max-torque", 1.0)? as f32;
    let max_velocity = value("--max-velocity", 0.5)? as f32;
    let max_drift = value("--max-drift", 0.05)? as f32;
    let settle = Duration::from_secs_f64(value("--settle-ms", 500.0)? / 1000.0);
    let replay = args.first().ok_or("REPLAY_FILE?")?.clone();
    for entry in std::fs::read_dir("/proc")?.flatten() {
        if std::fs::read_to_string(entry.path().join("comm")).map(|c| c.trim() == "rl_deploy").unwrap_or(false) {
            return Err("rl_deploy is running; it owns the bus".into());
        }
    }
    // the replay: (captured ns, frame body); every body must pass the gate before anything is sent
    let mut frames = Vec::new();
    for line in std::fs::read_to_string(&replay)?.lines() {
        let mut parts = line.split_whitespace();
        let (Some(t), Some("tx"), Some(hex)) = (parts.next(), parts.next(), parts.next()) else { continue };
        let bytes: Vec<u8> =
            (0..hex.len()).step_by(2).map(|i| u8::from_str_radix(&hex[i..i + 2], 16)).collect::<Result<_, _>>()?;
        let body = bytes.get(14..).ok_or("short frame")?.to_vec();
        gate::checked(&body)?;
        frames.push((t.parse::<u64>()?, body));
    }
    let socket = Arc::new(RawSocket::open("eth1")?);
    let start = Instant::now();
    let (tx, rx) = mpsc::channel::<(f64, Vec<u8>)>();
    {
        let socket = socket.clone();
        socket.set_nonblocking(false)?;
        std::thread::spawn(move || {
            let mut buf = [0u8; 1514];
            loop {
                let mut address = recv_address(socket.ifindex(), socket.mac());
                let mut len = 20u32;
                let n = unsafe {
                    libc::recvfrom(
                        socket.as_raw_fd(),
                        buf.as_mut_ptr().cast(),
                        buf.len(),
                        MSG_YSC,
                        (&mut address as *mut libc::sockaddr_ll).cast(),
                        &mut len,
                    )
                };
                if n > 14
                    && buf[6..12] != socket.mac()
                    && tx.send((start.elapsed().as_secs_f64(), buf[..n as usize].to_vec())).is_err()
                {
                    return;
                }
            }
        });
    }
    let send = |body: &[u8]| -> Result<(), Box<dyn std::error::Error>> {
        let body = gate::checked(body)?;
        let frame = m20_ecan::ethernet::wrap(socket.mac(), body);
        let address = send_address(socket.ifindex());
        let n = unsafe {
            libc::sendto(
                socket.as_raw_fd(),
                frame.as_ptr().cast(),
                frame.len(),
                MSG_YSC,
                (&address as *const libc::sockaddr_ll).cast(),
                std::mem::size_of::<libc::sockaddr_ll>() as u32,
            )
        };
        if n < 0 {
            Err(std::io::Error::last_os_error().into())
        } else {
            Ok(())
        }
    };
    #[derive(Default)]
    struct Seen {
        states: usize,
        ok: usize,
        flagged: usize,
        other: Vec<u16>,
        batteries: usize,
        max_tau: f32,
        max_vel: f32,
        max_drift: f32,
    }
    let mut first_q: Option<Vec<f32>> = None;
    // drain everything received; returns the tally of this window and prints battery / other frames
    let mut drain = |seen: &mut Seen| {
        while let Ok((t, f)) = rx.try_recv() {
            let Ok(frame) = Frame::parse_ethernet(&f) else { continue };
            match Message::decode(&frame, Direction::FromBoard) {
                Ok(Message::JointStates { joints, .. }) => {
                    seen.states += 1;
                    let q: Vec<f32> = joints.iter().map(|j| j.position).collect();
                    let first = first_q.get_or_insert_with(|| q.clone());
                    let drift = q
                        .iter()
                        .zip(first.iter())
                        .zip(&joints)
                        .filter(|(_, j)| j.joint.kind != JointKind::Wheel)
                        .map(|((a, b), _)| (a - b).abs())
                        .fold(0.0f32, f32::max);
                    seen.max_drift = seen.max_drift.max(drift);
                    seen.max_tau = seen.max_tau.max(joints.iter().map(|j| j.torque.abs()).fold(0.0, f32::max));
                    seen.max_vel = seen.max_vel.max(joints.iter().map(|j| j.velocity.abs()).fold(0.0, f32::max));
                    let statuses: std::collections::BTreeSet<u16> = joints.iter().map(|j| j.status_word).collect();
                    if statuses == [1].into() {
                        seen.ok += 1
                    } else if statuses == [0x21].into() {
                        seen.flagged += 1
                    } else {
                        seen.other.extend(statuses)
                    }
                }
                Ok(m @ Message::Battery(_)) => {
                    seen.batteries += 1;
                    if let Ok(r) = BatteryReport::from_message(&m) {
                        let p = |p: &m20_battery::BatteryPack| {
                            format!(
                                "{:.2} V {:+.2} A {}% mos {} contact {:.2} V detect {}",
                                p.voltage_v,
                                p.current_a,
                                p.level_percent,
                                p.mos_state,
                                p.charger_input_v,
                                p.charge_detect
                            )
                        };
                        println!("{t:.4} battery: L {} | R {}", p(&r.left), p(&r.right));
                    }
                }
                Ok(other) => println!(
                    "{t:.4} reply {:?} {}",
                    frame.header.frame_type,
                    format!("{other:?}").chars().take(90).collect::<String>()
                ),
                Err(e) => println!("{t:.4} undecodable {e}"),
            }
        }
    };
    let unsafe_now = |seen: &Seen| {
        seen.max_tau > max_torque || seen.max_vel > max_velocity || seen.max_drift > max_drift || !seen.other.is_empty()
    };
    let report = |name: &str, seen: &Seen, sent: usize| {
        println!("{:.3} {name}: sent {sent}, states {} (status 1: {}, 0x21: {}, other {:?}), battery {}, max|tau| {:.3} max|dq| {:.3} drift {:.4}",
        start.elapsed().as_secs_f64(), seen.states, seen.ok, seen.flagged, seen.other, seen.batteries, seen.max_tau, seen.max_vel, seen.max_drift)
    };
    let mut seq = 0u8;
    // 1) the replay, at the captured spacing
    let mut seen = Seen::default();
    let mut next = Instant::now();
    let mut last_t = frames.first().map(|f| f.0).unwrap_or(0);
    for (i, (t, body)) in frames.iter().enumerate() {
        next += Duration::from_nanos(t.saturating_sub(last_t)).min(Duration::from_secs(1));
        last_t = *t;
        // matches vendor: replays rl_deploy's captured startup at its own spacing (absolute deadlines, no drift)
        std::thread::sleep(next.saturating_duration_since(Instant::now()));
        let mut body = body.clone();
        body[2] = seq;
        seq = seq.wrapping_add(1);
        send(&body)?;
        drain(&mut seen);
        if unsafe_now(&seen) {
            report("STOP during replay", &seen, i + 1);
            return Ok(());
        }
    }
    report("replay (setup + handshake)", &seen, frames.len());
    // 2) word-4 polling: 1 kHz until settled, then the ladder
    let poll = |seq: &mut u8,
                period: Duration,
                duration: Duration,
                enable_at: Option<Duration>,
                seen: &mut Seen,
                drain: &mut dyn FnMut(&mut Seen)|
     -> Result<usize, Box<dyn std::error::Error>> {
        let begin = Instant::now();
        let mut next = begin;
        let mut sent = 0;
        let mut enabled = false;
        while begin.elapsed() < duration {
            if let Some(at) = enable_at {
                if !enabled && begin.elapsed() >= at {
                    for body in m20_battery::charge::charge_enable_request(*seq) {
                        send(&body)?;
                        *seq = seq.wrapping_add(1);
                        // matches vendor: rl_deploy sends the 3 charge-enable frames one per 1 ms cycle
                        std::thread::sleep(Duration::from_millis(1));
                    }
                    println!("{:.4} sent one 0xb3 request (3 frames)", start.elapsed().as_secs_f64());
                    enabled = true;
                }
            }
            send(&gate::passive_body(*seq))?;
            *seq = seq.wrapping_add(1);
            sent += 1;
            next += period;
            drain(seen);
            if unsafe_now(seen) {
                return Ok(sent);
            }
            // the pace under test, on absolute deadlines (next += period)
            std::thread::sleep(next.saturating_duration_since(Instant::now()));
        }
        drain(seen);
        Ok(sent)
    };
    let ms = Duration::from_millis;
    let mut seen = Seen::default();
    let sent = poll(&mut seq, ms(1), settle, None, &mut seen, &mut drain)?;
    report("1 kHz settle", &seen, sent);
    if unsafe_now(&seen) || seen.ok == 0 {
        println!("STOP: {}", if unsafe_now(&seen) { "torque / motion / status" } else { "status never reached 1" });
        return Ok(());
    }
    let mut seen = Seen::default();
    let sent = poll(&mut seq, ms(1), ms(2000), None, &mut seen, &mut drain)?;
    report("1 kHz 2 s", &seen, sent);
    if unsafe_now(&seen) || seen.flagged > 0 {
        println!("STOP at 1 kHz");
        return Ok(());
    }
    for (period, secs) in [(5u64, 2u64), (10, 2), (100, 2), (500, 3), (1000, 3)] {
        let mut seen = Seen::default();
        let sent = poll(&mut seq, ms(period), Duration::from_secs(secs), None, &mut seen, &mut drain)?;
        report(&format!("every {period} ms for {secs} s"), &seen, sent);
        if unsafe_now(&seen) {
            println!("STOP");
            return Ok(());
        }
        if seen.flagged > 0 {
            println!("status 0x21 at every {period} ms: too slow; back to 1 kHz");
            break;
        }
    }
    let mut seen = Seen::default();
    let sent = poll(&mut seq, ms(1), ms(3000), Some(ms(1000)), &mut seen, &mut drain)?;
    report("1 kHz with 0xb3 at 1 s", &seen, sent);
    println!("done");
    Ok(())
}

#[cfg(not(target_os = "linux"))]
fn main() {
    let _ = gate::checked(&[]);
    eprintln!("zero_gain_ladder needs Linux (AF_PACKET)");
}
