//! Watch both packs live through the ecan_tap.so shim's stream inside rl_deploy and run the charge-enable rule. Prints
//! one line per battery report (1 Hz). With `--act`, a Request creates the shim's one-shot inject file (the shim then
//! sends the 3 frames through rl_deploy's own socket, only while every joint gain is zero). Sends nothing itself.
//! usage: charge_rule_live [--act] [socket (/var/opt/robot/data/ecan_capture/live.sock)] [shim dir (socket's dir)]

#[cfg(unix)]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    use m20_battery::charge::{ChargeEnabler, Decision};
    use m20_battery::BatteryReport;
    use m20_ecan::tap::TapSocket;
    use m20_ecan::{decode_ethernet, Direction, JointKind, Message};

    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let act = args.first().map(String::as_str) == Some("--act");
    if act {
        args.remove(0);
    }
    let socket = args.first().cloned().unwrap_or_else(|| m20_ecan::tap::DEFAULT_SOCKET_PATH.to_string());
    let dir = args.get(1).cloned().unwrap_or_else(|| {
        std::path::Path::new(&socket).parent().unwrap_or(std::path::Path::new(".")).display().to_string()
    });
    let inject = std::path::Path::new(&dir).join("inject_charge_enable");
    let mut tap = TapSocket::bind(&socket)?;
    let mut rule = ChargeEnabler::new();
    let (mut legs_passive, mut seen_command, mut bad_status, mut enables_out) = (false, false, 0u64, 0u64);
    println!("t_s,decision,legs_passive,enables_out,bad_status,L_V,L_A,L_pct,L_mos,L_contact_V,L_detect,L_maxT,R_V,R_A,R_pct,R_mos,R_contact_V,R_detect,R_maxT");
    loop {
        for record in tap.recv()? {
            let Ok((_, message)) = decode_ethernet(&record.frame, record.direction) else { continue };
            match (&message, record.direction) {
                (Message::JointCommands(commands), Direction::ToBoard) => {
                    let passive = commands.iter().filter(|c| c.joint.kind != JointKind::Wheel).all(|c| c.kp == 0.0);
                    if passive && (!legs_passive || !seen_command) {
                        rule.lay_down();
                    }
                    if !passive && legs_passive {
                        println!("# legs got position gain: standing");
                    }
                    legs_passive = passive;
                    seen_command = true;
                }
                (Message::ChargeEnable, Direction::ToBoard) => enables_out += 1,
                (Message::JointStates { joints, .. }, Direction::FromBoard) => {
                    bad_status += joints.iter().filter(|j| j.status_word != 1).count() as u64;
                }
                (Message::Battery(_), Direction::FromBoard) => {
                    let Ok(report) = BatteryReport::from_message(&message) else { continue };
                    let decision = rule.on_report(&report, legs_passive);
                    if act && decision == Decision::Request {
                        std::fs::write(&inject, b"")?;
                    }
                    let pack = |p: &m20_battery::BatteryPack| {
                        let max_t = p.temperatures().iter().copied().fold(f32::MIN, f32::max);
                        format!(
                            "{:.2},{:+.2},{},{},{:.2},{},{:.1}",
                            p.voltage_v,
                            p.current_a,
                            p.level_percent,
                            p.mos_state,
                            p.charger_input_v,
                            p.charge_detect,
                            max_t
                        )
                    };
                    println!(
                        "{:.3},{decision:?},{legs_passive},{enables_out},{bad_status},{},{}",
                        record.timestamp_ns as f64 / 1e9,
                        pack(&report.left),
                        pack(&report.right)
                    );
                }
                _ => {}
            }
        }
    }
}

#[cfg(not(unix))]
fn main() {
    eprintln!("charge_rule_live needs a unix socket");
}
