//! Charge enable: the request that lets a docked M20 charge, as a frame encoder and a decision rule from the packs' own
//! state. Transport-agnostic: this builds bytes and answers "send now?"; whatever owns the motor bus sends.
//!
//! What the vendor stack does (rl_deploy v1.5.8 + NOS charge_manager; 2026-11-20 sessions and disassembly):
//! - The frame is a bare 10-byte header: magic 0 (not 0x5244), sequence, flags 0, code 0x11, type 0xb3, length 0,
//!   checksum 0. rl_deploy sends it 3 times, 1 ms apart, in place of 3 joint commands, taking their sequence numbers
//!   (`ecan::Motor::BatteryChargeEnable`, called from `ECAN2DDS::PowerEnableHandler` for every nonzero sample on the
//!   DrDDS topic `/BATTERY_CHARGE_ENABLE`).
//! - charge_manager publishes 1 there (`RobotCommander::Open12vPower`, "open dog 12v power switch", 2 s apart): 3 times
//!   when docking finishes or its charging flag flips, and once every 5 s (a 3 s wait plus the 2 s pause) while the BMS
//!   reports charging and the robot is not standing.
//! - What the request does, from the packs' "ch+" (contact voltage) and charge-detect readings: it puts the robot's own
//!   ~10.5 V on the charge contacts; the dock answers with its ~84-85 V, charge-detect goes to 0 (active low) and the
//!   BMS turns its charge MOS (bit 0 of `mos_state`) on within ~1 s. Off the dock the ~10.5 V stays on the contacts for
//!   roughly 25-60 s after the last request, then drops to 0; the charge MOS stays off.
//!
//! So before the first request a docked robot and an undocked one look the same in the battery frames. The rule here
//! needs no DrDDS and no clock: probe once whenever the robot lies down; on the charger (charge-detect 0, or contact
//! voltage at or above the pack's), request while any pack's charge MOS is off; if the charger's voltage goes away while
//! the robot keeps lying there, probe once more. Standing or walking stops it. Whether charging needs the vendor's 5 s
//! keepalive (the ~10.5 V timing out under a charging robot) is not known yet; the re-probe on loss covers it at the
//! cost of a short gap.

use m20_ecan::message::Message;

use crate::{BatteryPack, BatteryReport};

/// Frames per request, as rl_deploy sends them (1 ms apart, one per control cycle).
pub const FRAMES_PER_REQUEST: usize = 3;

/// One charge-enable frame body (header only) with sequence `seq`.
pub fn encode_charge_enable(seq: u8) -> Vec<u8> {
    Message::encode_charge_enable(seq)
}

/// The 3 frame bodies of one request, taking `first_seq` and the two after it from the command stream.
pub fn charge_enable_request(first_seq: u8) -> [Vec<u8>; FRAMES_PER_REQUEST] {
    std::array::from_fn(|i| encode_charge_enable(first_seq.wrapping_add(i as u8)))
}

/// The pack sees the dock's charger on its contacts: charge-detect (active low) or the contact voltage at least the
/// pack's own (the robot's probe is ~10.5 V, the dock ~84-85 V).
pub fn charger_connected(pack: &BatteryPack) -> bool {
    pack.voltage_v > 0.0 && (pack.charge_detect == 0 || pack.charger_input_v >= pack.voltage_v)
}

/// What the rule decided for one battery report.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Decision {
    /// Send a request (probe after lying down, or on the charger with a charge MOS off).
    Request,
    /// The legs are holding the robot up: never request (charge_manager doesn't either).
    Standing,
    /// No charger, nothing to do.
    NotDocked,
    /// On the charger and every present pack's charge MOS is on.
    Charging,
}

/// The charge-enable rule, fed every battery report (1 Hz) and lie-down events.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ChargeEnabler {
    probe_pending: bool,
    docked: bool,
}

impl ChargeEnabler {
    pub fn new() -> Self {
        Self::default()
    }

    /// The robot just lay down (its legs went passive): the next report sends one probe.
    pub fn lay_down(&mut self) {
        self.probe_pending = true;
    }

    /// Decide for the latest report; `legs_passive` = no position gain on the legs.
    pub fn on_report(&mut self, report: &BatteryReport, legs_passive: bool) -> Decision {
        if !legs_passive {
            self.probe_pending = false;
            self.docked = false;
            return Decision::Standing;
        }
        let present: Vec<&BatteryPack> = report.packs().into_iter().filter(|p| p.voltage_v > 0.0).collect();
        let on_charger = present.iter().any(|p| charger_connected(p));
        if on_charger {
            self.docked = true;
            self.probe_pending = false;
            return if present.iter().any(|p| !p.charge_mos_on()) { Decision::Request } else { Decision::Charging };
        }
        if std::mem::take(&mut self.probe_pending) || std::mem::take(&mut self.docked) {
            return Decision::Request;
        }
        Decision::NotDocked
    }
}

/// The charge rule fed straight from motor-bus frames, for whoever owns the bus: commands it sent (a switch to no
/// position gain on the legs is a lie-down) and battery frames from the board. Returns true when a request is due.
#[derive(Clone, Debug, Default)]
pub struct BusCharge {
    rule: ChargeEnabler,
    legs_passive: bool,
    seen_command: bool,
    pub last: Option<Decision>,
}

impl BusCharge {
    pub fn new() -> Self {
        Self::default()
    }

    /// One frame (Ethernet) and its direction; true = send one charge-enable request now.
    pub fn on_frame(&mut self, direction: m20_ecan::Direction, frame: &[u8]) -> bool {
        let Ok((_, message)) = m20_ecan::decode_ethernet(frame, direction) else { return false };
        match (direction, &message) {
            (m20_ecan::Direction::ToBoard, Message::JointCommands(commands)) => {
                let passive =
                    commands.iter().filter(|c| c.joint.kind != m20_ecan::JointKind::Wheel).all(|c| c.kp == 0.0);
                if passive && (!self.legs_passive || !self.seen_command) {
                    self.rule.lay_down();
                }
                self.legs_passive = passive;
                self.seen_command = true;
                false
            }
            (m20_ecan::Direction::FromBoard, Message::Battery(_)) => match BatteryReport::from_message(&message) {
                Ok(report) => {
                    let decision = self.rule.on_report(&report, self.legs_passive);
                    self.last = Some(decision);
                    decision == Decision::Request
                }
                Err(_) => false,
            },
            _ => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pack(mos: u8, detect: u8, contact_v: u16) -> BatteryPack {
        let mut record = [0u8; m20_ecan::message::BATTERY_PACK_LEN];
        record[0..2].copy_from_slice(&8300u16.to_le_bytes());
        record[20] = mos;
        record[112..114].copy_from_slice(&(contact_v * 100).to_le_bytes());
        record[114] = detect;
        BatteryPack::parse(&record).unwrap()
    }

    fn both(p: BatteryPack) -> BatteryReport {
        BatteryReport { left: p.clone(), right: p }
    }

    #[test]
    fn docking_sequence() {
        let mut rule = ChargeEnabler::new();
        let off_dock = both(pack(2, 1, 0));
        let probing = both(pack(2, 1, 10));
        let charger_on = both(pack(2, 0, 84));
        let charging = both(pack(3, 0, 84));
        assert_eq!(rule.on_report(&off_dock, false), Decision::Standing);
        rule.lay_down();
        assert_eq!(rule.on_report(&off_dock, true), Decision::Request);
        assert_eq!(rule.on_report(&probing, true), Decision::NotDocked);
        assert_eq!(rule.on_report(&charger_on, true), Decision::Request);
        assert_eq!(rule.on_report(&charging, true), Decision::Charging);
        // the charger's voltage goes away under a lying robot: one re-probe, then nothing until it lies down again
        assert_eq!(rule.on_report(&probing, true), Decision::Request);
        assert_eq!(rule.on_report(&probing, true), Decision::NotDocked);
        assert_eq!(rule.on_report(&off_dock, true), Decision::NotDocked);
        // standing clears everything
        rule.lay_down();
        assert_eq!(rule.on_report(&charging, false), Decision::Standing);
        assert_eq!(rule.on_report(&off_dock, true), Decision::NotDocked);
    }

    #[test]
    fn bus_charge_probes_after_lying_down_then_follows_the_dock() {
        let ethernet = |body: Vec<u8>| m20_ecan::ethernet::wrap([2, 0, 0, 0, 0, 1], &body);
        let sitting = ethernet(Message::encode_joint_commands(0, &m20_ecan::startup::sitting_commands()));
        let battery = |p: BatteryPack| {
            let mut payload = Vec::new();
            for _ in 0..2 {
                let mut record = [0u8; m20_ecan::message::BATTERY_PACK_LEN];
                record[0..2].copy_from_slice(&((p.voltage_v * 100.0) as u16).to_le_bytes());
                record[20] = p.mos_state;
                record[112..114].copy_from_slice(&((p.charger_input_v * 100.0) as u16).to_le_bytes());
                record[114] = p.charge_detect;
                payload.extend_from_slice(&record);
            }
            ethernet(m20_ecan::frame::encode(0, 6, 0x21, m20_ecan::FrameType::Battery, &payload).unwrap())
        };
        let mut charge = BusCharge::new();
        assert!(!charge.on_frame(m20_ecan::Direction::ToBoard, &sitting));
        assert!(charge.on_frame(m20_ecan::Direction::FromBoard, &battery(pack(2, 1, 0))), "probe after lying down");
        assert!(!charge.on_frame(m20_ecan::Direction::FromBoard, &battery(pack(2, 1, 10))));
        assert!(charge.on_frame(m20_ecan::Direction::FromBoard, &battery(pack(2, 0, 84))), "on the charger, MOS off");
        assert!(!charge.on_frame(m20_ecan::Direction::FromBoard, &battery(pack(3, 0, 84))));
        assert_eq!(charge.last, Some(Decision::Charging));
    }

    #[test]
    fn request_takes_three_sequence_numbers() {
        let request = charge_enable_request(255);
        assert_eq!([request[0][2], request[1][2], request[2][2]], [255, 0, 1]);
        assert!(request.iter().all(|f| f.len() == 10 && f[5] == 0xb3 && f[4] == 0x11 && f[0..2] == [0, 0]));
    }
}
