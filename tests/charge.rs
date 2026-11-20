//! Charge enable against the 2026-11-20 moving session: battery frames on the dock (charging), off it, and back on
//! after docking (before the BMS's charge MOS came back), and the 12 charge-enable frames rl_deploy sent then.

use m20_battery::charge::{charge_enable_request, charger_connected, ChargeEnabler, Decision};
use m20_battery::BatteryReport;
use m20_ecan::{decode_ethernet, tap, Direction, Message};

const CAPTURE: &[u8] = include_bytes!("fixtures/moving_20261120_141603.pcapng");

fn reports() -> Vec<(u64, BatteryReport)> {
    tap::read_capture(CAPTURE)
        .unwrap()
        .into_iter()
        .filter(|(_, d, f)| *d == Direction::FromBoard && f[19] == 0x87)
        .map(|(t, _, f)| (t, BatteryReport::from_ethernet(f).unwrap()))
        .collect()
}

#[test]
fn dock_states_from_the_packs() {
    let reports = reports();
    assert_eq!(reports.len(), 3);
    let mut rule = ChargeEnabler::new();
    // 14:16:20, docked and charging: left pack on the charger with its charge MOS on
    let (_, docked) = &reports[0];
    assert!(charger_connected(&docked.left) && docked.left.charge_mos_on());
    assert!(docked.left.charger_input_v > 84.0);
    assert_eq!(rule.on_report(docked, true), Decision::Charging);
    // 14:17:00, driving: no charger, charge MOS off, discharge MOS on
    let (_, driving) = &reports[1];
    for pack in driving.packs() {
        assert!(!charger_connected(pack) && !pack.charge_mos_on() && pack.discharge_mos_on());
    }
    assert_eq!(rule.on_report(driving, false), Decision::Standing);
    // 14:18:30, back on the dock after rl_deploy's requests: on the charger, charge MOS on again
    let (_, charging) = &reports[2];
    assert!(charging.packs().iter().all(|p| charger_connected(p) && p.charge_mos_on()));
    rule.lay_down();
    assert_eq!(rule.on_report(charging, true), Decision::Charging);
}

#[test]
fn requests_match_rl_deploy_byte_for_byte() {
    let frames = tap::read_capture(CAPTURE).unwrap();
    let sent: Vec<&[u8]> = frames
        .iter()
        .filter(|(_, d, f)| matches!(decode_ethernet(f, *d), Ok((_, Message::ChargeEnable))))
        .map(|(_, _, f)| &f[14..])
        .collect();
    assert_eq!(sent.len(), 12);
    for burst in sent.chunks(3) {
        let ours = charge_enable_request(burst[0][2]);
        for (wire, mine) in burst.iter().zip(&ours) {
            assert_eq!(*wire, mine.as_slice());
        }
    }
}
