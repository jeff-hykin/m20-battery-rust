//! Battery frames from a real capture (robot docked and charging, 2026-11-20 12:55:57-12:56:07 CST) against rl_deploy's
//! own log of the same seconds.

use std::collections::HashMap;

use m20_battery::{BatteryPack, BatteryReport, ProductionDate};
use m20_ecan::{tap, Direction};

const CAPTURE: &[u8] = include_bytes!("fixtures/sitting_docked_20261120_125557.pcapng");
const VENDOR_LOG: &str = include_str!("fixtures/vendor_log_20261120_125557.csv");

fn vendor_rows() -> HashMap<String, HashMap<String, f64>> {
    let mut lines = VENDOR_LOG.lines();
    let names: Vec<String> = lines.next().unwrap().split(',').map(|name| name.trim().to_string()).collect();
    lines
        .map(|line| {
            let fields: Vec<&str> = line.split(',').collect();
            let row = names
                .iter()
                .zip(&fields)
                .skip(1)
                .filter_map(|(name, value)| Some((name.clone(), value.parse().ok()?)))
                .collect();
            (fields[0].to_string(), row)
        })
        .collect()
}

/// (timestamp in the log's format, report) for every battery frame.
fn reports() -> Vec<(String, BatteryReport)> {
    tap::read_capture(CAPTURE)
        .unwrap()
        .into_iter()
        .filter(|(_, direction, frame)| *direction == Direction::FromBoard && frame[19] == 0x87)
        .map(|(timestamp_ns, _, frame)| {
            // CST = UTC+8, the robot's local time
            let local = timestamp_ns / 1_000_000_000 + 8 * 3600;
            let (day_seconds, days) = (local % 86_400, local / 86_400);
            assert_eq!(days, 20_777, "2026-11-20");
            let stamp =
                format!("20261120_{:02}{:02}{:02}", day_seconds / 3600, day_seconds % 3600 / 60, day_seconds % 60);
            (stamp, BatteryReport::from_ethernet(frame).unwrap())
        })
        .collect()
}

/// Every field the log has, compared at the log's precision; the names of the ones that differ.
fn mismatches(pack: &BatteryPack, row: &HashMap<String, f64>, side: &str) -> Vec<String> {
    let mut checks: Vec<(String, f64, f64, f64)> = vec![
        ("Voltage V".into(), pack.voltage_v as f64, 0.011, 0.0),
        ("Current A".into(), pack.current_a as f64, 0.011, 0.0),
        ("Percentage %".into(), pack.level_percent as f64, 0.0, 0.0),
        ("Cycles".into(), pack.cycles as f64, 0.0, 0.0),
        ("Protected State".into(), pack.protection_state as f64, 0.0, 0.0),
        ("Mos State".into(), pack.mos_state as f64, 0.0, 0.0),
        ("ch_+ 10mv".into(), pack.charger_input_v as f64 * 100.0, 0.5, 0.0),
        ("ch_det".into(), pack.charge_detect as f64, 0.0, 0.0),
    ];
    for (i, temperature) in pack.temperatures_c.iter().enumerate() {
        checks.push((format!("Battery Temperature_{}", i + 1), *temperature as f64, 0.051, 0.0));
    }
    for (i, cell) in pack.cell_voltages_mv.iter().enumerate() {
        checks.push((format!("cell_voltage_{} mv", i + 1), *cell as f64, 0.0, 0.0));
    }
    checks
        .into_iter()
        .filter(|(name, ours, tolerance, _)| (ours - row[&format!("Battery {side} {name}")]).abs() > *tolerance)
        .map(|(name, ours, _, _)| format!("{side} {name}: {ours} vs {}", row[&format!("Battery {side} {name}")]))
        .collect()
}

#[test]
fn every_report_matches_the_vendor_log() {
    let rows = vendor_rows();
    let reports = reports();
    assert_eq!(reports.len(), 10);
    for (stamp, report) in &reports {
        // The log samples rl_deploy's latest battery data once a second, so a frame shows up in the row of its own
        // second or, when it lands just after that sample, the next one.
        let next = rows.keys().filter(|row_stamp| *row_stamp > stamp).min();
        let differences: Vec<Vec<String>> = std::iter::once(stamp)
            .chain(next)
            .filter_map(|row_stamp| rows.get(row_stamp))
            .map(|row| [mismatches(&report.left, row, "L"), mismatches(&report.right, row, "R")].concat())
            .collect();
        assert!(differences.iter().any(Vec::is_empty), "{stamp}: {differences:?}");
    }
}

#[test]
fn static_fields() {
    for (_, report) in reports() {
        for pack in report.packs() {
            assert_eq!(pack.cell_count, 20);
            assert_eq!(pack.cells_mv().len(), 20);
            assert_eq!(pack.ntc_count, 5);
            assert_eq!(pack.temperatures().len(), 4);
            assert_eq!(pack.production_date, ProductionDate { year: 2024, month: 11, day: 30 });
            assert_eq!(pack.nominal_capacity_ah, 4.5);
            assert!(pack.charge_mos_on() && pack.discharge_mos_on());
            assert!(pack.protections().is_empty());
            assert!(pack.is_charging(), "docked and charging");
            assert_eq!(pack.serial_number, "");
            let sum: u32 = pack.cells_mv().iter().map(|&mv| mv as u32).sum();
            assert!((sum as f32 / 1000.0 - pack.voltage_v).abs() < 0.2, "cells sum to the pack voltage");
        }
        assert_eq!((report.left.software_version, report.right.software_version), (73, 65));
        assert_eq!((report.left.cycles, report.right.cycles), (97, 6));
    }
}
