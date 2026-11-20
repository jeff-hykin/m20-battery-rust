//! zero_gain_poll's frame is rl_deploy's sitting frame byte for byte (from the docked capture), apart from the sequence.

#[path = "../examples/zero_gain_poll.rs"]
#[allow(dead_code)]
mod zero_gain_poll;

const CAPTURE: &[u8] = include_bytes!("fixtures/sitting_docked_20261120_125557.pcapng");

#[test]
fn identical_to_rl_deploys_sitting_frame() {
    let mut checked = 0;
    for (_, direction, frame) in m20_ecan::tap::read_capture(CAPTURE).unwrap() {
        if direction == m20_ecan::Direction::ToBoard && frame[19] == 0x89 {
            let body = &frame[14..14 + 10 + 384];
            let ours = zero_gain_poll::passive_body(body[2]);
            assert_eq!(ours.as_slice(), body);
            assert!(zero_gain_poll::checked(&ours).is_ok());
            checked += 1;
        }
    }
    assert_eq!(checked, 40);
}

#[test]
fn gate_refuses_any_gain() {
    let mut body = zero_gain_poll::passive_body(0);
    assert!(zero_gain_poll::checked(&body).is_ok());
    body[10 + 5 * 24 + 16] = 1; // kp of one joint, checksum now stale but the gate decodes first
    assert!(zero_gain_poll::checked(&body).is_err());
    let mut body = zero_gain_poll::passive_body(0);
    body[10 + 3 * 24 + 20] = 5; // control word 5 (motor reset): not in the handshake
    assert!(zero_gain_poll::checked(&body).is_err());
    for word in zero_gain_poll::ALLOWED_WORDS {
        let mut body = zero_gain_poll::passive_body(0);
        for unit in 0..16 {
            body[10 + unit * 24 + 20..10 + unit * 24 + 22].copy_from_slice(&word.to_le_bytes());
        }
        assert!(zero_gain_poll::checked(&body).is_ok(), "{word}");
        body[10 + 3 * 24 + 18] = 10; // kd 1.0 on one joint: refused whatever the word
        assert!(zero_gain_poll::checked(&body).is_err(), "{word}");
    }
    assert!(zero_gain_poll::checked(&m20_battery::charge::encode_charge_enable(4)).is_ok());
    assert!(zero_gain_poll::checked(&[0u8; 30]).is_err());
}

#[test]
fn gate_takes_rl_deploys_setup_frames_only() {
    // the captured version request and a setup 0x81 frame pass; a 0x81 with another word or value does not
    let version = m20_ecan::frame::encode(2, 1, 0x11, m20_ecan::FrameType::Version, &[]).unwrap();
    assert!(zero_gain_poll::checked(&version).is_ok());
    let mut units = Vec::new();
    for leg in 0..4u8 {
        for id in 2..6u16 {
            units.extend_from_slice(&[0x79, leg]);
            units.extend_from_slice(&id.to_le_bytes());
            units.extend_from_slice(&1300u32.to_le_bytes());
            units.extend_from_slice(&[0; 12]);
            units.extend_from_slice(&14u16.to_le_bytes());
            units.extend_from_slice(&[0, 0]);
        }
    }
    let ok = m20_ecan::frame::encode(6, 1, 0x84, m20_ecan::FrameType::Type81, &units).unwrap();
    assert!(zero_gain_poll::checked(&ok).is_ok());
    let mut other = units.clone();
    other[4] = 0x15; // value 1301
    assert!(zero_gain_poll::checked(
        &m20_ecan::frame::encode(6, 1, 0x84, m20_ecan::FrameType::Type81, &other).unwrap()
    )
    .is_err());
    let mut reset = units.clone();
    reset[20] = 5; // motor reset word
    assert!(zero_gain_poll::checked(
        &m20_ecan::frame::encode(6, 1, 0x84, m20_ecan::FrameType::Type81, &reset).unwrap()
    )
    .is_err());
}

/// With ZERO_GAIN_REPLAY=<file>, every frame of a replay file must pass the gate (renumbered).
#[test]
fn replay_file_passes_the_gate() {
    let Ok(path) = std::env::var("ZERO_GAIN_REPLAY") else { return };
    let mut n = 0;
    for line in std::fs::read_to_string(path).unwrap().lines() {
        let hex = line.split_whitespace().nth(2).unwrap();
        let bytes: Vec<u8> =
            (0..hex.len()).step_by(2).map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap()).collect();
        let mut body = bytes[14..].to_vec();
        body[2] = n as u8;
        zero_gain_poll::checked(&body).unwrap_or_else(|e| panic!("line {n}: {e}"));
        n += 1;
    }
    assert!(n > 0);
}
