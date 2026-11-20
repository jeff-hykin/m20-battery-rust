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
    body[10 + 3 * 24 + 20] = 2; // control word 2
    assert!(zero_gain_poll::checked(&body).is_err());
    assert!(zero_gain_poll::checked(&m20_battery::charge::encode_charge_enable(4)).is_ok());
    assert!(zero_gain_poll::checked(&[0u8; 30]).is_err());
}
