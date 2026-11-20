# m20-battery

The Deep Robotics M20 (Lynx) battery packs, as the ECAN board reports them once a second on the motor bus
(decoded with [`m20-ecan`](https://crates.io/crates/m20-ecan)): voltage, current, level, remaining/nominal capacity,
cycles, production date, temperatures, all 20 cell voltages, balancing, protection state (by name), MOS state,
charger input and charge detect, for both packs.

```rust
use m20_battery::BatteryReport;
use m20_ecan::{tap, Direction, FrameType, Frame};

let bytes = std::fs::read("capture.pcapng")?;
for (_, direction, frame) in tap::read_capture(&bytes)? {
    if direction == Direction::FromBoard && Frame::parse_ethernet(frame)?.header.frame_type == FrameType::Battery {
        let report = BatteryReport::from_ethernet(frame)?;
        println!("left {}% {:.2} V, right {}% {:.2} V", report.left.level_percent, report.left.voltage_v,
            report.right.level_percent, report.right.voltage_v);
    }
}
# Ok::<(), Box<dyn std::error::Error>>(())
```

`charge` holds the charge-enable request (the 0xb3 frame, byte for byte as rl_deploy sends it) and a rule for when
to send it from the packs' own charge-detect / contact-voltage / charge-MOS readings and lie-down events, with no
DrDDS and no timer. It only builds bytes; whatever owns the motor bus sends them.

Tested against a real capture and the vendor's own battery log of the same seconds.

License: Apache-2.0
