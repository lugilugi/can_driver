# Changelog

## 5.0.0 — CAN V5 breaking migration

Release tag: `v5.0.0`

This release adopts the `ECT2026_CAN_V5` DBC as the authoritative wire
protocol. All ECT2026 firmware consumers must migrate together.

### Breaking changes

- Regenerated the typed protocol bindings from `network.dbc` with cantools
  41.4.3.
- Added the V5 vehicle, GPS, and motor telemetry messages.
- Changed generated message names and signal structures.
- Kept the transport classic-CAN-only with standard 11-bit identifiers.
- Kept message scheduling and freshness policy in application firmware.
- Rear light-board brake state now comes from a fresh `PEDAL_STATUS` frame,
  independently from `AUX_COMMAND` freshness.
- `inhibit_reason` values 6–15 round-trip through the codec but are reserved
  and invalid for application policy.

### v4 → v5 historical migration

The following mappings are historical compatibility notes only; the V5
bindings do not provide aliases for the removed V4 names:

| V4 identifier | V5 identifier | Migration note |
| --- | --- | --- |
| `DASH_STAT` at `0x400` | `VEHICLE_MOTION` at `0x400` | Same ID, incompatible payload semantics |
| `AUX_CTRL` at `0x210` | `AUX_COMMAND` at `0x210` | The auxiliary brake signal was removed |
| `PEDAL` at `0x110` | `PEDAL_STATUS` at `0x110` | Safety fields and names changed |
| `PWR_MONITOR_780` | `PACK_POWER` | Message and signal names changed |
| `PWR_MONITOR_740` | `AUX_POWER` | Message and signal names changed |
| `PWR_ENERGY` | `PACK_ENERGY` | Message and signal names changed |

The ESP Component Registry release is `5.0.0`, derived by the existing
publishing workflow from the `v5.0.0` tag.
