# Vehicle Configuration Presets

Ready-to-upload master bridge configuration profiles (`Settings → Advanced → Configuration JSON → Upload`).

## `subaru_2009_legacy_b4_ej204.json`

Verified preset for a **2009 Subaru Legacy B4 2.0R** (BM chassis, EJ204, 4EAT, 500 kbps CAN, ECM `5B443C4107` / `A11009` / `D30Z4EC3`).

### Key hardening & cleanup applied
- **Bus protection & timing:** `guard: true`, `rx_guard: true`, `tx_passive: true`, `tx_hold: true`, `start_delay_s: 5`, `can_timing: 0` (`80 %`, SJW 3) with `can_sp875: false`, `req_max_hz: 20`, `obd_gap: 20`, `obd_to: 80`, `obd_p2can: 50`, `radio_dbm: 13`.
- **SSM2 batching:** `ssm_batch: 1`, `ssm_gap: 200`, `ssm_switches: false` — avoids `7F A8 12` rejections on multi-address `0xA8` reads on ECM `5B443C4107`.
- **No duplicate module polling:** `reads` polls only BIU physical `0x750 → 0x758` (`21 30`, every 500 ms), removing the colliding `0x700 → 0x758` functional entry.
- **Cleaned passive CAN signals (25 entries):**
  - Removed 13 dead Impreza-generation frames (`0x0D0`, `0x0D4`, `0x144`, `0x282`, `0x374`) and 8 broken/duplicate learned signals.
  - Corrected `0x231` RPM (`1.0 / 0.0`), `0x231` B4 Relative Pedal (`0.212 / 0.0`), `0x431` Door FL / In Park collision, and remapped `0x705` B3–B4 to **Fuel Rate** (`0x015E`, `0.005 / 0.0`) instead of MAF (`0x0110`).
  - Added verified signals: `0x253` TCU Turbine RPM (`0x3003`), `0x231` Engine Running (`0x3001`), `0x232` ECM State (`0x3002`), `0x333` Brake Switch VDC (`0x2113`), `0x431` Gear in Park (`0x1209`), and disabled candidate `0x706` Cluster/ATF Temp (`mode: 0`) pending warm-drive verification.
- **OBD-II & SSM2 pruning:** Disabled all unsupported PIDs (including `0x5C` Oil temp, `0x5D` Injection timing), disabled passive-CAN duplicate PIDs, disabled unsupported SSM2 addresses (including `0x113` Oil temp), and enabled verified Subaru-only SSM2 parameters (`0x032` CPC Duty, `0x03C/0x03D` Intake VVT R/L, `0x03E/0x03F` Intake OCV R/L, `0x0CE–0x0D9` cylinder roughness, `0x020` Inj 1 PW, `0x028` Learned timing, `0x03A` Alternator duty, `0x199` Fine knock corr, `0x271/0x273` battery current/temp).
