# Week 8 review (Stage 2 gate)

Self-review of the Stage 2 diff (W-01..W-15) against the brief's STEP 3 text, the plan,
HANDOVER sections 5-6 and the OWNER CORRECTIONS. Evidence: `week8-verify.md`.

## Checked

| Area | Finding |
|---|---|
| OWNER CORRECTION 0 (names) | only Team Information values; no new names |
| OWNER CORRECTION 1 (D2) | D2 row unchanged ("PROPOSED, TODO(owner-to-confirm)"); timing.h text unchanged; TEAM.md and subsystem1 say "not approved" |
| OWNER CORRECTION 2 (fault record) | magic + CRC, print/clear only when both match, else silent re-init: in ARCHITECTURE.md D6, docs/DEBUG.md, DATA_REPRESENTATION.md section 5. `.uninitialized_data`, not `.noinit` (checked in `cmake/memmap_mtk3.ld:191, 217-220`) |
| OWNER CORRECTION 3 (obstacle) | TMO_POL send failure cited from `messagebuf.c` on disk; drain-then-send, race and 2-slot note documented; 2-bit sequence KEPT, TODO(owner-to-confirm, buddy 5) |
| OWNER CORRECTION 4 (NFR2) | exact proposed wording in D7 with TODO(owner-to-confirm, team), reason and top-speed test; no team document edited |
| Section 6 decisions | servo GP15; PIO0 one SM per wheel; `enc_get_count()` prototype only; pitch ±45° @0.5° 8 bits; motion_cmd 19 bits; obstacle clearances, no timestamp, no bypass state, 3-bit bypass command in nav_input_t; system FSM recount = 10 -> 4 bits; EVF_SAFETY wait `TWF_ORW|TWF_BITCLR`, FAULT_LATCHED, no-echo informational, `tk_ref_flg`; `BC_CAPTURE_TIMEOUT_MS` TUNE + static assert; ENC_CPR / WHEEL_CIRC_MM symbolic 0; language-neutral dashboard table; Week 13 statement |
| Pins | map matches the brief; every check fires under mutation (WV-3); UART0 rejected at configure and by `#error` (WV-2) |
| Scope | no packed types, no pack/unpack code, no encoder or PIO code, no `.pio` file, no ipc/port/dbg/main.c |
| Rules | `drv_ir.c` SDK only; `drv_*.h` `<stdint.h>` only; `check_rules` PASS; `tools/` scanned (json_sizes.cmake has no R4/R5 strings) |
| Never-edit files | unchanged; README.md / .gitignore untouched |
| NOTICE.md | append only; BSD-3 text matches the source |

## Findings fixed during review

1. DATA_REPRESENTATION.md first gave `obstacle_profile_t` as "37 bits + sequence = 39"; the 39 in
   the impact report included the removed bypass state. Corrected to 37 bits with the sequence
   (35 without), 5 B wire.
2. `nav_input_t` packed size cell said 2 B while the row total is 17 bits. Corrected to 4 B.
3. `messagebuf.c` line citations corrected to the on-disk lines (:315, :319, :320).
4. ARCHITECTURE.md D3 and D10 "reason" cells first stated reasons not recorded anywhere.
   Reworded to the recorded sources.

## Judgement calls (recorded as assumptions)

- W-03 constant names follow the brief (`ADC_*`) rather than the plan (`IR_SANITY_*`).
- The encoder overflow `#error` checks one second of counts at top speed, which covers the
  20 ms delta and avoids a timing.h dependency in thresholds.h.
- EVF_SAFETY bit positions, the split of the IMU fault into I2C and watchdog bits, and
  `FAULT_IR_SENSOR` are proposals.
- Telemetry roll uses the pitch format; JSON reference values and binary payload layouts are
  planning figures.

Verdict: APPROVED.
