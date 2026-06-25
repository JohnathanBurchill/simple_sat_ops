# any-sdr → main PR-readiness checklist

Working checklist from a subsystem-by-subsystem review of the `any-sdr` branch
(11 parallel reviewers over the current tree, 2026-06-20). No PR is imminent;
this is the punch list to work down first.

**How to work this:**
- Knock out **§A blockers** before anything else.
- Items tagged `(suspected)` were flagged by reasoning but not fully traced —
  **verify the bug is real before fixing**; some may be non-issues.
- **§C cross-cutting** items are each a clean standalone refactor commit and
  remove whole classes of "fixed in one copy, not the other" bugs.
- Check the box when done; reference the ID (e.g. `B1`) in the commit message.

Clean bill (no action): CSP/AX100/RS/Golay/scrambler framing, the IPC byte
parser, packet_db SQL (parameterized + finalized), the root setup scripts, and
the SDR device open/close + LO-leak-unmap lifecycle all checked out. No
confirmed memory-safety overrun in the live RX byte-parsing core.

---

## §A — Merge blockers (do first)

- [x] **B1 — CRITICAL / Concurrency** `rx_session.c:1493` + `sdr_uhd.c:485,288`
  TX burst and RX recv drive the *same* UHD device handle with **no lock** (the
  TX worker writes `set_tx_freq/gain/subdev` while the RX worker is in
  `uhd_rx_streamer_recv`). Zero sync anywhere in the SDR backend layer. The
  single biggest risk. Fix: per-core mutex held across recv and the whole TX
  burst, or a genuine RX hand-off for the burst window.
- [x] **B2 — HIGH / Bug+Safety** `auto_tcmd.c:727-730` (+ reload at `:447`)
  Auto-tcmd never enforces `TCMD_RF_MAX_LEN` (215); only clamps to the 2048-byte
  buffer, and the file is reloaded every open but never re-linted, so the startup
  `tcmd_lint_file` gate is bypassable for a post-launch edit. Fix: reject
  `strlen(wire) > TCMD_RF_MAX_LEN` in `auto_tcmd_tick`; re-lint on reload.
- [x] **B3 — HIGH / Bug** `prediction.c:233-257`
  One-step phase lag in the pass walk: elevation gate + AOS/LOS az/jul are
  sampled one `delta_t` out of phase with the altitude/azimuth they're paired
  with → AOS/LOS azimuths and duration off by one step. Fix: update position
  once at loop top, read el/alt/az from the same sample before advancing.
- [x] **B4 — HIGH / Bug** `prediction.c:534-538`
  Regex leak on the invalid-TLE early return (`if (!Good_Elements) { fclose;
  return -3; }` skips `regfree`). Fix: `regfree(&pattern)` (guarded on
  `criteria->regex != NULL`) before that return.
- [x] **B5 — HIGH / Bug** `tracking.c:611-612`
  Motion-settle uses exact float equality (`==0`); sub-step jitter leaves
  `antenna_is_moving` stuck at 1 and wedges the aim loop (`:722`) and scan-dwell
  (`scan_sky_tick :223`). Fix: tolerance compare (e.g. `< 0.05`).
- [x] **B6 — HIGH / Security** `beacon_detect.c:631-638`
  Shell injection: argv `wav_path`/`render_dir` interpolated into a `system()`
  string for `--render-png` (`"`-quoting trivially escaped). Fix: build an argv
  and `posix_spawn`/`execvp` ffmpeg directly.
- [x] **B7 — HIGH / Perf+Concurrency** `packet_db.c:238-245,409-421`
  The V3 `DELETE … WHERE id NOT IN (SELECT MIN(id) … GROUP BY …)` (+ V2/V4
  ALTERs) runs on *every* `packet_db_open` — full-table scan + write lock per
  short-lived tool launch, contending with the live receiver. `user_version` is
  read nowhere. Fix: read `user_version`, skip the migration ladder when current.

---

## §B — Should-fix (correctness / security / portability)

- [x] **S1 — HIGH / Concurrency** `rx_session.c:807,1374` + `decode_loop.c:42-104`
  `lo_offset_hz` and the `decode_loop` observer globals are written by the main
  thread and read by the RX worker outside the mutex (the adjacent CSV write *is*
  locked → reads as an oversight). A recorded packet can capture a half-updated
  observer frame. Fix: assign/read under `mu`, or snapshot the observer.
- [x] **S2 — HIGH / Concurrency** `rx_session.c:849,1478,1550` *(suspected benign)*
  `device_lost` written/read under `mu` in two places but read lock-free in
  `rx_session_device_lost()`; `volatile` ≠ memory ordering. Pick one discipline
  (atomic, or lock the poll too).
- [x] **S3 — HIGH / Bug** `gnss_opm.c:~269` *(suspected)*
  Fragment-id list truncated to 8 but reassembles all `n`; `--id=` lookup past
  the 8th fragment silently fails to match. Fix: store all ids or scan by id.
- [x] **S4 — HIGH / Bug** `gen_waterfall.c:607-635`
  Fragile in-place PDF page-object patch: if the rewritten body exceeds the
  reserved span, every captured xref offset is wrong → invalid PDF. Works today
  only because the placeholder is wide enough. Fix: assign `pages_id` before
  emitting the Page object, or buffer the body and write once.
- [x] **S5 — HIGH / Perf** `sso_ipc_codec.c` (decode path)
  Decoder is O(fields × line-length): ~70 `json_get_*`, each does a full
  `strlen` + linear rescan from `{`. Fix: parse once into a key→span map (at
  minimum drop the redundant per-call `strlen`).
- [x] **S6 — HIGH / Portability** `CMakeLists.txt:970,1027,1033`
  Install destinations hardcode `/usr/local/bin` and `/usr/local/share/sso/etc`,
  ignoring `CMAKE_INSTALL_PREFIX`; breaks DESTDIR staging / packaging. Fix: use
  `${CMAKE_INSTALL_PREFIX}/bin` (GNUInstallDirs).
- [x] **S7 — MED / Bug** `CMakeLists.txt:1023-1028`
  `file(GLOB) scripts/*.sh,*.py` installs dev/codegen scripts
  (`gen_tcmd_spec.py`, `lint_warnings.sh`, `self_test_smoke.sh`,
  `sso_setup_root.sh`) as operator binaries. Fix: install an explicit list.
- [x] **S8 — MED / Bug** `sso_admin.sh:364`, `self_test_smoke.sh`, `etc/logrotate.d/sso:5`
  Stale references to retired `b210_rx_live` → `sso_admin.sh verify` always
  reports FAIL on a correct install. Fix: drop it everywhere.
- [x] **S9 — MED / Bug** `modem_iq.c:215`, `modem_viterbi.c:137`
  `atan2(-s2i,-s2r)` undefined when both sums are exactly 0 (all-zero/DC window).
  Fix: `if (s2r==0 && s2i==0) bias=0;` first.
- [x] **S10 — MED / Portability** `prediction.c:543`, `pass_session.c:208-210`
  Use `isspace`/`isdigit` without `#include <ctype.h>` (transitive only), and
  pass plain (possibly negative) `char` to `isspace` → UB. Fix: include + cast
  `(unsigned char)`.
- [x] **S11 — MED / Security** `viewer.c:231-237` *(suspected)*
  `rx_ribbon_n` clamped only on the upper bound; a negative value from a garbled
  broadcast → huge `size_t` in `memcpy` and OOB nul write. Fix: `if (rn<0) rn=0;`.
- [x] **S12 — MED / Bug** `rx_session.c:884-892` *(latent)*
  `rx_session_request_burst_sync` can return an uninitialized result when the
  wait exits on `stop_requested` instead of `burst_complete`. Fix: distinct
  aborted return code. (Live path uses the async API, so latent.)
- [x] **S13 — MED / Design+Concurrency** `rx_session.c:1425-1453` + packet_db open flags
  `worker_record_sent_tcmd` writes packet_db from the TX thread concurrently with
  the RX worker, relying on SQLite "serialized mode." Verify the handle is opened
  `SQLITE_OPEN_FULLMUTEX` (or give the TX thread its own connection).
- [x] **S14 — MED / Bug** `rx_session.c:1278-1289` *(suspected, shadow-only)*
  PCM and IQ windows can desync when `iq_decode_pairs < n`, so the IQ/Viterbi
  shadow no longer sees the same time slice and the dedup absolute-sample label
  can be wrong. Fix: track a separate IQ-window sample counter.
- [x] **S15 — MED / Bug** `sdr_uhd.c:307-331` *(latent: multi-device)*
  `recv_err_run`/`md_err_run` are function-local `static`, shared across all UHD
  handles; two devices cross-contaminate the dead-link counter. Fix: move into
  `struct sdr_uhd`.
- [x] **S16 — MED / Bug** `antenna_rotator_async.c:77` *(latent: delays ≥1s)*
  `abs_deadline` carry is an `if`, not `while`; a residual ≥1e9 leaves `tv_nsec`
  out of range → `pthread_cond_timedwait` EINVAL/busy-spin. Fix: `while`.
- [x] **S17 — MED / Bug** `packet_db.c:127-131,612-617`
  `sha1_digest`/`tle_sha1` ignore all OpenSSL returns and don't null-check
  `EVP_MD_CTX_new()` → NULL-deref on the dedup hot path. Fix: check ctx + returns.
- [x] **S18 — MED / Bug** `CMakeLists.txt:171` *(suspected, reconfigure-order)*
  `set(WITH_RTL_SDR OFF)` is a normal var but `option()` cached it ON; on
  reconfigure the cached ON wins and the backend links nothing while the compile
  define stays set. Fix: `set(... OFF CACHE BOOL "" FORCE)` or an internal flag.
- [x] **S19 — MED / Bug** `monitor_squelch.c:30,34,42`
  `auto_offset_db != 0` rejects a deliberate 0 dB offset; default `noise_hi`
  inverts the band for `fs ≤ 18 kHz` → negative `bw_hz`. (Live path 96 kHz, so
  latent.) Fix: `> 0`-style default for offset only where 0 is invalid; validate fs.
- [x] **S20 — MED / Bug** `cli_args.c:1041` *(suspected)*
  `n_positional = argc - n_options - 1` infers positionals by subtraction; mixing
  option forms can miscount (reject a lone sat name / accept two). Fix: derive
  "too many" from a second-positional-seen flag (the `positional` ptr at `:437`
  is the real source of truth).
- [x] **S21 — MED / Bug** `main.c:139` + `cli_args.c:378-420` *(latent)*
  Defaults set in two places, most inside `apply_args`'s `if (!help)`; an early
  exit / new module caller gets a half-init struct (e.g. `serial_speed==0`). Fix:
  one initializer the struct can't be used without.
- [x] **S22 — MED / Bug** `oem.c:440` *(suspected)*
  Eccentric-anomaly seed `cosE0=(1-r0m/a)/max(e,1e-20)` is degenerate for
  near-circular orbits (e≈1e-4) → precision loss just outside the window
  (extrapolation only). Fix: `atan2`-based E0 from (sinE0,cosE0).
- [x] **S23 — MED / Security** `sso_admin.sh:419-421` *(latent)*
  `cmd_verify` forwards `declare -f` to `su - "$name" -c "…"`; static today but
  any future `$name`/`$key` interpolation becomes root-context injection. Fix:
  keep the forwarded body variable-free / pass data via env/stdin.
- [x] **S24 — MED / Bug** `tx_frame_sdr.c:559,634-692` *(suspected, offline tool)*
  `sps`/`repeat`/`gap_ms` from CLI with no clamp; `n_iq_total` has no overflow
  check before `calloc(n_iq_total*2,…)`. Fix: range-check the inputs.
- [x] **S25 — MED / Bug** `decode_inspector.c:1676-1688` *(OOM-only)*
  `decmode_diag_grow` strobe branch: a mid-way realloc failure leaves `d->`
  dangling (old block freed) and leaks the succeeded reallocs. Fix: mirror the
  lpf branch (store back each success).
- [x] **S26 — MED / Bug** `decode_inspector.c:1217` *(tiny-input only)*
  `apply_fir_complex` tail-zero loop `i = n_pairs - Mhalf` underflows `size_t`
  when `n_pairs < Mhalf` → OOB. Fix: `if (n_pairs > (size_t)Mhalf)` guard.
- [x] **S27 — MED / Bug** `live_waterfall.c:386` *(suspected)*
  `alloca(W*sizeof(float))` in the per-row hot path with `W` floored only at 200
  (no upper clamp) — large unchecked stack alloc, no failure path. Fix: `malloc`
  a scratch buffer once at init.
- [x] **S28 — MED / Design** `sso_ipc_codec.c:469-610` + `sso_ipc_internal.h:19`
  Worst-case encoded STATE/WELCOME nears the caller's 4096 buffer; on overflow
  `encode` returns -1 and the event is silently dropped (no log). `SSO_IPC_LINE_MAX`
  (8000) is declared but never used. Fix: size encode buffers from one shared
  constant; log/assert on encode failure.

---

## §C — Cross-cutting (each = one refactor commit, fixes a class)

- [~] **X1 — Duplication.** (Remaining sub-items tracked in #23.) Same logic copy-pasted across files; consolidate each. Five done in #23; the two below them left deliberately un-merged with rationale.
  - [ ] ASM-finder + M&M timing + boxcar matched filter copied ×4/×3 across
    `modem.c`/`modem_iq.c`/`modem_viterbi.c`/`modem_fsk.c`. ASM finder DONE →
    shared `src/dsp/asm_search` (e01debd, byte-identical, also folds the 4×
    `ASM_BIG_ENDIAN_U32` define). M&M timing + boxcar: CONSIDERED, DEFERRED
    (#23). `modem_fsk` does not use M&M at all — it runs Gardner timing + a
    Farrow cubic interpolator (different detector, opposite sign, different
    gains), so it can't share. The other three M&M loops are arithmetically
    identical but `modem_viterbi`'s strobe also emits the complex soft symbols
    the Viterbi decoder consumes (a load-bearing side-product), and the loops
    are inlined hot DSP. A shared block helper would change `modem_viterbi`'s
    output path and add per-symbol call overhead for ~60 lines saved, guarded
    only by the modem selftests. Not worth the risk; revisit only if a real
    M&M bug appears in one copy.
  - [x] Text-field editing (`*_field_insert/backspace/.../draw`) duplicated
    `tx_compose.c` ↔ `auto_tcmd.c` → shared `src/ui/ui_textfield` + TAP selftest
    (cbea41e). Callers keep their own char-acceptance + side effects.
  - [x] FFT (`is_pow2`/`fft_*`) + `VIRIDIS[256][3]` duplicated `live_waterfall.c`
    ↔ `waterfall_core.c` → exported `wf_is_pow2`/`wf_fft_forward` + already-public
    `WF_VIRIDIS`, linked `waterfall_core` into `live_waterfall`, dropped the local
    copies (dcc5f14, -100 lines; verified gen_waterfall still renders).
  - [x] `tcmd_response` grouping SQL + magic offsets (`packet_type=4`,
    `substr(payload,2,8)`, `substr(payload,13,1)`) across 5 files → standalone
    `src/proto/tcmd_response.h` (b67744a): offset constants, ts_sent/seq/max
    accessors, and the SQL fragments, with static asserts tying them to the
    firmware header where it's in scope. Each query's column list stays local
    (they genuinely differ). Verified `gnss_opm`/`gnss_reports` output is
    byte-identical on the live DB.
  - [x] `gnss_opm.c` ↔ `gnss_reports.c` (`starts_with`, `parse_time_spec`,
    `frag_t`, `reassemble`) → shared `src/proto/gnss_frag` (a16caf4). The
    tool-specific CONSIDER/FLUSH macros and trim logic stay local (they differ
    in semantics, not boilerplate).
  - [x] FM discriminator + min-mag squelch reimplemented in `b210_rx_capture.c`
    (×2) vs `b210_rx_tx_core` → shared `src/dsp/fm_demod.h` (29d47e1):
    `fm_demod_pcm` (clipped atan2 sample), `fm_demod_k_scale`, `fm_iq_mag_sq`.
    The loops stay local (the core/live monitor carry phase state across
    chunks; the WAV pass runs once and counts clipped/squelched), only the
    arithmetic is shared. Added `fm_demod_selftest` with an analytic tone
    oracle. `monitor_squelch` is a separate post-demod ratio detector, not
    part of this. (The two `b210_rx_capture` paths can't run headless here;
    guarded by the selftest + byte-exact extraction.)
  - [~] "newest .tle" discovery + TLE-name parsing. Name-line parsing
    consolidated → header-only `src/orbit/tle_io.h` (dd4e5a7):
    `tle_io_read_line` + `tle_io_is_element_line`, adopted by `pass_session.c`,
    `tracking.c`, `tle_keps.c` (verified `tle_keps --csv` identical over a
    23-object TLE). Left deliberately: the three discovery routines
    (`pass_session` newest-by-mtime, `next_in_queue` newest-dated-filename,
    `tle_keps` FrontierSat day-directory layout) are three distinct policies,
    not copies — folding them together relocates working code for no dedup.
    `prediction.c` (core loader) and `rx_replay.c` keep their own line readers
    (CR/LF-only trim, which the by-name prefix match + replay path rely on).
  - [~] Minor: `iso_utc` dup DONE → `sso_iso_utc_from_ts` in `sso_time.h`
    (ffea7b0). `tcmd_browser` ↔ `packet_browser` `format_ts`/`fmt_epoch_ms`
    DONE → header-only `utils/browser_timefmt.h` (5146425; `format_ts` now
    takes the local-time flag as an argument). `rx_decode` decode loop vs
    `decode_loop`: shared the subtle bit, the unframe + partial-RS rescue →
    `ax100_unframe_with_rescue` (a5638bc), called by all four
    `try_decode_window*` variants and `rx_decode`; `rx_decode` keeps its own
    loop (polarity sweep, preamble anchoring, diagnostics — forensic, not
    duplicated). `pass_schedule.c:36-53` private JSON reader vs the codec:
    CONSIDERED, NOT MERGED — different contracts (the codec is bounds-checked
    with full `\uXXXX` unescaping and brace-depth tracking; pass_schedule's is
    an intentionally minimal flat-object reader for a fixed numeric payload).
    Merging would regress one or bloat the other.
- [x] **X2 — Stale HMAC surface** (RX dropped HMAC, integrity is CSP CRC32):
  `state.h` hmac fields + `HMAC_DISPLAY_*` enum + "TX REFUSED" comments;
  `panels.c:617-636` HMAC keyfile row; `packet_db` `hmac_ok` column (note as
  legacy-rows-only); dead `use_hmac` plumbing in `rx_session.c:997,1049,1185`.
  Decide retain-for-uplink vs remove; at minimum stop the docs/UI contradicting
  the direction.
- [x] **X3 — Unclamped `off += snprintf(...)` accumulator** (truncation wraps
  `size`/`cap - off` to a huge value → latent OOB). Clamp `off` to buffer size
  after each write at: `panels.c:583-597`, `packet_browser.c:454-490`,
  `tcmd_browser.c:234`, `gnss_reports.c:389`. (Verified not currently reachable,
  but fix the pattern.)
- [ ] **X4 — `state_t` god-object** (tracked in #24) `state.h:283-480`: no single init, defaults
  split `main.c:139` / `cli_args.c:378-420` (see S21). Group into per-subsystem
  sub-structs with their own init/teardown (rotator/scan already are; tx/modal/
  ipc are not).
- [x] **X5 — Unchecked `atoi`/`atof` + unbounded size math from CLI** —
  size-math DONE (`b210_rx_capture.c` `--rate=0` div + buffer, `beacon_gen.c`
  total_cap, `gen_waterfall.c` `--fft` bound); `cli_args.c` numeric options now
  route through `parse_arg_long`/`parse_arg_double` (strtol/strtod + endptr;
  reject empty / trailing-garbage / out-of-range) (8e4b29d).

---

## §D — Long tail (LOW / NIT)

### DSP — #25
- [ ] `modem*.c` — `(size_t)sps * 32u` computed in 32-bit before widening; use
  `(size_t)sps * (size_t)32`.
- [ ] `b210_rx_tx_core.c:308-333,424-432` — display snapshots read unsynchronized
  (non-atomic `double`/int pair); document as intentionally racy or `_Atomic`.
- [ ] `modem_fsk.c:71-84` — `getenv("FSK_IQ_LPF_HZ")` buried in a DSP leaf as a
  function-local static; move the cutoff into `modem_params_t`.
- [ ] `sw_nco.c:49-66` — per-sample `cos`/`sin` instead of a phasor recurrence
  (perf, not correctness).
- [ ] `biquad.c:28` — `bw_hz<=0` → Inf/negative `Q` slips the `<0.5` clamp →
  silent all-stop filter; add an entry guard.
- [ ] `iq_burst.c:168` — `1e-6` power floor is meaningless vs int16² scale (comment
  oversells it).
- [ ] `modem_iq.c:159-176` — three "3." comment blocks describe a superseded
  per-sample differential the code doesn't do; trim.
- [ ] `b210_rx_tx_core.h` banner still says "USRP B210 RX core" vs device-agnostic
  `.c`; reconcile. `ASM_BIG_ENDIAN_U32` magic `#define`d in 4 files → shared header.

### SDR / hardware — #26
- [ ] `antenna_rotator_async.c:72-104` — `cond_timedwait` on CLOCK_REALTIME (wall
  jumps); note the dependency (CLOCK_MONOTONIC attr unavailable on macOS).
- [ ] `sdr_usb_detect.c:40-56` / `sdr_uhd.c:99,158-159` — first-B2xx serial scan
  ignores `device_index`; FPGA-map runs even when `--uhd-args` pins a serial;
  `get_rx_rate` return ignored (feeds decimation math).
- [ ] `sdr_backend.c:64`,`.h:41-45` — AUTO try-order array `[2]` hardcodes "two
  backends"; size from the enum.
- [ ] `antenna_rotator.c:108,164-167` — Rot2Prog SET (ASCII) vs STATUS parse (raw
  bytes) asymmetry — pre-existing, add a clarifying comment.
- [ ] `rotator_calibrate.c:233` — park-leg 0-samples mislabeled `PARK_TIMEOUT`
  (cosmetic).

### Protocol — #27
- [ ] `ax100.c:178` — `memcpy(inner, NULL, 0)` when `packet_len==0` (UB); guard.
- [ ] `ax100.c:206,279` — 255-byte scrambler `i % 255` only valid because peer
  matches; add a comment (diverges from a true 256-period randomizer if
  inner_len > 255).
- [ ] `ax100.c:182,303` — `mac[4]`/`expected[4]` not zero-init (written before use;
  house style). `ax100.c:308` — `memcmp` not constant-time (non-issue in current
  use; matters only if reused for uplink auth).
- [x] `hmac_keyfile.c:136-144` — "non-uppercase-hex" message also rejects lowercase
  (wording). Reworded to "non-hex or lowercase char ... (the key must be uppercase
  0-9 A-F)". Same pass added `unit_tests/hmac_keyfile_selftest.c` (28 assertions:
  permission-mode matrix + parser) and a `uplink_test --fix-permissions` repair
  action (`setfacl -b` + chmod) via new `hmac_keyfile_fix_permissions`.

### Pipeline — #28
- [ ] `rx_session.c:719,1385` — `long` sample counts wrap on a long pass; WAV
  `data_sz` (uint32) caps ~6.2 h @ 96 kHz. Widen to int64; document the WAV limit.
- [ ] `rx_session.c:259,1126` — `frames_in_window` written, never read (dead).
- [ ] `tx_burst.c:57-71` — `fm_modulate` carries phase across unrelated bursts via
  `static double phi`; thread it through a caller-owned var.
- [ ] `decode_loop.c:62-79` — NaN sentinels via `0.0/0.0`; use `NAN`.

### IPC — #29
- [ ] `sso_ipc_codec.c:256` — `\u` unescape `i+4 < srclen` should be `<=`
  (under-consumes one at buffer end; cosmetic).
- [ ] `sso_ipc_server.c:191-194` — `accept()` collapses all non-EAGAIN errors into
  `return`; `continue` on EINTR/ECONNABORTED, log on EMFILE/ENFILE.
- [ ] `sso_paths.h:18-23` — header doc claims `$HOME/FrontierSat` fallback; impl
  unconditionally uses `/FrontierSat`. `sso_ipc_paths.c`/`sso_paths.c` first-caller
  static caches fragile now a worker thread exists.
- [ ] `sso_ipc_client.c:194` — link-graph hack (`__attribute__((unused))` + fn-ptr
  cast) to force-link `sso_unix_user`; add the audit obj to the CMake link line.
- [ ] `sso_audit.c`/`sso_paths.c`/`sso_ipc_paths.c` — missing GPL header + `/* */`
  vs `//` style.

### UI — #30
- [ ] `tx_compose.c:143-150` — history/recall can momentarily exceed 215 in the
  field (validate catches at commit); re-clamp on populate.
- [ ] `cmd_line.c:434` — `(v < 1e6) ? v*1e6 : v` MHz/Hz heuristic is brittle
  (rejects valid low freqs). `cmd_line.c:334-646` — `cmd_dispatch` ~300-line
  if/else, `spectrum` nests ~9 deep → table-dispatch.
- [ ] `auto_tcmd.h:50-51` stale `auto_field_is_text` doc; `auto_tcmd.c:616-621`
  identical if/else bodies → collapse.

### Orbit — #31
- [ ] `prediction.c:651` — `find_passes` appends to the module-static list without
  clearing; document the precondition or `free_passes()` on entry. `:300-302`
  `minutes_until_visible` returns negative for already-visible (name vs sign).
  `:405-406` `pass_t.tle` duplicates `name` (dead/misnamed field). `:469` vs `:326`
  duplicated TLE-pack buffers, magic `69`.
- [ ] `oem.c:283-300` — `cmd_cap = 2*len+64` undersizes the `'`→4-byte escape worst
  case (bails safely but spuriously fails on quote-heavy ids); use `4*len+64`.
  `:419-424` misleading `(void)h` (h *is* used by `vxh`).
- [ ] `shortarc.c:144` — `n>64` silently drops observations (warn / share the cap).
  `:240` RMS divides by `n` not `dof` (inconsistent with chi-square above).
- [ ] `tle_csv.c:246-247` — `format_decexp` clamp `break` can leave the mantissa
  desynced from `a` (real BSTAR bounded → low impact).
- [ ] Document in `prediction.h`: "load_tle leaves raw units; caller must
  `select_ephemeris` exactly once" (non-idempotent).

### Control / apps — #32
- [ ] `cli_args.c:1170`/`next_in_queue.c` — `strdup(p->name)` never freed (lifetime
  leak; ownership of `satellite_ephem.name` is inconsistent).
- [ ] `main.c:341-345` — comment/CLAUDE.md say redraw "10 Hz" but it's 2 Hz.
  `:610-611` — fixed `usleep` means the cadence gates are upper bounds, not
  schedules (note it).
- [ ] `pass_session.c:168-180` — `unlink; symlink` is not atomic and ignores
  `unlink` failure; use symlink-to-temp + `rename`.
- [ ] `lifetime.c:41` — unsanitized satellite name → `/tmp` path (local toy).
- [ ] `tracking.c:598-828` — `tracking_tick` ~230 lines / 5 responsibilities with a
  stray extra indent; `jul_idle_start` is dead (`(void)`-cast).
- [ ] Path-buffer sizes hand-tuned across apps (`[1024]`/`[256]`/`[512]`/`[1100]`) →
  shared `#define` (truncation checks are present/correct).

### DB / build — #33
- [ ] `packet_db.c:690-704` — `register_tle` returns 0 for both real error and
  post-timeout `SQLITE_BUSY`; distinguish. `:633-643` `parse_tle_line1` trusts
  fixed offsets after a weak gate (no overflow; data-quality only).
- [ ] `sso_admin.sh:304` — key appended at umask default before `chmod 600`; use
  `install -m 600` / umask-guard.
- [ ] `CMakeLists.txt:30,37` — `-O0 -g` hardcoded (no `CMAKE_BUILD_TYPE`);
  `-Wformat-truncation` without `=N` is a clang no-op. `gen_tcmd_spec.py:106,111`
  — `open()` without `with`/encoding.

### Utils (decode/waterfall, packet/offline) — #34
- [ ] `gen_waterfall.c:751-1109` — ~360-line dead `#if 0 build_waterfall_legacy`
  block (has `const`-cast-away UB if revived); delete.
- [ ] `b210_rx_capture.c:838` — `peak_pct` round constant off by one (diagnostic).
- [ ] `decode_inspector_macos.m:20-32` — `g_..._pinch_delta` unsynchronized
  float (best-effort; note/`_Atomic`).
- [ ] `tcmd_import.c`/`tcmd_browser.c:~192` — response index caps at 100000 silently
  (undercount past cap); SQL `GROUP BY` or warn. `tcmd_import.c` — `json_str`
  matches bare `"ts"` via `strstr` (substring of `"tssent"`); anchor.
- [ ] `gnss_opm.c:~502`/`gnss_reports.c:~531` — fragment `malloc` failure then
  deref in `reassemble`; on OOM set `payload_len=0`/skip.
- [ ] `rx_replay.c:~284`/`rx_decode.c:~105` — empty input (`n==0`) misreported as
  IO error; special-case. `tx_frame_sdr.c:~739` — ignored `fclose` return on
  `--dump-iq`.
- [ ] Style: EOL `/* */` comments vs `//` (multiple); `(0.0/0.0)` vs `NAN`
  (`rx_replay.c:786,818`); terse `s`/`n` RMS names; duplicated `0xC0FFEEu` seed.
