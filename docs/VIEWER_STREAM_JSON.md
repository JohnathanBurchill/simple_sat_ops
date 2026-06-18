# `simple_sat_ops --viewer-stream` — JSON contract

Reference for clients that read the headless telemetry stream (e.g. the
iOS/iPadOS viewer over SSH). This is the wire format `simple_sat_ops
--viewer-stream` writes to **stdout**.

Applies to: commit `2e7a084` (2026-06-16). The format is additive and
forward-compatible — see [Parser rules](#parser-rules) — so a client built
against this revision keeps working as fields are added.

---

## 1. How to run the producer

```
simple_sat_ops --viewer-stream            # auto-discovers the newest TLE
simple_sat_ops --viewer-stream <sat> --tle <path>
simple_sat_ops --viewer-stream next       # auto-pick the next pass
```

It opens **no hardware**, binds **no server**, loads **no signing key** — it
only reads the sky and writes JSON. It runs whether or not a control operator
is up. `--control` and `--viewer-stream` together are refused.

Typical remote use: `ssh ground-station simple_sat_ops --viewer-stream` and
read the child process's stdout line by line.

---

## 2. Framing

- **One event per line**, terminated by `\n`. Parse line-by-line.
- Each line is a **flat JSON object**. There are no nested objects, with one
  exception: `roster` is a JSON **string** whose contents are a JSON array
  (parse it as a second step if you need it).
- Every line carries:
  - `t` — event type (string, see [§5](#5-event-types)).
  - `ts` — ISO-8601 UTC timestamp, millisecond precision, e.g.
    `"2026-06-16T15:36:57.511Z"`.
- The stream is **continuous**: ~1 Hz in tle-only mode, ~2 Hz when relaying an
  operator. A line arriving = the link is alive; no separate heartbeat event.

---

## 3. Mode discriminator: `source`

The producer has two modes and switches between them automatically. Every
`state` and `welcome` line tells you which mode produced it via `source`:

| `source`      | meaning |
|---------------|---------|
| `"tle-only"`  | No operator is running. The producer loaded the TLE and is propagating the satellite itself (SGP4). |
| `"operator"`  | A control operator is up; this line was relayed from it. |

Over `--viewer-stream`, `source` is **always present** on `state`/`welcome`.
Treat it as the authoritative mode flag and read it off each line.

Transitions are seamless: with no operator you get `tle-only` lines; the
producer notices the **moment** a `--control` operator starts — it watches the
runtime directory for the operator's socket — and switches to `operator` lines
within a tick; when the operator drops it falls back to `tle-only`. No gap —
the stream never goes silent during a handover beyond the normal tick.

> The non-`state` event types (§5.3) only ever appear in `operator` mode, so
> seeing one also implies an operator is connected.

---

## 4. The `state` event

This is the event a viewer renders. Field-conditional encoding is used, so
**absence is meaningful** (see [Parser rules](#parser-rules)).

### 4.1 Field table

| key | JSON type | present | meaning |
|-----|-----------|---------|---------|
| `sat` | string | always | Satellite name |
| `source` | string | always (over viewer-stream) | Mode discriminator (§3) |
| `p_az` | number | always | **Satellite** sky azimuth, deg (SGP4) |
| `p_el` | number | always | **Satellite** sky elevation, deg (SGP4) |
| `az` | number | always | **Rotator** azimuth, deg (hardware); `0` in tle-only |
| `el` | number | always | **Rotator** elevation, deg (hardware); `0` in tle-only |
| `target_az` | number | always | Commanded rotator target az, deg; `0` in tle-only |
| `target_el` | number | always | Commanded rotator target el, deg; `0` in tle-only |
| `freq` | integer | when ≠ 0 | Downlink carrier, Doppler-shifted, Hz |
| `doppler` | number | when ≠ 0 | Per-tick carrier delta, Hz (small; usually absent in tle-only) |
| `rng` | number | always | Slant range to satellite, km |
| `rrate` | number | always | Range rate, km/s (negative = approaching) |
| `alt` | number | always | Sub-satellite altitude, km |
| `lat` | number | always | Sub-satellite latitude, deg |
| `lon` | number | always | Sub-satellite longitude, deg (east positive) |
| `spd` | number | always | Orbital speed, km/s |
| `mv` | number | always | Predicted minutes until AOS (next visible) |
| `ma0` | number | always | Predicted minutes above 0° elevation this pass |
| `ma30` | number | always | Predicted minutes above 30° elevation this pass |
| `max_el` | number | always | Predicted max elevation of the pass, deg |
| `idesg` | string | when set | International designator (e.g. `"98067A"`) |
| `ep_min` | number | always | Minutes since the TLE epoch |
| `jul` | number | when ≠ 0 | Julian date of this tick |
| `in_pass` | bool | when true | tle-only: satellite is above the horizon. operator: in the tracking-prep window (flips a few min before AOS) |
| `tracking` | bool | when true | Operator is actively tracking (operator mode only) |
| `flip` | bool | when true | Flip-mode pass (operator mode only) |
| `has_rot` | bool | when true | Operator has a rotator attached (absent in tle-only) |
| `tle_path` | string | when set | Source TLE file path |
| `operator` | string | when set | Operator's (or stream's) Unix user |
| `roster` | string (JSON array) | when present | Connected users: `[{"user","role","since"},…]` (operator mode) |
| `at_on` | bool | when true | Auto-telecommand run in progress (operator mode) |
| `at_sent` | integer | with `at_on` | Telecommand bursts sent so far |
| `at_tot` | integer | with `at_on` | Planned total bursts |
| `at_st` | string | with `at_on` | `"running"`/`"stopped"`/`"done"`/`"pass-over"` |
| `rx_warn` | string | when set | Operator-wide warning row (e.g. low disk) |
| `rx_has` | bool | when true | Operator has a live RX session — the `rx_*` block below follows |

When `rx_has` is true (operator mode with an SDR), these accompany it:

| key | JSON type | meaning |
|-----|-----------|---------|
| `rx_rec` | bool | Recording active |
| `rx_fhz` | number | RX tuned frequency, Hz |
| `rx_pk` | number | Peak level, dBFS |
| `rx_rm` | number | RMS level, dBFS |
| `rx_fr` | integer | Frames decoded (IQ chain) |
| `rx_fr_pcm` | integer | Shadow PCM/FM frame count |
| `rx_fr_vt` | integer | Shadow Viterbi-MLSE frame count |
| `rx_lf` | string | Last frame summary |
| `rx_age` | number | Age of last frame, s (< 0 = none yet) |
| `rx_pt`N`_c` | integer | Per-type packet count, slot N (0–5) |
| `rx_pt`N`_l` | integer | Payload preview length, slot N |
| `rx_pt`N`_p` | string | Payload preview, hex-encoded, slot N |
| `rx_pt`N`_s` | string | Per-type summary line, slot N |
| `rx_rb` | string | Activity ribbon: one `.`/`-` char per second |
| `rx_rb_p` | string | Parallel peak-dBFS per second, hex int8 (two's-complement) |

### 4.2 What differs between modes

Same field *set* in both modes — write **one** decoder. The difference is
which optional fields are populated:

- **tle-only**: `sat`, `source:"tle-only"`, `tle_path`, the full prediction
  block (`p_az p_el rng rrate alt lat lon spd mv ma0 ma30 max_el idesg
  ep_min`), `freq` (Doppler-shifted), `jul`; `az el target_az target_el` are
  `0`; `in_pass` appears only when above the horizon. **No** `has_rot`,
  `tracking`, `flip`, `roster`, `at_*`, or `rx_*`.
- **operator**: all of the above plus `source:"operator"`, real `az el`,
  `has_rot`, possibly `tracking`/`flip`/`in_pass`, `roster`, `at_*`, and the
  `rx_*` block.

> **Sky vs. boom:** `p_az`/`p_el` are the *satellite's* position — drive the
> sky display from these. `az`/`el` are the *rotator boom* and are `0` in
> tle-only (no hardware). Don't use `az`/`el` for the satellite marker.

---

## 5. Event types

### 5.1 `state`
The telemetry tick (§4). Emitted in both modes.

### 5.2 `welcome`
Sent once right after the producer connects to an operator. Same state block
as `state` (carries `source:"operator"`), giving an immediate snapshot rather
than waiting for the next tick. Treat it exactly like a `state`.

### 5.3 Operator-only events (relayed verbatim, re-tagged)
These appear only while relaying an operator. A client may ignore any it
doesn't need:

| `t` | purpose | notable fields |
|-----|---------|----------------|
| `tx-command-sent` | a telecommand reached the air | `ascii`, `tx_pl`, `tx_kind`, CSP `tx_src/tx_dst/tx_dp/tx_sp/tx_prio`, `tx_freq`, `tx_gain` |
| `tx-preview` | operator is composing a telecommand (debounced draft) | same as above |
| `tx-not-sent` | a telecommand did **not** reach the air | above + `tx_st` (reason) |
| `cmd-preview` | operator's live `:` command-line buffer | `cmd_text` |
| `cmd-executed` | a command-line was dispatched | `cmd_text`, `cmd_status` |
| `rx-stats` | periodic RX stats | `snr_db`, `packets`, `last_packet_ts`, `last_packet_summary` |
| `operator-changed` | operator handover | `prev`, `new` |
| `yield-request` | force-claim in progress | `reason` |
| `bye` | operator is shutting down | — |

### 5.4 `passes` — upcoming-pass schedule

A self-contained list of the next ~7 days of passes over the ground station,
emitted **once near the start of every stream** (each SSH connection spawns its
own producer) and refreshed every few hours on long-lived streams. Unlike the
operator-relayed events, the producer computes this itself from its loaded TLE
(SGP4 + the observer location), so it appears in **both** modes. It lets a
read-only client show a pass list and schedule local alerts even when the link
is down — cache the most recent one.

It is **not** an `sso_event_t` (it would bloat that struct); it has its own
codec, `src/ipc/pass_schedule.c`, kept in lockstep with the standalone viewer's
vendored copy.

| key | JSON type | meaning |
|-----|-----------|---------|
| `t` | string | `"passes"` |
| `sat` | string | Satellite name |
| `idesg` | string | International designator |
| `gen` | number | When this schedule was computed, **Unix seconds UTC** (monotonic; advance = a newer schedule) |
| `ep_min` | number | Minutes since the TLE epoch (staleness hint) |
| `n` | integer | Number of passes in `p` |
| `p` | array of numbers | `n` × 5 flat: `[aos, los, peak, peak_el_deg, peak_az_deg, …]` |

Each pass is five consecutive numbers in `p`: `aos`/`los`/`peak` are **Unix
seconds UTC** (acquisition, loss, and culmination time); `peak_el_deg` is the
maximum elevation; `peak_az_deg` is the azimuth at culmination. Passes are
sorted soonest-first. A decoder should keep only whole 5-tuples (ignore a
trailing partial) and may cap at its own limit.

---

## 6. Parser rules

Build a tolerant decoder:

- **Missing field = default.** A missing string is `""`, a missing number is
  `0`, a missing bool is `false`. Many fields are emitted only when non-empty
  / non-zero / true — do not require them.
- **Numbers use C `%.6g`.** Large magnitudes appear in scientific notation
  (e.g. `"jul":2.46121e+06`, `"ep_min":705080`). Your number parser must
  accept both fixed and exponent forms. `freq`, `rx_fr`, `at_*`, CSP fields,
  and `packets` are integers; everything else numeric is a double.
- **Unknown `t` and unknown keys are fine.** Ignore event types you don't
  handle and keys you don't model — the format only grows.
- **`roster` is a string.** Its value is a JSON array; parse it separately if
  needed.
- **Strings are JSON-escaped** for `" \ \n \r \t`. Control characters that
  can't be represented decode lossily (to `?`) — don't rely on exotic bytes
  surviving.
- **Don't assume a fixed key order.**

---

## 7. Examples

tle-only (no operator; the satellite is below the horizon here, so `in_pass`
is absent):

```json
{"t":"state","ts":"2026-06-16T15:36:57.511Z","operator":"john","sat":"OSCAR 7 (AO-7)","source":"tle-only","az":0,"el":0,"freq":436154570,"tle_path":"TLEs/amateur.tle","target_az":0,"target_el":0,"jul":2.46121e+06,"idesg":"74089B","ep_min":705080,"mv":28,"ma0":20.8,"ma30":6.8,"max_el":42.5891,"p_az":29.69,"p_el":-57.8283,"alt":1457.52,"lat":2.77747,"lon":40.8607,"spd":7.13106,"rng":12470.4,"rrate":-3.14168}
```

operator (relayed; rotator tracking, satellite up). Same shape, plus the
hardware/rotator fields:

```json
{"t":"state","ts":"2026-06-16T15:40:01.002Z","operator":"alice","sat":"FRONTIERSAT","source":"operator","az":123.4,"el":12.1,"has_rot":true,"tracking":true,"in_pass":true,"target_az":124.0,"target_el":12.5,"freq":436151230,"jul":2.46121e+06,"idesg":"24001A","ep_min":1503,"mv":-2.1,"ma0":9.5,"ma30":3.0,"max_el":71.5,"p_az":124.2,"p_el":12.0,"alt":512,"lat":50.1,"lon":-114.0,"spd":7.61,"rng":1840,"rrate":-3.25,"roster":[{"user":"alice","role":"operator","since":""},{"user":"john","role":"external","since":""}]}
```

---

## 8. Suggested client model

A minimal viewer needs only the `state` event:

1. Read lines; JSON-decode each; switch on `t`. Keep only `t == "state"` (and
   `t == "welcome"`, handled identically) for the dashboard.
2. Track current mode from `source`; show a "TLE prediction (no operator)" vs
   "Live operator" badge.
3. Sky position from `p_az`/`p_el`; range/Doppler from `rng`/`rrate`/`freq`;
   pass timing from `mv`/`max_el`/`ma0`/`ma30`; ground track from
   `lat`/`lon`/`alt`.
4. Liveness: if no line for, say, > 5 s, show "stale".
5. Optionally surface `rx-*`, `tx-*`, `cmd-*`, and `roster` when in operator
   mode for a fuller view.
