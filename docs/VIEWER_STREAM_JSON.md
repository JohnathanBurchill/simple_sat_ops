# `simple_sat_ops --viewer-stream` — JSON contract

Reference for clients that talk to the headless stream (e.g. the iOS/iPadOS
viewer over SSH). It covers:

- the **outbound telemetry** the producer writes to **stdout** (§1–§8); and
- the **inbound command channel** the client writes to **stdin**, plus the
  **live audio** path it unlocks (§9).

This is the wire format `simple_sat_ops --viewer-stream` uses.

Applies to: commit `0d134d2` (2026-06-22). The format is additive and
forward-compatible — see [Parser rules](#parser-rules) — so a client built
against an earlier revision keeps working as fields and event types are added.
Audio and the inbound channel are purely additive: a client that never writes to
stdin and ignores the audio events behaves exactly as a telemetry-only client.

---

## 1. How to run the producer

```
simple_sat_ops --viewer-stream            # auto-discovers the newest TLE
simple_sat_ops --viewer-stream <sat> --tle <path>
simple_sat_ops --viewer-stream next       # auto-pick the next pass
```

It opens **no hardware**, binds **no server**, loads **no signing key** — it
only reads the sky, relays a running operator, and reads/writes JSON. It runs
whether or not a control operator is up. `--control` and `--viewer-stream`
together are refused.

Typical remote use: `ssh ground-station simple_sat_ops --viewer-stream` and read
the child process's stdout line by line. To use live audio (§9), also write
command lines to that child's stdin.

| flag | meaning |
|------|---------|
| `--no-audio` | Refuse all audio requests for this producer. Telemetry is unaffected. |

Live audio additionally requires the ground station's build to include
**libsndfile** (the same dependency `ham_listen` uses); without it audio
requests are answered `"unavailable"`. Telemetry never depends on it.

---

## 2. Framing (both directions)

- **One event per line**, terminated by `\n`, in **both** directions. Parse
  line-by-line; write one complete line at a time.
- Each line is a **flat JSON object**. There are no nested objects, with one
  exception: `roster` is a JSON **string** whose contents are a JSON array
  (parse it as a second step if you need it).
- Every line carries:
  - `t` — event type (string, see [§5](#5-event-types)).
  - `ts` — ISO-8601 UTC timestamp, millisecond precision, e.g.
    `"2026-06-16T15:36:57.511Z"`. (Outbound only; inbound commands need no `ts`.)
- The outbound stream is **continuous**: ~1 Hz in tle-only mode, ~2 Hz when
  relaying an operator. A line arriving = the link is alive; no separate
  heartbeat event.
- Lines longer than ~8 KB are rejected in both directions. Keep commands small;
  audio frames are sized to stay well under the limit (§9.4).

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

> The non-`state` event types (§5.3) and **live audio** (§9) only ever appear in
> `operator` mode, so seeing one also implies an operator is connected.

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
| `audio-status` | live-audio lifecycle (§9.2) | `state`, `sr`, `ch`, `reason` |
| `audio` | a chunk of the live Ogg/Vorbis audio (§9.3) | `seq`, `start`, `sr`, `ch`, `data` |

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
  `packets`, and the audio `seq`/`sr`/`ch` are integers; everything else numeric
  is a double.
- **Unknown `t` and unknown keys are fine.** Ignore event types you don't
  handle and keys you don't model — the format only grows.
- **`roster` is a string.** Its value is a JSON array; parse it separately if
  needed.
- **Strings are JSON-escaped** for `" \ \n \r \t`. Control characters that
  can't be represented decode lossily (to `?`) — don't rely on exotic bytes
  surviving. (The audio `data` field is base64, whose alphabet is never
  escaped, but decode it through your normal string path anyway.)
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
6. Optionally offer "listen live" (§9): write `audio-ctl` to stdin, decode the
   `audio` events, drive a play/stop button off `audio-status`.

---

## 9. Live audio + the inbound command channel

For the first time the client can also **write** to the producer — newline-JSON
commands on **stdin** — and the producer can stream the operator's demodulated
**receiver audio** back as Ogg/Vorbis, carried as base64 inside ordinary JSON
lines on the same stdout pipe. Everything here is opt-in: if you never write to
stdin, no audio is ever produced.

### 9.1 Where the audio comes from

```
  client (remote)                     simple_sat_ops --viewer-stream        simple_sat_ops --control
  ──────────────                      ──────────────────────────────        ───────────────────────
   reads stdout ◀── telemetry + base64 audio ── stdout                            (owns the one SDR;
   writes stdin ──▶ audio-ctl                ──▶ stdin                              has the demod PCM)
                       (over SSH)                  │  ▲
                                         unix sock │  │ unix sock
                                         audio-ctl ▼  │ audio / audio-status
                                                operator IPC socket
```

- **The operator sources the audio**, not `--viewer-stream`. The station has
  **one** SDR, owned by the `--control` operator; `--viewer-stream` still opens
  no hardware and relays.
- **Audio is only available in `operator` mode.** In `tle-only` mode there is no
  receiver, so an enable request is answered `"unavailable"`.
- **Each SSH connection is its own subscriber.** The operator runs a separate
  encoder per listening viewer, so every client gets a complete, self-contained
  Ogg stream that **begins with its own headers** — you never join mid-stream
  missing them.

### 9.2 Inbound command: `audio-ctl` (you → stdin)

Same framing as the outbound stream (§2): one flat JSON object per line, `\n`
terminated, no `ts` required. Unknown `t`/keys are ignored, so the channel can
grow. A command is acted on within one producer tick (≤ ~500 ms).

| key | JSON type | required | meaning |
|-----|-----------|----------|---------|
| `t` | string | yes | `"audio-ctl"` |
| `enable` | bool | yes | `true` to start audio, `false` to stop |
| `q` | number | no | VBR quality `0.0`–`1.0` (default `0.2`); honored on `enable:true`, clamped |

```json
{"t":"audio-ctl","enable":true}
{"t":"audio-ctl","enable":true,"q":0.4}
{"t":"audio-ctl","enable":false}
```

- `enable:true` in operator mode → you start receiving `audio` frames, preceded
  by `audio-status:"on"`. Idempotent — re-enabling restarts the stream (fresh
  `start:true` frame).
- `enable:true` in tle-only mode → `audio-status:"unavailable"`. **Not
  remembered**; re-send once `source` flips to `"operator"`.
- `enable:false` → `audio-status:"off"`, no more `audio` frames. Always safe.

### 9.3 Outbound: `audio-status` and `audio`

**`audio-status`** — sent on every state change (always in reply to `audio-ctl`;
also unsolicited when the operator stops sourcing audio or an error occurs):

| key | JSON type | present | meaning |
|-----|-----------|---------|---------|
| `t` | string | always | `"audio-status"` |
| `state` | string | always | `"on"` \| `"off"` \| `"unavailable"` \| `"error"` |
| `sr` | integer | with `"on"` | Stream sample rate, Hz |
| `ch` | integer | with `"on"` | Channel count (always `1` today) |
| `reason` | string | when not obvious | Detail for `"unavailable"`/`"error"` (`"no operator"`, `"audio disabled"`, `"no audio support in this build"`, `"operator gone"`, `"encoder init failed"`) |

`"unavailable"` is not an error — retry when an operator appears. On `"error"`
tear down your decoder; you may re-request.

**`audio`** — one chunk of the Ogg/Vorbis bitstream:

| key | JSON type | present | meaning |
|-----|-----------|---------|---------|
| `t` | string | always | `"audio"` |
| `seq` | integer | always | Per-stream frame counter from `0`, +1 each frame. Detects drops. |
| `start` | bool | when true | `true` **only** on the first frame of a fresh stream — it carries the Ogg/Vorbis headers. Reset your decoder on it. Absent (=false) afterwards. |
| `sr` | integer | with `start` | Sample-rate hint, Hz (authoritative value is in the Ogg headers) |
| `ch` | integer | with `start` | Channel-count hint |
| `data` | string | always | **base64** ([RFC 4648](https://www.rfc-editor.org/rfc/rfc4648), standard alphabet, `=` padding) of a contiguous slice of the Ogg bitstream |

Base64-decode `data` and **concatenate in `seq` order** → exactly the bytes
`ham_listen --ogg-stdout` would write; nothing is reframed. Frames arrive ≈ 4–10
per second, variable size, each ≤ ~4 KB raw.

### 9.4 Decoding

The stream is **Ogg-encapsulated Vorbis**, mono, VBR. Any standard Vorbis
decoder works (`libvorbisfile`, an `AudioToolbox`/`AVAudioFile` reader fed the
bytes, `ffmpeg`, …).

1. On `audio-status:"on"` (or the first `audio` frame with `start:true`), create
   a fresh decoder; discard any previous one.
2. For each `audio` frame, base64-decode `data` and feed the bytes to the
   decoder **in `seq` order**.
3. Play the PCM. The decoder learns the true rate/channels from the Ogg headers
   (in the `start` frame); `sr`/`ch` are just pre-allocation hints — **don't
   hardcode a rate**.
4. On `audio-status:"off"`/`"error"`, flush and tear down the decoder.

**Gaps & backpressure (important).** The audio path is **lossy under
backpressure**: if your SSH reader stalls, the producer **drops** audio frames
rather than blocking telemetry. You detect a drop as a **jump in `seq`**. On a
gap, keep feeding subsequent bytes — Ogg is page-framed with checksums, so the
Vorbis decoder **resyncs at the next page** (a brief glitch, surfaced as e.g.
`OV_HOLE`; keep going). You do **not** wait for a new `start`; a new `start`
only appears if the *operator* restarts its encoder. Drain stdin promptly so
*you* aren't the slow consumer.

**Handover / disconnect.** If the operator drops while you listen you get
`audio-status:"error"`,`reason:"operator gone"` and `source` flips to
`"tle-only"`. Tear down; re-send `audio-ctl enable:true` once an operator
returns to get a new `start:true` stream.

### 9.5 Bandwidth & latency

Mono Vorbis VBR at the default quality is ~30–50 kbit/s; base64 adds ~33%, so
plan for **~5–8 KB/s** per listening viewer (lower `q` for tight links).
End-to-end latency is a few hundred ms — this is a monitoring path, not a
low-latency one. Audio costs nothing until a viewer enables it.

### 9.6 Worked transcript

You write to stdin: `{"t":"audio-ctl","enable":true}`

Producer stdout (telemetry lines elided as `…`):

```json
{"t":"state","ts":"2026-06-22T19:00:01.002Z","source":"operator","sat":"FRONTIERSAT",…}
{"t":"audio-status","ts":"2026-06-22T19:00:01.210Z","state":"on","sr":96000,"ch":1}
{"t":"audio","ts":"2026-06-22T19:00:01.250Z","seq":0,"start":true,"sr":96000,"ch":1,"data":"T2dnUwACAAAAAAAAAAA…"}
{"t":"audio","ts":"2026-06-22T19:00:01.470Z","seq":1,"data":"AQAAAB1U…"}
{"t":"audio","ts":"2026-06-22T19:00:01.690Z","seq":2,"data":"k4t9f3…"}
…
```

You write: `{"t":"audio-ctl","enable":false}`

```json
{"t":"audio-status","ts":"2026-06-22T19:01:14.880Z","state":"off"}
```

(`T2dnUw==` decodes to `"OggS"` — the Ogg page-capture pattern — confirming the
first frame begins a real Ogg stream.)

---

## 10. Internal architecture (informative — not part of the consumer contract)

This documents the simple_sat_ops side so the two stay in sync; a consumer
implementer can skip it.

### 10.1 One audio frame's path

```
rx_session worker (operator)            operator main loop                  viewer-stream relay        client
  demod PCM ─▶ PCM ring  ───────▶  drain ring ─▶ per-subscriber        ─▶  SSO_EVT_AUDIO over    ─▶  (this contract)
                                    Ogg encoder ─▶ base64 ─▶ SSO_EVT_AUDIO     unix sock, re-encoded
                                    targeted-send to that subscriber id         to stdout
```

### 10.2 New IPC events (`src/ipc/sso_ipc.h`, `sso_ipc_codec.c`)

Three new `sso_event_type_t` values traverse **both** hops (operator↔relay,
relay↔client) with identical shape; the relay re-encodes them like it already
re-encodes `state`/`welcome`:

| enum | wire `t` | direction | fields (`sso_event_t` → wire) |
|------|----------|-----------|-------------------------------|
| `SSO_EVT_AUDIO_CTL` | `audio-ctl` | client → relay (stdin) → operator (IPC) | `audio_enable`→`enable`, `audio_quality`→`q` |
| `SSO_EVT_AUDIO_STATUS` | `audio-status` | operator → relay → client | `audio_state`→`state`, `audio_sr`→`sr`, `audio_ch`→`ch`, `reason` (reused) |
| `SSO_EVT_AUDIO` | `audio` | operator → relay → client | `audio_seq`→`seq`, `audio_start`→`start`, `audio_sr`/`audio_ch`, `audio_b64`→`data` |

`SSO_AUDIO_RAW_MAX = 4096` (raw Ogg bytes/frame) bounds the base64 field
(`SSO_AUDIO_B64_MAX`) so a full `audio` line stays well under `SSO_IPC_LINE_MAX`
(8000). The operator **targeted-sends** (`sso_ipc_server_send`) audio only to
subscribed client ids; it never broadcasts it.

### 10.3 Shared Ogg/Vorbis module (the de-duplication)

The Ogg/Vorbis sink that lived inline in `utils/ham_listen.c` (the
`ogg_sink_t` struct, the libsndfile virtual-IO callbacks, and the encoder
setup) is factored into `src/audio/ogg_stream.{c,h}`, used by **both**
`ham_listen` and the operator's audio pump:

```c
// HAVE_SNDFILE gates the whole module. The sink callback receives encoded
// Ogg bytes, so the same encoder serves a pipe, a socket, or a base64 framer.
// It mirrors write(2): return bytes consumed, or <0 to latch an error.
typedef long (*ogg_stream_sink_fn)(const uint8_t *bytes, size_t n, void *user);

ogg_stream_t *ogg_stream_open(int sample_rate, int channels,
                              double vbr_quality,
                              ogg_stream_sink_fn sink, void *user);
int  ogg_stream_write(ogg_stream_t *s, const int16_t *pcm, size_t frames);
unsigned long long ogg_stream_bytes(const ogg_stream_t *s);
void ogg_stream_close(ogg_stream_t *s);   // flushes
```

`ham_listen` keeps its CW/BFO demod, squelch, de-emphasis, and CLI — those are
tool-local, not streaming concerns. Its sink writes to `STDOUT_FILENO`; the
operator's sink base64-encodes into an `SSO_EVT_AUDIO` and targeted-sends it.

### 10.4 Operator & relay plumbing

- `rx_session` gains a small lock-protected **PCM tap ring** the worker fills
  after each pump (alongside the WAV append) and `rx_session_read_audio()` the
  main loop drains. No subscriber → no cost.
- The operator's `ipc_on_event` (`src/control/operator_ipc.c`) handles
  `SSO_EVT_AUDIO_CTL`: ref-counts subscribers, creates/destroys that client's
  `ogg_stream_t`, replies `SSO_EVT_AUDIO_STATUS`. A disconnecting subscriber is
  dropped (encoder freed) on the next server step. Subscriber count is capped
  (`SSO_AUDIO_MAX_SUBS`) to bound CPU on the single-threaded operator.
- The operator main loop's **audio pump**: when ≥1 subscriber, drain the PCM
  ring, feed each subscriber's encoder, base64 each emitted chunk into an
  `SSO_EVT_AUDIO`, targeted-send.
- `run_viewer_stream` (`src/ui/viewer.c`) adds non-blocking **stdin** reads in
  both loops. A parsed `audio-ctl` is forwarded to the operator via
  `sso_ipc_client_send` in operator mode, or answered locally with
  `audio-status:"unavailable"` in tle-only mode. `stream_relay_on_event`
  forwards `audio`/`audio-status` downstream automatically once the codec knows
  the new types (no `source` re-tag — they are not `state`).
