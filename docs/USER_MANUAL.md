---
pagetitle: "The Sat Ops Guide"
---

# The CalgaryToSpace Guide to Cubesat Operations

*A field guide to pointing antennas, pulling frames out of the noise,
and talking to a satellite that only answers when you ask politely.*

Version: 3 (working draft)

Applies to `simple_sat_ops` and friends on `main`, commit
`7f0990d` (2026-06-25). This is a working draft.

Prepared by Johnathan K. Burchill and Claude Opus 4.8 at the University
of Calgary.

A tool-by-tool reference for the Calgary-to-Space FrontierSat ground
station software (`simple_sat_ops` and friends), written for the team
members who run passes and the developers who extend the code.

---

## Foreword

Operating a satellite pass combines three pleasures: aiming a few
kilograms of aluminum at a fast-moving point in the sky, coaxing a
coherent frame out of a hiss of thermal noise, and the small thrill of
sending a command that the spacecraft actually answers.

The most important thing to understand is that *running a pass* is not
hard. A pass lasts a few minutes. The hardware is forgiving. The
software refuses, by default, to do anything that could hurt the radio,
the rotator, or the link: the transmitter starts inhibited, the rotator
moves only when you tell it to, and only one operator can touch the
hardware at a time. A beginner sitting in viewer mode cannot break
anything, no matter which keys they lean on, and an operator flying a
*vetted* telecommand file - one a qualified person has already reviewed -
will have to try hard to do any harm. That is exactly the job
`simple_sat_ops` is built for, and the skill it asks for is almost all
understanding and only a little practice.

There is one real exception, and it belongs up front: the software
protects the *ground station*, not the *spacecraft*. It cannot tell a
sensible command sequence from a structurally valid but operationally
disastrous one. *Composing* a telecommand file - choosing which commands
go out, in what order, with which arguments - is an
intermediate-to-advanced activity, and a poorly chosen sequence can
brick the satellite for good. So the rule that keeps newcomers safe is a
simple one: fly files that other people wrote and vetted, and learn to
write your own only once you understand the whole chain. The manual is
honest about how a brick happens where it belongs (see [Populating a
`--tc-file`](#populating-a---tc-file-for-auto-commanding)).

Every part of the ground station lies to you in its own small,
well-understood way, and the job is mostly knowing how. The SPID rotator will
cheerfully report that it has reached a new position the instant you
ask it to move, long before the motor has turned. The B210 will leak
its local oscillator straight out the antenna if you let the transmit
chain idle hot. The satellite answers only the frames that carry the
right signature and drops the rest without a word, so a wrong key
looks exactly like a dead pass. None of these is a defect you fix;
each is a quirk you learn, plan around, and eventually stop thinking
about.

One more thing before the controls: this is a team activity, and the
software is built around that fact. One person runs the pass while
others watch the same screen in viewer mode, calling out a drifting
azimuth, a missed frame, a Doppler curve that looks wrong. Passes are
short and a lot happens at once; a second pair of eyes catches what
the operator, head down in the panels, will miss. If you ever find
yourself running a pass entirely alone, at night, talking to a
spacecraft that only answers when you ask it politely - pause and
reconsider some of your life choices. Then call a teammate.

And one standing request while you read. This manual was written
alongside the code, and the code keeps moving - a flag gets renamed, a
path moves, a default changes - so some of what follows is already
wrong by the time it reaches you. Treat that as part of the job: as you
read, your one task is to catch the errors and report the corrections
as you find them. When the manual and the code disagree, the code
wins (see [How to read this manual](#how-to-read-this-manual)); when
you win an argument with the manual, fix the manual. A guide is only as
honest as its last careful reader left it.

Read the theory before you touch the keys. Then run a pass as a
viewer, where nothing you do reaches the hardware. Then take the
controls. By the third pass the sequence will feel obvious, and this
manual can go back on the shelf where it belongs.

---

## Table of Contents

- [Foreword](#foreword)
- [How to read this manual](#how-to-read-this-manual)

1. [Overview](#overview)
2. [How the downlink works: from photon to plaintext](#how-the-downlink-works-from-photon-to-plaintext)
   - [From radio wave to numbers](#from-radio-wave-to-numbers)
   - [Chasing the Doppler](#chasing-the-doppler)
   - [What's on the wire: MSK](#whats-on-the-wire-msk)
   - [Turning frequency into bits](#turning-frequency-into-bits)
   - [Finding the frame: the sync marker](#finding-the-frame-the-sync-marker)
   - [The length, well protected: Golay(24,12)](#the-length-well-protected-golay2412)
   - [Undoing the scrambler: CCSDS](#undoing-the-scrambler-ccsds)
   - [Correcting the errors: Reed-Solomon(255,223)](#correcting-the-errors-reed-solomon255223)
   - [Reading the packet: CSP](#reading-the-packet-csp)
   - [Watching it happen](#watching-it-happen)
   - [The uplink, in reverse](#the-uplink-in-reverse)
   - [When the chain breaks](#when-the-chain-breaks)
3. [Hardware](#hardware)
4. [Build and install](#build-and-install)
5. [First-run setup](#first-run-setup)
6. [Before you operate: licensing and authorization](#before-you-operate-licensing-and-authorization)
7. [A map of the cat: the tools](#a-map-of-the-cat-the-tools)
8. [Operator UI: `simple_sat_ops`](#operator-ui-simple_sat_ops)
   - [Modes: operator vs. viewer](#modes-operator-vs-viewer)
   - [Command-line options](#command-line-options)
   - [SDR backends](#sdr-backends)
   - [Keyboard controls](#keyboard-controls)
   - [Status, RX, and TX panels](#status-rx-and-tx-panels)
   - [Colon-command prompt](#colon-command-prompt)
   - [TX compose modal (`t`)](#tx-compose-modal-t)
   - [Auto-telecommand modal (`A`)](#auto-telecommand-modal-a)
   - [simple_sat_ops-directed commands (`SSO+`)](#simple_sat_ops-directed-commands-sso)
   - [Finding telecommands and their arguments](#finding-telecommands-and-their-arguments)
   - [Rotator calibration (`--calibrate-rotator`)](#rotator-calibration---calibrate-rotator)
   - [Pursuit tracking](#pursuit-tracking)
9. [Pass scheduling: `next_in_queue`](#pass-scheduling-next_in_queue)
10. [Agenda review: `agenda_check`](#agenda-review-agenda_check)
    - [Telecommand linting](#telecommand-linting)
11. [Offline analysis tools](#offline-analysis-tools)
    - [`gen_waterfall`](#gen_waterfall)
    - [`rx_replay`](#rx_replay)
    - [`decode_inspector`](#decode_inspector)
    - [Replaying SatNOGS recordings](#replaying-satnogs-recordings)
    - [`beacon_detect`](#beacon_detect)
    - [`fm_preview`](#fm_preview)
    - [`packet_query` and `packet_browser`](#packet_query-and-packet_browser)
    - [`tcmd_import`](#tcmd_import)
    - [`tcmd_browser`](#tcmd_browser)
    - [`tle_keps`](#tle_keps)
    - [`gnss_reports`](#gnss_reports)
12. [Uploading the orbit to the space safety database](#uploading-the-orbit-to-the-space-safety-database)
    - [The workflow](#the-workflow)
    - [When a fix is good enough to upload](#when-a-fix-is-good-enough-to-upload)
13. [Bring-up and test tools](#bring-up-and-test-tools)
    - [`tx_frame_sdr`](#tx_frame_sdr)
    - [`b210_rx_capture` and `b210_gain_sweep`](#b210_rx_capture-and-b210_gain_sweep)
    - [`live_waterfall`](#live_waterfall)
    - [`uplink_test`](#uplink_test)
    - [`rx_decode`](#rx_decode)
    - [`lifetime`](#lifetime)
    - [Amateur-band voice: `ham_listen` and `ham_speak`](#amateur-band-voice-ham_listen-and-ham_speak)
14. [Testing and validation](#testing-and-validation)
    - [How the code is validated: four layers](#how-the-code-is-validated-four-layers)
    - [Running the unit tests](#running-the-unit-tests)
    - [Catching what the compiler hides](#catching-what-the-compiler-hides)
    - [Decode smoketest: known-good recordings](#decode-smoketest-known-good-recordings)
    - [Validation that runs on every pass](#validation-that-runs-on-every-pass)
    - [Every bug leaves a test behind](#every-bug-leaves-a-test-behind)
    - [What isn't covered](#what-isnt-covered)
15. [Architecture notes](#architecture-notes)
    - [IPC: one operator at a time](#ipc-one-operator-at-a-time)
    - [Worker threads](#worker-threads)
    - [TX safety gates](#tx-safety-gates)
    - [Pursuit planner internals](#pursuit-planner-internals)
16. [File layout](#file-layout)
17. [Troubleshooting](#troubleshooting)
18. [A note on feel](#a-note-on-feel)
19. [Glossary](#glossary)

*Appendices:*

- [Appendix A: What didn't stick - the FT-991A and IC-9700 radio path](#appendix-a-what-didnt-stick---the-ft-991a-and-ic-9700-radio-path)
- [Appendix B: Trial and error - the git repository](#appendix-b-trial-and-error---the-git-repository)
- [Appendix C: The test-suite audit and continuous integration](#appendix-c-the-test-suite-audit-and-continuous-integration)

---

## How to read this manual

The chapters run theory first, hardware in the middle, tools last, so
you can read straight through or jump to the tool you need. Each tool
section says what the thing is for before it lists the flags, on the
theory that one flag you understand is worth ten you have memorized.

There is a Feynman story behind that. As a student he was handed a
paper on nerve impulses in cats, thick with the names of muscles - the
extensors, the flexors, the gastrocnemius - and he had no idea where
any of them sat on the animal. So he asked the biology librarian for
"a map of the cat" and got laughed at: the term of art was "zoological
chart," and for a while he was the dumb student who wanted a map of a
cat. But when his turn came to give the talk he drew the outline and
named the muscles off it anyway, and when the other students cut in
with "we know all that," he realized they had spent four years
memorizing what he had just looked up in fifteen minutes.

That is how this manual is meant to be used. You do not need to carry
the flags, the menu numbers, or the frame format around in your head -
you need to know they exist and where to find them when the moment
comes. Read it once for the shape of the thing; after that it is a
reference you look things up in, not a text you memorize.

A few conventions:

| You see | It means |
|---------|----------|
| `monospace` | A literal command, file path, flag, or key to press. |
| `$x^\circ$` | Mathematics, set with LaTeX (degrees, deltas, and the like). This manual uses no special characters outside math, so it survives copy-paste into a wiki, a terminal, or a plain-text email intact. |
| **Drill** | A hands-on exercise. Do each one on real hardware at least once; they build the feel the prose cannot. |
| **Safety** | A point where the software is protecting you from yourself. Read these even if you skip everything else. |

When the manual and the code disagree, the code wins. `--help-full`
and the source files named in each section are authoritative; a manual
is only a snapshot of how things stood when it was written, and
hardware and firmware drift.

---

## Overview

The ground station drives a USRP B210 software-defined radio and a
SPID Rot2ProG antenna rotator to receive telemetry from and command
CalgaryToSpace's CTS-SAT-1 (FrontierSat). Uplink and downlink share
one carrier at **436.150 MHz** (simplex). Downlink is **FSK 9600
baud** with the AX100 frame format: 32-bit ASM, Golay-coded length,
CCSDS scrambling, Reed-Solomon FEC. Every uplink frame carries an
HMAC. The satellite drops unsigned frames silently, so a missing
key is fatal to the link.

A word on why a software-defined radio sits at the centre of this. When
the team came together, few of us held the advanced amateur
qualification needed to put hardware of our own design on the air, and
none of us wanted the project's schedule to depend on earning it. So
the guiding rule was to lean on commercial, off-the-shelf parts
wherever we could: a stock SDR, a stock rotator, stock transceivers
before them. The B210 is COTS in exactly that sense - a finished,
type-approved radio we drive with software rather than a circuit we
built. The advanced work lives in the code, where a mistake costs a
rebuild and not a violation.

There is one **operator UI** binary (`simple_sat_ops`) that owns the
hardware during a pass. Around it sits a set of CLI tools for pass
scheduling, calibration, command-list review, and offline analysis
of recorded captures. The operator UI runs a cooperative
single-threaded ncurses loop but pushes serial I/O, SDR I/O, and
audit-log writes onto background pthreads so device latency never
freezes the screen.

Only **one** operator process can hold the hardware at a time.
Other invocations are *viewers*: they connect to the operator's IPC
socket and render the same state read-only. A viewer can force-claim
the operator role; the running operator yields and the viewer
re-execs itself as the new operator. A third mode, `--viewer-stream`,
is a headless JSON feed for a remote viewer: it streams the same data
to stdout, works even with no operator running (propagating the TLE
itself), and never touches the hardware.

## How the downlink works: from photon to plaintext

This is the "how it works" chapter. You can run a pass without reading
it, the way you can drive a car without knowing what the engine does. But
knowing what each stage does is exactly what lets you tell a weak pass
from a broken one, and read the panels instead of just watching them.
Skip the equations on a first pass if you like; the prose carries the
argument and the math is there for when you want it.

Decoding is the satellite's encoding run backwards. To get a few
fragile bytes of telemetry across a few thousand kilometres of noisy
vacuum with a Doppler-smeared carrier, the spacecraft wraps them in
layer after layer of protection. The receiver peels those layers off
in the reverse order. Every arrow below is one small algorithm, and
each was forced on us either by the channel (noise, Doppler, an
unknown symbol clock) or by an upstream encoder that traded raw data
rate for the ability to survive the trip:

```
RF wave at ~436 MHz
  |  antenna, analog frontend, ADC, I/Q mixer
  v
complex baseband samples (96 kHz)
  |  FM discriminator
  v
instantaneous-frequency stream
  |  matched filter
  v
soft symbol estimates
  |  timing recovery + slicer
  v
raw bits
  |  sync-word (ASM) search
  v
framed bits
  |  Golay(24,12) length decode
  v
known-length bytes
  |  CCSDS descrambler
  v
de-whitened bytes
  |  Reed-Solomon(255,223) decode
  v
error-corrected payload
  |  CSP packet parse
  v
header fields + application bytes ("Hello from CalgaryToSpace FrontierSat")
```

![The receive chain, end to end](figures/rx-signal-chain.png)

The rest of this chapter walks down that chain. Everything here runs
live inside `simple_sat_ops` and offline in `rx_replay`; you can watch
it happen stage by stage in `decode_inspector` (see
[Watching it happen](#watching-it-happen)).

### From radio wave to numbers

The satellite transmits a passband signal centred at
$f_c \approx 436.150$ MHz. The B210 tunes to $f_c$ and uses an analog
quadrature mixer to multiply the antenna voltage against both a cosine
and a sine at $f_c$, low-pass filtering each. Pack the two outputs into
one complex number $s(t) = I(t) + i\,Q(t)$, and a sinusoid at
$f_c + \Delta f$ becomes a complex exponential
$e^{\,i\,2\pi\,\Delta f\,t}$ at baseband: a positive $\Delta f$ sits
above the tuner, a negative one below. This is a lock-in amplifier with
two phases.

The B210 hands the host complex IQ at 480 kHz; a software FIR in
`src/dsp/fir_decim.c` decimates by 5 to 96 kHz. At 9600 baud that is
**10 samples per symbol**, comfortable headroom for everything below.

### Chasing the Doppler

The satellite closes on the line of sight at up to
$v_r \approx 7.6$ km/s, shifting the carrier by

$$
\Delta f_{\mathrm{dop}} \approx \pm \frac{v_r}{c}\, f_c
  \approx \pm 10\ \mathrm{kHz}\ \text{at 436 MHz}.
$$

Left alone that would walk the signal clean out of the matched
filter's band by mid-pass. Instead the receiver recomputes the
expected Doppler from the SGP4 propagator every tick and retunes the
B210 whenever the prediction has drifted more than 200 Hz from the
current tune. Near closest approach, where the Doppler rate peaks
around 50 Hz/s, retunes fire every few seconds; far from it, rarely.

The chain downstream is built so a retune is a non-event: the FM
discriminator (below) is differential, so the absolute tuner phase
that jumps at each retune cancels; the residual offset between retunes
is a small DC pedestal that the high-pass filter flattens in about a
millisecond; and a frame lasts about 50 ms while retunes are seconds
apart, so even a frame that straddles a retune loses at most one
sample, well inside the error budget. The rest of this chapter assumes
the tuner stays within $\pm 200$ Hz of the real carrier.

![A pass on the waterfall, Doppler curving the carrier](figures/pass-waterfall-doppler.png)

### What's on the wire: MSK

FrontierSat uses **binary frequency-shift keying**: a `1` is a tone at
$+2400$ Hz relative to the carrier for one symbol, a `0` is a tone at
$-2400$ Hz, at 9600 baud. The dimensionless modulation index is

$$
h = \frac{2 \cdot \mathrm{deviation}}{\mathrm{baud}}
  = \frac{2 \cdot 2400}{9600} = 0.5 .
$$

An index of $h = 0.5$ is **MSK**, minimum-shift keying: the narrowest
FSK that still keeps the two tones orthogonal over a symbol while
holding the phase continuous across symbol boundaries, so it does not
splatter energy into the neighbours. It is the standard setting for the
GomSpace AX100 modem the satellite carries.

Why frequency keying at all? Because the bit lives in the frequency,
not the amplitude, the link shrugs off fading and lets the spacecraft's
power amplifier run hard into saturation without corrupting data. The
price is bandwidth: MSK is wider than an equivalent phase scheme.

### Turning frequency into bits

Four small steps take the complex baseband stream to a clean run of
bits.

**The FM discriminator** turns frequency into a voltage. A clean sample
is $s[n] = A\,e^{\,i\,\phi[n]}$, and the instantaneous frequency is the
phase step, which we read off as

$$
\Delta\phi[n] = \arg\!\bigl(s[n]\,s^{*}[n-1]\bigr).
$$

The product $s[n]\,s^{*}[n-1]$ cancels the absolute phase, so we never
need to know the carrier phase and slow Doppler becomes a constant DC
offset rather than a moving sinusoid. For FrontierSat the output hops
between about $+0.157$ and $-0.157$ rad/sample. Two wrinkles: when
noise dominates, `atan2` wraps and throws a huge spurious spike (the
classic FM "click"), so we low-pass the IQ at 12 kHz first; and the
residual carrier offset puts a DC pedestal on the output that a
one-pole high-pass filter removes.

**The matched filter** averages out the noise. The transmitter holds
each frequency for a full symbol, so the pulse is a rectangle, and the
filter that maximises signal-to-noise at the decision instant is itself
a rectangle: a running mean over the $\mathrm{sps} = 10$ samples of a
symbol. That cuts the noise by $\sqrt{10} \approx 3.2$ while leaving the
signal intact.

**Timing recovery** finds where each symbol actually falls. We have
ten samples per symbol but do not know the phase offset, and the two
clocks drift, so the right sampling instant moves. A Gardner timing
detector, fed fractional-sample values from a Farrow interpolator,
forms an error term

$$
\operatorname{ted}[k] = y\!\left[k-\tfrac{1}{2}\right]\cdot\bigl(y[k]-y[k-1]\bigr),
$$

which is zero at correct timing and changes sign with the direction of
the error; a small loop gain nudges the sampling instant to drive it to
zero.

**The slicer** makes the call: the bit is $1$ when the matched-filter
output is positive, $0$ otherwise. That throws away the soft magnitude,
which a soft-decision decoder could spend to claw back roughly 2 dB,
but the Reed-Solomon layer downstream is strong enough that we do not
need to.

All four live in `src/dsp/modem_fsk.c`.

### Finding the frame: the sync marker

The bit stream is continuous; we have to find where each frame begins.
The transmitter prepends a fixed 32-bit **Attached Sync Marker** to
every frame:

```
ASM = 0x930B51DE
```

We slide a 32-bit window along the bits and count the Hamming distance
to the ASM, accepting any position within 4 bits as a match. The
marker is the CCSDS standard value, chosen for low autocorrelation so a
partial match never masquerades as a full one. With a 4-bit tolerance
the chance a random 32-bit run matches is

$$
P_{\mathrm{fa}} = \frac{1}{2^{32}} \sum_{k=0}^{4} \binom{32}{k}
  \approx 10^{-5},
$$

about one false alarm per pass, which the length, Golay, and RS layers
below reject cheaply.

### The length, well protected: Golay(24,12)

The 24 bits right after the ASM carry the **frame length** in a
Golay(24,12) block code: 12 data bits plus 12 parity, correcting up to
3 bit errors. The length gets the heaviest per-bit protection in the
whole frame for a simple reason: a wrong length offsets the byte
boundary for the entire rest of the frame, and possibly the next one
too. It is the single most expensive field to lose. Lives in
`src/proto/golay24.c`.

### Undoing the scrambler: CCSDS

The payload bytes were XOR'd at the transmitter against a
pseudo-random sequence from an 8-bit shift register seeded to all-ones,
with polynomial

$$
h(x) = x^{8} + x^{7} + x^{5} + x^{3} + 1 ,
$$

the additive scrambler from CCSDS 131.0-B. Descrambling is the same XOR
again. The point is not secrecy (anyone with the standard can undo it)
but engineering hygiene: it guarantees a healthy density of bit
transitions no matter how repetitive the payload, which keeps the
spectrum flat and the timing loop locked. A long run of identical bytes
would otherwise hold the discriminator flat and let the clock drift.

### Correcting the errors: Reed-Solomon(255,223)

The de-whitened payload is a Reed-Solomon codeword: 223 data bytes plus
32 parity bytes, 255 in all, over the finite field
$\mathrm{GF}(2^{8})$ where each byte is one field element. It corrects

$$
t = \left\lfloor \frac{255 - 223}{2} \right\rfloor = 16
$$

byte errors anywhere in the block, about 6% of the codeword. This is
the workhorse: at our usual link margin the slicer makes a handful of
bit errors per frame and RS quietly fixes them.

The interesting part is the failure: the threshold is a cliff, not a
slope. Assuming independent bit errors at rate $p_b$,

$$
P_{\mathrm{ok}}(p_b)
  = \sum_{k=0}^{16} \binom{255}{k}\, p_B^{\,k}\,(1 - p_B)^{\,255-k},
  \qquad p_B = 1 - (1 - p_b)^{8} .
$$

Below about $p_b = 4\times 10^{-3}$ the decoder is essentially
infallible; by $2\times 10^{-2}$ it is hopeless; the transition sits
where the expected byte-error count crosses 16. A frame that fails RS
is rescued only by the partial-decode path in
`src/pipeline/decode_loop.c`, which prints whatever ASCII survived
uncorrected. Lives in `src/proto/rs.c` and `src/proto/ax100.c`.

### Reading the packet: CSP

What survives RS is a **CSP** packet, the CubeSat Space Protocol, in
spirit a stripped-down IP for small satellites. A v1 header is 4 bytes:
priority, source and destination node addresses, source and
destination ports, and a few flags. After it come the application
bytes, an ASCII string for a beacon or a packed binary struct for
telemetry, in the format the flight software defines. The parse lives
in `src/proto/csp.c`; the result is written to the SQLite packet
database (`src/db/packet_db.c`) and broadcast over the IPC bus to any
attached viewers.

### The last check: CSP CRC-32C

RS corrects the bytes; the CRC tells you whether what came out is
exactly what the satellite sent. The FrontierSat downlink runs CSP in
**CRC mode**: after the CSP header and application bytes it appends a
four-byte **CRC-32C (Castagnoli)** computed over the whole packet
(header included) and written big-endian. This is *not* the zlib /
IEEE 802.3 CRC-32 that the name "CRC32" often implies - the satellite's
libcsp and the ground reference both use the Castagnoli polynomial
(`0x1EDC6F41`), and computing the wrong one makes every clean frame
look corrupt. The check lives next to the parse in `src/proto/csp.c`
(`csp_crc32c`).

The receiver validates the trailer by default (`--no-csp-crc32` on
`rx_replay` / `rx_decode` turns it off). The outcome lands in the
database's `crc_status` column and drives how a packet is shown:

| `crc_status` | Meaning | What happens to the frame |
|----|----|----|
| `1` | CRC matched | the four trailer bytes are stripped; the packet is stored clean |
| `0` | CRC mismatched | the trailer is **kept** in the payload and the frame is still stored, so weak or partly corrupted telemetry stays visible rather than vanishing - but `packet_browser` flags it as an error |
| `-1` | not checked | validation was disabled, or the frame was too short to carry a trailer |

The deliberate choice is never to drop a frame on a bad CRC: a marginal
pass where RS only just held is exactly when you most want to see what
got through, errors and all. The CRC is advisory metadata, not a gate.

### Watching it happen

`decode_inspector` renders this exact chain, stage by stage, on a live
or replayed capture: the ASM correlation dropping to zero at the frame
start, the Golay length decode, the descrambler, the Reed-Solomon
block, and the final CSP parse. It is the best way to build intuition
for the theory above, because you watch a real frame walk through every
step and see which stage a marginal pass dies at.

![decode_inspector walking a frame through the chain](figures/decode-inspector-stages.png)

Where each stage runs in the live receiver:

| Stage | File(s) |
|-------|---------|
| RF capture, downconversion, decimation | `src/hw/b210_rx_tx_core.c`, `src/dsp/fir_decim.c` |
| LPF, FM discriminator, matched filter, timing recovery, slicer, ASM | `src/dsp/modem_fsk.c` |
| MSK Viterbi fallback slicer | `src/dsp/modem_viterbi.c` |
| Length decode, descramble, Reed-Solomon | `src/pipeline/decode_loop.c`, `src/proto/golay24.c`, `src/proto/ax100.c`, `src/proto/rs.c` |
| CSP parse | `src/proto/csp.c` |
| Session glue, WAV capture, dedup, database | `src/pipeline/rx_session.c`, `src/db/packet_db.c` |
| Operator UI and IPC | `apps/main.c` |

For offline work `rx_replay` runs the same chain in two passes: a fast
sync search sweeps the whole capture and collects ASM candidate
timestamps, then the heavier decoder runs on tight windows anchored at
each candidate, which is far quicker than slicing the whole file at
full cost.

### The uplink, in reverse

Transmitting is the same film run backwards. The operator's payload
goes into a CSP header, gets signed with the HMAC the satellite
demands, then runs through Reed-Solomon, the CCSDS scrambler, the Golay
length header, and the ASM, and is MSK-modulated onto the carrier. The
one asymmetry is Doppler: where the receiver chases the incoming shift,
the transmitter must *pre-compensate* the outgoing one, keying at
$f_{\mathrm{carrier}} / (1 - v_r/c)$ so the signal lands on the
satellite's receiver at the right frequency. The mechanics of composing
and sending a burst are in the
[TX compose modal](#tx-compose-modal-t).

### When the chain breaks

The failure modes map onto the stages above, and the operational
symptoms and fixes are collected in [Troubleshooting](#troubleshooting):

* **Doppler tracking lost** - the SGP4 prediction is more than about
  10 kHz off the real carrier, almost always a stale or wrong TLE. The
  signal slides out of the band and the chain never locks. Refresh the
  TLE.
* **Symbol-clock drift beyond about 100 ppm** - the timing loop cannot
  hold; rare with healthy hardware.
* **Modulation-index mismatch** - if the signal is not quite $h = 0.5$,
  the boxcar matched filter is suboptimal and costs around 1 dB. The
  `src/dsp/modem_viterbi.c` slicer recovers most of it on a true MSK
  signal and is the fallback when the discriminator path cannot close.
* **Passes sync and descramble but fail RS** - the link is a few dB
  below the cliff. Nothing to do in software short of soft-decision
  decoding; this is a signal-margin problem, not a decode bug.

## Hardware

| Component | Notes |
|-----------|-------|
| **USRP B210** | Two-channel SDR; channel 0 receives on `RX2` and transmits on `TX/RX`. UHD must be installed. Receive runs continuously, including during a transmit burst. |
| **SPID Rot2ProG** | Az/El antenna rotator with a built-in controller. USB-serial CAT at any baud (it autodetects). Constant slew rates configured on the controller's front panel. |
| **UHF T/R switch** | Optional CalgaryToSpace RP2040-Zero board on USB-CDC at `/dev/ttyACM0`. Senses RF on the TX line and flips two relays (K1 = antenna path, K2 = dummy). Auto-mode by default; the firmware emits a heartbeat the operator UI parses for a status panel. |

The live RF path is the B210. A conventional transceiver (Yaesu
FT-991A, Icom IC-9700) drove that path in an earlier design and the
code to run one is still in the tree, but it is no longer the way we
operate. That history, and the tools that go with it, lives in
[Appendix A](#appendix-a-what-didnt-stick---the-ft-991a-and-ic-9700-radio-path).

## Build and install

CMake, out-of-tree, installs to `$HOME/bin`:

```sh
mkdir -p build && cd build && cmake .. && make install
```

Which targets actually build depends on what the host has:

| Dependency | Targets it unlocks |
|------------|---------------------|
| always | `radio_ctl`, `rs_selftest`, `fm_preview`, `agenda_check` |
| OpenSSL / libcrypto | `uplink_test`, `rx_decode`, `packet_query`, `packet_browser`, `tcmd_browser`, `tcmd_import` |
| SGP4SDP4 | `next_in_queue`, `lifetime`, `tle_keps`, `prediction_selftest`, `pursuit_selftest` |
| UHD (B210) | `b210_rx_capture`, `b210_gain_sweep`, `tx_frame_sdr`, `sdr_probe` |
| librtlsdr | RTL-SDR RX-only backend in `simple_sat_ops` (on by default; auto-disables if absent) |
| libusb | USB-serial clone detection in the UHD backend (a UHD dependency, so normally already present) |
| libsndfile | `rx_replay` reading SatNOGS `.ogg` audio recordings |
| raylib | `live_waterfall`, `decode_inspector` |
| ncurses + SGP4SDP4 + libcrypto | `simple_sat_ops` (pulls UHD too when present) |

### Dependencies

The build auto-detects each library via pkg-config; install the **`-dev`**
packages so the headers and `.pc` files are found.

Ubuntu/Debian (the ground machine):

```sh
sudo apt install build-essential cmake pkg-config \
    libncurses-dev libssl-dev libsqlite3-dev libasound2-dev \
    libuhd-dev librtlsdr-dev libusb-1.0-0-dev libsndfile1-dev
```

macOS (Homebrew, dev host):

```sh
brew install cmake pkg-config ncurses openssl sqlite uhd librtlsdr libusb libsndfile
```

Mind the Debian names: it is **`librtlsdr-dev`** and **`libusb-1.0-0-dev`**
- the bare `librtlsdr` / `libusb` / `libusb-1.0` packages do not exist
(`apt` says "Unable to locate package"). UHD is `libuhd-dev`; libsndfile
(lets `rx_replay` read SatNOGS `.ogg`) is `libsndfile1-dev`; `raylib`
(optional `live_waterfall`) is `libraylib-dev`.

The bundled `sgp4sdp4/` directory is a separate CMake project; build
and install it first (see `sgp4sdp4/README.md`). `CMakeLists.txt`
hard-codes the install location, so a non-default prefix needs a
small edit.

There is no project-wide test suite, linter, or formatter.
`scripts/lint_warnings.sh` runs a parallel gcc-15 build of the whole
tree under `-Wformat-truncation=2 -Werror=format-truncation` to catch
warnings Apple clang misses. Output lives in `build-lint/`.

## First-run setup

1. **Confirm hardware.** Run `sdr_probe` (see
   [SDR backends](#sdr-backends)): it shows the SDR, the FPGA image it
   will load, and the RX/TX antenna ports - and, for a clone, surfaces
   the USB serial even when a wrong-FPGA open would fail. SPID powered
   and in `A` mode on the front panel (the firmware path that responds
   to the W-frame STATUS query). T/R switch USB-CDC visible if
   installed. Antenna connected to the right port.

2. **TLE file.** By convention the team keeps dated TLEs under the
   FrontierSat TLE directory (referred to below as `$TLES`, i.e.
   `$FRONTIERSAT_ROOT/TLEs`), one per day in a date-stamped subfolder:
   `$TLES/20260529/tle-20260529.tle`. `simple_sat_ops --control` with
   no `<satellite_id>` loads the newest `*.tle` it finds there
   (searched recursively) and pins it into the pass folder. Otherwise
   pass `--tle <path>` explicitly, or stage one at the default
   `$HOME/.local/state/simple_sat_ops/active.tle`, which `next_in_queue`
   shares.

   On the ground machine these dated files are produced automatically.
   `scripts/fetch_tle.sh` runs once a day from cron (`/etc/cron.d/sso-tle`,
   09:00), pulls the latest TLE for FrontierSat (NORAD **69015**) from
   CelesTrak, and writes `$TLES/<date>/tle-<date>.tle` - exactly the layout
   the newest-dated auto-load expects, so each morning's fresh elements are
   picked up with no manual step. It rewrites the name line to
   `FrontierSat` (CelesTrak returns upper-case `FRONTIERSAT`, which the
   case-sensitive name match would miss) and, via the setgid
   `root:sso-ops` TLE directory plus `umask 0002`, leaves the file group
   `sso-ops` and readable by every controller account. A failed or
   malformed download never overwrites a good file. Run it by hand any
   time to refresh the current day (`fetch_tle.sh`), and see the script
   header for the cron line and the `CATNR` / `SAT_NAME` / `USE_UTC`
   knobs. It installs to `/usr/local/bin` with the other tools via
   `sudo make install`.

3. **HMAC key.** Required for any on-air uplink (it signs every
   transmitted frame; the downlink is checked separately by its CSP
   CRC32, so a key problem never affects receiving). The operator UI
   loads the shared keyfile (`/FrontierSat/HMAC/frontiersat_hmac`),
   failing that `$HOME/.local/state/simple_sat_ops/frontiersat_hmac`,
   failing that `--hmac-keyfile <path>` on the command line. The file
   must be plain **uppercase** hex, no spaces or separators, one
   optional trailing newline, and `chmod 0600` (personal) or `0640`
   (the shared, group-`sso-ops`-readable copy) - any world bit or a
   stray ACL is rejected. The operator banner shows `(N bytes ok)`,
   `(MISSING)`, or `(BAD - see --self-test)` so you can confirm it loaded; the
   bytes never reach the UI. If it shows `(BAD)` or `(MISSING)`, see
   [the HMAC banner entry under Troubleshooting](#troubleshooting) -
   `uplink_test --fix-permissions` repairs a permissions/ACL problem in
   place.

4. **Rotator calibration.** One-time per controller:

   ```sh
   simple_sat_ops --control --calibrate-rotator --confirm-rotator-calibrate
   ```

   The antenna moves through three legs (park, az $90^\circ$, el
   $45^\circ$, park).
   Results write to
   `~/.local/share/simple_sat_ops/rotator_{az,el}_rate_dps`. The
   pursuit planner reads them at the next normal startup. See
   [Rotator calibration](#rotator-calibration---calibrate-rotator).

5. **T/R switch (optional).** If `/dev/ttyACM0` is present, the
   operator UI auto-probes it on start. No flags needed.

## Before you operate: licensing and authorization

Setting up the station is the easy part. Operating it carries legal
and institutional obligations, and they are not optional. Before you
run a pass - even as the person at the keyboard of a *remote* session -
every item below must apply to you. If any is missing, you are a
viewer, not an operator.

> **Safety.** This is the one chapter where "the software won't let me"
> is not the safeguard. The transmitter inhibit protects the hardware;
> nothing in the code checks your licence. That check is on you.

**Amateur-radio qualification (ISED Canada).** Under the University of
Calgary's rules - which follow from the authorization Global Affairs
Canada has granted - you personally need:

- a **Basic** ISED amateur-radio qualification to operate the antenna
  in *receive* mode, and
- an **Advanced** qualification to *transmit*, whether you are sitting
  at the RAO station or driving it remotely.

These qualifications are yours, not the station's. Hold the right one
before you take the corresponding role.

**Approval to operate the satellite.** Hearing the spacecraft is one
thing; commanding it is another. You must be approved by Dr. Burchill
before you operate FrontierSat itself.

**The rules of the road.** Be fluent in amateur-radio operating
practice and etiquette: band plans, station identification, listening
before you transmit, yielding to other users. The bands are shared and
the hobby runs on courtesy; operate accordingly.

**The RSSSA constraints.** FrontierSat is a remote-sensing space
system, so its operation is governed by Canada's **Remote Sensing
Space Systems Act (RSSSA)**, administered by Global Affairs Canada. The
Act, and the licence issued under it, constrain what the system may do
and how we may operate it. You are expected to know what the RSSSA is
and how it limits us, and to have read both our RSSSA application and
the GAC-issued licence before you operate.

**Your own account.** Operate from a *personal* account on the RAO
ground-station computer - never a shared or generic login. The
same-operator enforcement and the audit log both assume one human per
account, and the licensing obligations above are individual, so the
record has to be too.

Until the whole list is true for you, sit in viewer mode (no
`--control`), watch, and learn. That is exactly what it is for.

## A map of the cat: the tools

Everything from here to the appendices is the tool reference. It is
meant to be consulted, not held in your head: learn that these tools
exist and roughly what each is for, then look up the details when you
need them. This table is the outline you name the parts off.

One binary touches the hardware during a pass; the rest plan it,
review it, or take apart what it recorded.

| When you want to | Reach for |
|------------------|-----------|
| Run a live pass (the only tool that drives the radio and rotator) | [`simple_sat_ops`](#operator-ui-simple_sat_ops) |
| Find the next pass and plan a schedule | [`next_in_queue`](#pass-scheduling-next_in_queue) |
| Summarize a TLE's orbital elements (keps) | [`tle_keps`](#tle_keps) |
| Sanity-check a command list before you send it | [`agenda_check`](#agenda-review-agenda_check) |
| Pull frames out of a recorded capture offline | [Offline analysis tools](#offline-analysis-tools) |
| Bench bring-up, one-shot test transmits, IQ recording | [Bring-up and test tools](#bring-up-and-test-tools) |
| Confirm the math still holds after a change | [Testing and validation](#testing-and-validation) |

Every tool accepts **`-V` / `--version`**, which prints its name and the
git commit the build was made from (with a `-dirty` suffix if the working
tree had uncommitted changes), e.g. `packet_query a1b2c3d4e5f6
(2026-06-01)`. The commit is baked in at build time, so it reflects the
exact build, not a runtime `git` call. This is the value to record on a
pass sheet when you note which build flew the pass; `simple_sat_ops
--self-test` also prints it on a `version:` line alongside the rest of the
configuration snapshot.

## Operator UI: `simple_sat_ops`

The operator UI is an ncurses program that:

* propagates the satellite from a TLE (SGP4/SDP4) and shows live
  Doppler-corrected uplink and downlink frequencies,
* drives the SPID rotator on a worker thread,
* owns the B210 in RX continuously and switches to TX briefly during
  each uplink burst,
* broadcasts state to read-only viewers over a Unix-domain IPC
  socket,
* writes a continuous IQ and WAV capture into a per-pass folder
  under `/FrontierSat/Operations/<yyyymmdd>/<HHMMLT>/`,
* decodes AX100 frames on the fly and inserts each one into a shared
  SQLite packet database.

It also lets the operator compose and send individual uplink
telecommands interactively, or pull a list of timed telecommands
from disk and stream them across the pass at their scheduled
execution times.

### Modes: operator vs. viewer

```text
simple_sat_ops --control [<satellite>] [options]        # operator mode
simple_sat_ops [options]                                 # viewer mode (an operator must be running)
simple_sat_ops --viewer-stream [<satellite>] [options]   # headless JSON stream (no operator required)
```

**Operator (`--control`).** Opens the IPC socket
(`/run/sso/simple_sat_ops.sock`), takes ownership of the B210,
rotator, and pass folder. Refuses to start if another operator is
already running. The refusal message names the running operator and
points at the force-claim path (see
[Troubleshooting](#troubleshooting)).

**Viewer (no flag).** Connects to the running operator's socket and
renders the same state read-only. There is no longer a "track on my
own" standalone mode: without `--control`, if no operator is running,
the program exits with `operator not found: try simple_sat_ops
--control ...` rather than starting. A satellite name given without
`--control` is ignored - a viewer mirrors whatever the operator is
tracking. Press `c` then `y` inside the confirmation window to
force-claim. The running operator yields, the socket disappears, and
the viewer re-execs into `--control` with the same TLE and pass folder.

**Stream (`--viewer-stream`).** A headless feed for a remote,
machine-readable viewer (the iOS/iPadOS app over SSH). It writes
newline-JSON to stdout, draws no terminal UI, opens no hardware, loads
no signing key, and can neither transmit nor take control. With no
operator running it grabs the latest TLE the same way `--control` does,
propagates the satellite itself, and streams the prediction tagged
`source:"tle-only"`. It watches the runtime directory for an operator's
socket, so the moment a `--control` operator starts it relays that
broadcast tagged `source:"operator"`, falling back to TLE-only when the
operator drops.
It runs whether or not an operator is up, and any number of streams can
attach to one operator (each is just another read-only client).
`--control` and `--viewer-stream` together are refused. The wire format
is specified in [the viewer-stream JSON contract](VIEWER_STREAM_JSON.md).

> **Drill (your first pass).** Find an operator already running, or
> start one yourself, then open a second `simple_sat_ops` with no
> flags. You are now a viewer: every panel updates and nothing you
> type reaches the hardware. Watch a full pass go by. Notice how the
> Doppler figures walk through the carrier as the satellite rises and
> sets, how the rotator panel trails the satellite az/el by a degree
> or two, and how the frame counter ticks when the link is good. When
> the pass ends you will have seen everything an operator does, at zero
> risk to anything.

### Command-line options

`--help` and `--help-full` are authoritative. Most-used flags:

| Flag | Purpose |
|------|---------|
| `--control` | Operator mode (one allowed at a time). |
| `--viewer-stream` | Headless newline-JSON stream to stdout for a remote viewer; no terminal UI, no hardware, no TX. TLE-only until an operator appears, then relays it. Refused together with `--control`. See [the JSON contract](VIEWER_STREAM_JSON.md). |
| `--tle <path>` | Path to a 3-line TLE file. Default: `$HOME/.local/state/simple_sat_ops/active.tle`. Space form is tab-completable; `--tle=<path>` also works. |
| `<satellite_id>` | Name prefix to match in the TLE. Optional with `--control` or `--viewer-stream` (auto-discovered from the TLE name). Ignored by a plain viewer - it mirrors the operator's target. |
| `--lat=<deg>` `--lon=<deg>` `--alt=<m>` | Observer override. Defaults to the Rothney Astrophysical Observatory (Priddis, SW of Calgary). |
| `--rotator-device <path>` | Override the SPID tty. |
| `--without-rotator` (alias `--without-hardware`) | Skip the SPID entirely. |
| `--without-tr-switch` | Skip the T/R switch probe. |
| `--tr-switch-device=<path>` | Override the T/R switch tty. |
| `--without-b210` | Run UI plus rotator only (skip the SDR). |
| `--sdr-type=uhd\|rtlsdr\|auto` | SDR backend (default `auto`: probe UHD, then RTL-SDR). See [SDR backends](#sdr-backends). |
| `--uhd-args=<args>` | UHD device args verbatim (e.g. `type=b200,serial=...`); overrides detection. |
| `--sdr-fpga=<path>` | Force a UHD FPGA image (a B2xx clone whose bitstream differs from stock). |
| `--sdr-device=<sel>` | RTL-SDR dongle index (for UHD prefer `--uhd-args`). |
| `--no-tx` | Open the SDR but block PA keying. The TX compose modal still shows preview and dry-run. |
| `--hmac-keyfile <path>` | Override the HMAC keyfile. |
| `--tc-file <path>` | Telecommand list for the `A` auto-tcmd modal. Linted against the firmware at startup; lint errors refuse startup (see [Telecommand linting](#telecommand-linting)). |
| `--ignore-at-your-peril-all-tc-errors` | Start even when the `--tc-file` agenda has telecommand lint errors. Warnings never block. |
| `--ignore-at-your-peril-dangerous-tcmds` | Start even when the `--tc-file` has a brick-risk command (a `danger:` finding, e.g. one that arms the boot-time agenda). A separate gate from the errors one, so accepting parse errors never also waves a brick command through (see [Telecommand linting](#telecommand-linting)). |
| `--calibrate-rotator` `--confirm-rotator-calibrate` | One-shot calibration mode (see below). |
| `--without-rotator-pursuit` | Disable the pursuit / lead-aim planner; the track loop falls back to today's aim-where-sat-is-now logic. Useful for A/B on the bench. |
| `--scan-sky` `--scan-step=<deg>` | Drive the rotator through a sky grid, dwelling at each target. Bypasses the satellite-tracking gate. |
| `--always-record` | Start WAV and IQ capture immediately at open; don't gate on elevation. |
| `--live-waterfall` | Auto-launch the raylib `live_waterfall` viewer alongside the terminal UI. |
| `--self-test` | Print the resolved configuration and exit (includes a `version:` line with the build commit). Useful in scripts. |
| `-V` / `--version` | Print the build commit and exit (see [the tool map](#a-map-of-the-cat-the-tools)). |

`--help-full` also lists the TX safety gates and viewer options.

### SDR backends

`simple_sat_ops` talks to the radio hardware through a pluggable SDR
backend. By default (`--sdr-type=auto`) it probes the compiled-in
backends in order - UHD first, then RTL-SDR - and uses the first that
opens. Pick one explicitly with `--sdr-type=uhd` or `--sdr-type=rtlsdr`.
The active SDR and its transmit capability are shown on the RX panel
(`SDR <name>` or `SDR <name> (RX-only)`) and on the startup line.

**Confirm the hardware first with `sdr_probe`.** Before a pass, run
`sdr_probe` to see what's attached without starting `simple_sat_ops`. It
opens the UHD device the same way the operator UI would, prints its name
and the RX/TX channel counts and antennas, names the ports
`simple_sat_ops` uses (receive on `RX2`, transmit on `TX/RX`), and then
lists any RTL-SDR dongles. Use `sdr_probe --uhd-args=serial=...` to point
at a specific device when more than one is present.

**USRP (UHD) - the operational path.** The UHD backend transmits and
receives, so all operator functions are available.

If you run a **B210-ish clone whose FPGA differs from the stock image**,
that bitstream has to be loaded for the board to come up. A clone is
indistinguishable from a genuine board on the USB bus *except by its
serial number*, so the image is selected per-serial.

The bitstream is **not** in the repository (a large binary derived from
vendor IP, so it is git-ignored and never published). It lives on the
RAO ground-station computer under `/FrontierSat/sdr/`. One-time setup on
your machine:

1. Copy the image into your FrontierSat tree, at the same path used on
   RAO:

   ```
   mkdir -p /FrontierSat/sdr
   scp <user>@va6raogndstn:/FrontierSat/sdr/usrp_b210_fpga.bin /FrontierSat/sdr/
   ```

   If your FrontierSat root is elsewhere, use it - the tools resolve
   `$FRONTIERSAT_ROOT` if set, otherwise `/FrontierSat`.

2. Find the device serial: run `sdr_probe`. It prints e.g.
   `USB serial: 30AA038`.

3. Map the serial to the image in the shared, admin-managed map file
   `/usr/local/share/sso/sdr_fpga_map`. Each line is
   `<serial> <absolute-image-path>`:

   ```
   30AA038 /FrontierSat/sdr/usrp_b210_fpga.bin
   ```

   One file serves every operator, so a shared ground station picks it up
   without any per-home setup. The image lives in the shared tree
   (`/FrontierSat/sdr/`) where all operators can read it - the bitstream
   isn't a secret, so it just needs to be group-readable. Neither the image
   nor the map is shipped or installed by the build; the admin supplies
   both out of band.

4. Confirm: re-run `sdr_probe`. It should report the clone image and the
   board should open (RX2 / TX-RX).

From then on the clone's image loads automatically whenever that serial
is seen; a genuine board with a different serial still gets the stock
image. To override the map for a one-off, pass `--sdr-fpga=<path>`, or
fold it into `--uhd-args="type=b200,serial=...,fpga=<path>"` (which wins
over everything). The serial is read via libusb; UHD's own
device-enumeration call segfaults on macOS, so it is not used.

**RTL-SDR - receive only.** An RTL-SDR dongle (`--sdr-type=rtlsdr`, or
auto with no USRP present) runs the full receive chain: tracking,
Doppler, recording, waterfall, and decode all work. It **cannot
transmit**: the TX compose (`t`) and auto-telecommand (`A`) modals open
for composing and preview but the allow-tx gate is forced off, and a
commit is refused with "TX not supported by this SDR (RX-only backend)".
Pick the dongle with `--sdr-device=<index>` if you have more than one.
The RTL backend is compiled in by default when `librtlsdr` is present
(it auto-disables if the library is missing; `-DWITH_RTL_SDR=OFF` forces
it off).

### Keyboard controls

Keyboard starts **unlocked**. Press `K` to toggle the lock; the status
line reads `unlocked` or `LOCKED`. Lock it when you want to lean on the
keyboard, or hand the screen to someone, without nudging the antenna.

| Key | Action |
|-----|--------|
| `K` | Toggle keyboard lock. |
| `T` | Start tracking the current satellite. |
| `s` | Stop tracking (and stop the rotator). |
| `r` | Stop tracking and return to (az=0, el=0). |
| `[` / `]` | Nudge antenna azimuth -5 / +5 deg. |
| `{` / `}` | Nudge antenna azimuth -1 / +1 deg (fine). |
| `,` / `.` | Nudge antenna elevation -5 / +5 deg. |
| `<` / `>` | Nudge antenna elevation -1 / +1 deg (fine). |
| `t` | Open the TX compose modal. |
| `A` | Open the auto-telecommand modal (requires `--tc-file`). |
| `:` | Enter the colon-command prompt. |
| `q` | Quit. |

The nudge keys stop satellite tracking before moving. They route
through the same worker thread as the main track loop, so they
don't block the UI.

Inside a modal (`t`, `A`, or the `:` prompt) every keystroke is
consumed by the modal until you commit or cancel. The jog
characters above are typed as plain text in that context.

### Status, RX, and TX panels

Screen layout (ncurses, redrawn at ~10 Hz):

* **Top status block.** Operator user, IPC role, mode banner,
  carrier frequencies (nominal and Doppler), TLE pinned for the
  pass, rotator panel (target az/el, current az/el, in-flight or
  stale flag), T/R switch panel, telemetry overlay.
* **Main pass region.** Satellite az/el, range, range-rate,
  sub-point lat/lon, altitude. Per-pass progress bar. Predicted
  pass parameters (AOS, LOS, max elevation, duration). Pass folder
  path.
* **RX panel** (when the B210 is open). Live IQ peak and RMS dBFS,
  frame counter from the live AX100 decode loop, shadow decoder
  frame counts, signal-quality estimate.
* **TX log panel** (bottom). Rolling N-line history of TX events:
  `draft>` as you compose, `sent>` once a command goes on the air,
  and a `notsent>` line *only* when a command did **not** reach the air,
  carrying the reason. A clean send leaves just the `sent>`
  record - the ground station does not acknowledge its own transmit,
  and the satellite's reply, if any, arrives on the downlink (the RX
  panel and packet database), not here. Viewers see the same lines
  via IPC.

Elapsed-time readouts (T/R switch last-TX, RX last-frame age) are shown
in a compact form that lists only the parts that matter: `2s ago`,
`1h 12s ago`, `3d 4h ago` - not a long fractional second count.

**Errors don't scramble the screen.** While the live screen is up, any
stderr from the radio/SDR code or a system library is redirected to
`sso_stderr.log` in the pass folder instead of landing on the panels
(the worst offender was a B210 USB-overflow message that could repeat
many times a second). Nothing is lost - it's captured for later - and
normal stderr is restored when the screen is torn down. On a clean quit
the program prints one final line: `No errors reported` if nothing was
written this run, or `Errors logged in <file>` pointing straight at it
(the check compares the log's size against its size at startup, so a
reused pass folder won't give a false positive from an earlier run).

### Colon-command prompt

Press `:` to open a one-line command bar at the bottom. The prompt
echoes keystrokes to viewers (debounced) so observers see what the
operator is about to commit. Enter fires, Esc cancels.

Line editing:

* **Up / Down** cycle a history of the commands you have run this
  session. The line you were typing is preserved and comes back when
  you press Down past the newest entry.
* **Tab** completes: the first word against the command names; any
  later word as a filesystem path.
* **`$VAR`, `${VAR}`, and a leading `~`** are expanded when a command
  reads a path. Tab completion keeps the `$VAR` / `~` prefix literal in
  the buffer and only completes the trailing component, so
  `:retarget $TLES/20260529/<Tab>` keeps `$TLES` and fills in the file.

Commands:

| Command | Effect |
|---------|--------|
| `:track` / `:stop` | Start / stop tracking the current satellite. |
| `:home` | Stop tracking and return to (az=0, el=0). |
| `:retarget <tle-file>` | Switch the tracked satellite mid-pass to the first one in the file (see below). |
| `:tx` / `:auto` | Open the TX compose / auto-telecommand modal. |
| `:freq <MHz>` | Override the nominal downlink frequency for the current pass (a value below 1e6 is read as MHz, otherwise Hz). |
| `:gain <dB>` | Set the AD9361 RX gain. |
| `:lo_offset <signed-kHz>` | Move the B210's LO offset to dodge a baseband artifact. |
| `:lo_bandwidth <kHz>` | Set the live waterfall's visible bandwidth (needs `--live-waterfall`). |
| `:spectrum <N>` | Render a spectrogram of the last `N` seconds of WAV/IQ via `gen_waterfall` (forked, non-blocking). |
| `:rs on\|off` | Reed-Solomon toggle (not yet runtime-wired; reports as much). |
| `:help` | List the commands. |
| `:quit` | Quit (`:q` and `:exit` too). |

The definitive set is whatever's wired up in `apps/main.c`'s
`cmd_dispatch`; the line-editing keys live in `cmd_handle_key`. Check
those if you need to be sure.

#### `:retarget <tle-file>`

Switch the satellite being tracked partway through a pass - for
example when a second satellite you also want is above the horizon at
the same time. The **first** satellite in the named file is used; its
name need not match anything already loaded. If you are tracking, the
antenna slews straight to the new target on the short path - it does
**not** unwind through (0, 0) first. A repeat `:retarget` on the same
file is a no-op; a different file swaps even when it names the same
satellite. The satellite display row and the viewer broadcast both
follow the new target. The path argument is expanded and
Tab-completable, so `:retarget $TLES/20260529/tle-20260529.tle` works.

### TX compose modal (`t`)

Multi-field modal for composing a single uplink burst. Fields:
payload (hex or ASCII), CSP v1 header (`prio`, `src`, `dst`,
`dport`, `sport`, `flags`), frequency override, gain, repeat count,
gap between repeats, preroll, and three safety-gate checkboxes
(`--allow-tx`, `--allow-high-power`, `--allow-hf-tx`).

Each field edit is debounced (~200 ms) and broadcast to viewers as
a `tx-preview` event, so observers see the draft before commit.
Enter sends a `tx-request`. The burst runs on its own thread, which
brings the transmit chain up (on the B210's `TX/RX` port), sends, and
powers the transmit chain back down; **receive keeps running
continuously the whole time** (on `RX2`), so the satellite's reply in
the seconds right after a command isn't missed. A `tx-command-sent`
event is recorded in the TX log. If the burst never reaches the air -
no SDR, frame-build failure, UHD error, or `--tx-dry-run` - a
`tx-not-sent` event carries the reason instead. Esc cancels.

On an [RX-only SDR](#sdr-backends) (RTL-SDR), the modal still opens for
composing and preview, but the allow-tx gate is forced off and a commit
is refused with "TX not supported by this SDR (RX-only backend)".

### Auto-telecommand modal (`A`)

Requires `--tc-file <path>` on the command line. The file format
matches what the wider CalgaryToSpace tooling uses: one telecommand
per line. Blank lines and whole-line `#` comments are ignored, and an
inline trailing comment is stripped from a command:

```
CTS1+function_name(arg1,arg2)@tssent=<unix_ms>@tsexec=<unix_ms>@resp_fname=<f>!
# a whole-line comment
CTS1+adcs_identification()@tsexec=<unix_ms>!   # an inline comment, ignored
```

A telecommand always ends with `!`, so a `#` is treated as a comment
only when whitespace precedes it; a `#` inside the command text (no
space before it) is left intact, so a command is never silently
truncated on its way to the air.

Before the UI starts, the `--tc-file` is linted against the firmware's
telecommand set - the same check `agenda_check` runs (see [Telecommand
linting](#telecommand-linting)). If any line has a lint *error* (unknown
command name, wrong argument count, broken `CTS1+...!` framing),
`simple_sat_ops` prints the offending lines and refuses to start, so a
malformed agenda is caught before the pass instead of on the air. Pass
`--ignore-at-your-peril-all-tc-errors` to start anyway. A *danger*
finding - a well-formed but brick-risk command, such as one that arms
the boot-time agenda - also refuses startup, behind its own separate
gate `--ignore-at-your-peril-dangerous-tcmds`. *Warnings* (such as a
command not meant for routine flight operation) are printed but do not
block startup. The same re-lint runs each time the `A` modal (re)loads
the file, so an edit made after launch is caught too: a danger blocks
the run with a `can BRICK the satellite` status until you fix the file
or, having started with the gate, override it.

The `A` modal lists the commands, lets the operator set per-pass
parameters (power, repeats, delay), and ticks the same three TX
safety gates. Once running, each tick checks whether the next
command's `@tsexec=` has arrived. If it has, the command is staged
into the same `tx-request` slot the compose modal uses. The TX
dispatch is async (submit + poll), so the rotator, redraw, IPC, and
the next auto-tcmd tick keep running while each burst is in flight.

The auto-tcmd loop respects the safety gates and the LOS guard.
Once the pass ends, it stops sending commands. On an
[RX-only SDR](#sdr-backends) the loop refuses to send (the same RX-only
gate as the compose modal).

Viewers follow the run too: once a run starts, the state broadcast
carries its progress, and every viewer's status panel gains an
`auto-tcmd   <sent>/<total> sent (<state>)` line - red while running,
then `stopped`, `done`, or `pass-over` with the final tally. The line
disappears when the operator closes the modal. (The per-send detail -
each command as it goes out - already appears in everyone's TX log
panel; this line is the at-a-glance summary.)

### simple_sat_ops-directed commands (`SSO+`)

Some operations need a value only the ground knows at the moment of
transmission - the current time, say. For those, an agenda line (or a line
typed into the `t` compose modal) may use a `SSO+` directive in place of a
literal `CTS1+...!` telecommand. `simple_sat_ops` expands it into a real
telecommand *at transmit time*, then sends and logs that.

The first such directive is the clock sync:

```
SSO+sync_sat_time_to_ground()!
```

which expands, each time it is queued, to:

```
CTS1+set_system_time(<current_utc_ms + 500>)@tssent=<pass-start ms>!
```

The argument is the ground's current UTC in milliseconds plus 500 ms - a budget
for ~250 ms of transmit lag and ~250 ms of average on-board execution lag - so
the satellite's clock lands close to true UTC when the command runs. The
timestamp is recomputed every time the line is queued, so if the agenda is
retransmitted, or the command is sent more than once, each copy carries a fresh
time rather than a stale copy of the first.

The `@tssent` value is the pass-start time: the UTC at which `simple_sat_ops`
started, truncated to the minute, held constant for the whole session. The
flight firmware can be configured to ignore a telecommand whose `@tssent` it has
already seen, so a value constant across the session means a re-sync runs once
per pass and retransmissions are dropped - while the next pass (a new
`simple_sat_ops` run, a new minute) syncs again. Relaunch `simple_sat_ops` for
each pass.

What goes on the air is the expanded `CTS1+...!` command, and that is what the
TX log records - annotated with the directive it came from:

```
ascii:CTS1+set_system_time(1749752645623)@tssent=1749752580000! (replaced 'SSO+sync_sat_time_to_ground()!')
```

`SSO+` lines are checked at startup with everything else in the `--tc-file` (an
unknown directive or a wrong argument count refuses startup the same way a
malformed `CTS1+` line does, see [Telecommand
linting](#telecommand-linting)), and they behave identically from the `A`
auto-telecommand modal and the `t` compose modal.

### Finding telecommands and their arguments

`simple_sat_ops` only carries telecommands to the satellite - it does
not define them. The authoritative documentation lives with the flight
firmware in the CalgaryToSpace repository:

- Mission operations ICD (start here - the directory of operations
  documents):
  <https://github.com/CalgaryToSpace/CTS-SAT-1-OBC-Firmware/tree/main/docs/Mission_Operations>
- The command list itself - commands, their arguments, and the
  responses to expect:
  <https://github.com/CalgaryToSpace/CTS-SAT-1-OBC-Firmware/blob/main/docs/Mission_Operations/Telecommands_and_Config_Variables.md>

Use the command list to look up the exact function name, its argument
order and types, and the form of the reply on the downlink. The command
you type into the `t` compose modal (or list in a `--tc-file`) is the
same `CTS1+function_name(...)!` string documented there.

The ground tools also lint commands against a copy of this command set
generated from the firmware: `agenda_check` and `simple_sat_ops` startup
both check every command's name, argument count, and `CTS1+...!` framing
(see [Telecommand linting](#telecommand-linting)), so a typo or a wrong
argument count is caught before the pass. That copy is pinned to a
firmware tag and regenerated when the command set changes; it validates
the *structure* of a command, not its intent, so the firmware command
list above remains the source of truth for what each command does.

> Note: the manual link points at `main`. The image you are actually
> flying may be a tagged release rather than `main`; when in doubt,
> read the docs at the firmware tag that is on the spacecraft.

#### Populating a `--tc-file` for auto-commanding

This is the authoring side of telecommanding, and it is a different
skill from operating. Operating a pass means flying a file someone has
already vetted; *writing* that file means deciding what the spacecraft
will be told to do, and the ground software does not - cannot - check
your intent. Treat it as an intermediate-to-advanced activity: do it
alongside someone who knows the spacecraft until you know it too.

**The line format.** One telecommand per line, the same string the `t`
compose modal would send, with optional `@`-prefixed metadata before the
closing `!`:

```
CTS1+function_name(arg1,arg2)@tssent=<unix_ms>@tsexec=<unix_ms>@resp_fname=<file>!
```

| Field | Meaning |
|-------|---------|
| `function_name(...)` | The command and its arguments, exactly as the firmware command list documents them - name (case-sensitive), argument order, and types. See [Finding telecommands and their arguments](#finding-telecommands-and-their-arguments). |
| `@tsexec=<unix_ms>` | When the satellite should *execute* the command (on-board clock). The `A` loop holds each command until this time arrives, so this is how you schedule a sequence across a pass. Omit it to run on receipt. |
| `@tssent=<unix_ms>` | The send-time / dedup key. The firmware can drop a command whose `@tssent` it has already seen, so a stable value makes a command run once even if the burst is retransmitted (see the [`SSO+`](#simple_sat_ops-directed-commands-sso) clock-sync note). |
| `@resp_fname=<file>` | Optional name for the on-board response file the command writes, when it produces one. |

Blank lines and comments are stripped per the rules in the [`A` modal
section](#auto-telecommand-modal-a).

**Schedule, then check.** Set `@tsexec=` values that fall inside the
pass window (use [`next_in_queue`](#pass-scheduling-next_in_queue) for
the AOS/LOS times), leave room between commands for each burst and its
reply, then run the file through
[`agenda_check`](#agenda-review-agenda_check) *before* the pass. It
converts every `@tssent`/`@tsexec` to readable time so you can eyeball
the order, flags duplicate lines, and lints every command against the
firmware command set; with `--tle` it even prepends the sub-satellite
point at each execution time. Fix every `ERROR>` and read every
`warning:` before you fly the file. `simple_sat_ops` runs the same lint
at startup and refuses to start on an error.

> **Safety - it is possible to brick the satellite.** The linter checks
> *structure*, not *intent*. A command can be perfectly well-formed -
> right name, right arguments, passes the lint, starts the UI without a
> murmur - and still be the wrong thing to send. To help you tell them
> apart, every command carries a *readiness level* taken from the
> firmware, and the linter prints a `warning:` for anything that is not
> routine flight operation:
>
> | Readiness | Meaning |
> |-----------|---------|
> | `operation` | Normal flight operation. The only level meant for routine use. |
> | `recovery/expert` | Flight-safe but expert-only. |
> | `flight-testing` | Flight-safe but disruptive (e.g. a flash test). |
> | `ground-only` | Umbilical / ground use only - not for a flying spacecraft. |
> | `high-risk/unsafe` | High risk; can do permanent harm. |
>
> Here is how a *valid* file bricks the spacecraft. Suppose a file,
> meaning to test booting from the satellite's second firmware bank,
> schedules:
>
> ```
> CTS1+stm32_internal_flash_set_active_flash_bank(2)@tsexec=...!
> CTS1+eps_system_reset()@tsexec=...!
> ```
>
> Both lines lint clean - the names and argument counts are right - so
> all the ground tells you is two `warning:` lines (`high-risk/unsafe`
> and `recovery/expert`), and nothing refuses to run. But if bank 2 was
> never programmed with a valid image, the reset hands control to an
> empty bank: the processor boots nothing, the command receiver never
> comes up, and no telecommand can fix it because nothing on the
> spacecraft is left listening for one. The satellite is bricked. The
> same shape of mistake hides behind the other flash and filesystem
> commands - `stm32_internal_flash_bank_erase`,
> `flash_force_corrupt_filesystem`, `fs_format_storage` - and behind any
> destructive command scheduled ahead of the one that would have undone
> it.
>
> That example *warns* but does not block - readiness is advisory. There
> is one brick path the ground stops outright. The firmware keeps a
> boot-time agenda file, `default_tcmd_agenda.txt`: whatever it contains
> is loaded into the agenda queue on *every* boot (a last-resort way to
> resume an agenda after repeated crashes). Write that file with a
> command that crashes the satellite -
>
> ```
> CTS1+fs_write_file_str(default_tcmd_agenda.txt,<a command that crashes on boot>)!
> ```
>
> - and you have armed a trap that springs on the next reset and every
> reset after it: boot, load the bad agenda, crash, repeat. Nothing is
> left listening long enough to fix it. Because this command is routine
> in shape (it is just a file write, `operation` readiness), the linter
> would not even warn - so it is on a small ground-side *brick-risk
> blacklist* and flagged as a `danger:`, which **refuses startup** unless
> you pass `--ignore-at-your-peril-dangerous-tcmds`. That gate is
> deliberately separate from the errors gate: waving through a typo must
> never also wave through a boot-loop.
>
> The protections are real but bounded: the readiness warnings, the
> brick-risk `danger:` blacklist, the `agenda_check` dry run, and a
> second qualified reader. None of them *understands* your agenda;
> `simple_sat_ops` carries bytes, it does not vet missions, and it is not
> designed to keep you from bricking the satellite. (It cannot: there are
> days a satellite needs to be bricked on purpose - a deliberate reflash,
> an end-of-life passivation - which is why the blacklist is a gate you
> can open, not a wall. We are mostly joking. Mostly.) The rule is the
> dull one - never fly a file you do not fully understand, and have
> someone who knows the spacecraft vet anything that carries a warning.

### Rotator calibration (`--calibrate-rotator`)

The pursuit planner needs the SPID's deg/s slew rates on each axis.
They're set on the SPID controller's front panel and aren't reported
via the W-frame STATUS reply, so we measure them by moving the
antenna across a known arc and watching the encoder.

This is the SPID quirk promised in the foreword. The controller
updates its reported position the instant it receives a SET, so the
first STATUS after a move is a confident lie: the target dressed up as
the truth. The calibrator works around it by discarding the first
couple of samples and waiting until the encoder is genuinely crawling
toward the target before it trusts a number.

```sh
simple_sat_ops --control --calibrate-rotator --confirm-rotator-calibrate
```

Three legs:

1. Park to (az=0, el=10). The starting move is not timed; it just
   establishes a known reference.
2. Sweep az 0 to 90 deg, streaming STATUS samples.
3. Sweep el 10 to 55 deg, streaming STATUS samples.

For each timed leg the calibrator:

* drops the first two samples after the SET (the SPID firmware's
  position counter updates atomically on receipt, so the first
  STATUS reflects the **target**, not the encoder),
* drops the last two samples (deceleration and settle),
* finds the first and last interior samples that show motion
  ($>0.1^\circ$ from their neighbor),
* computes the rate as $|\Delta v| / \Delta t$ across that span.

If the firmware actually target-latches and never reports encoder
progression, the interior shows no motion and the calibrator bails
with `bad_rate` instead of producing a misleading number.

On success it writes
`~/.local/share/simple_sat_ops/rotator_{az,el}_rate_dps` (plain
text, one float each). Re-run any time the controller's rate
settings change.

**Safety.** `--confirm-rotator-calibrate` is required because the
antenna physically moves through all three legs without further
prompting. Clear the mast area before you run it. A stray
`--calibrate-rotator` left in a script can't move anything without the
confirm flag.

### Pursuit tracking

The rotator slews at a fixed, modest rate. The satellite, near the
apex of a high pass, does not: it can cross the sky faster than any
reasonable antenna can follow. Aiming at where the satellite is *right
now* means arriving late on every tick, worst at the top of the pass
where the signal is best. Pursuit aims instead at where the satellite
is about to be, planning the whole arc in advance so the rotator's
steady slew coasts through the curve rather than chasing it.

When rotator calibration is on disk and `--without-rotator-pursuit`
is **not** passed, then at pass start (`T`, or auto AOS) the
operator UI:

1. Pre-samples the satellite's mech-frame az/el every 1 s across
   the pass on a working copy of the prediction (so the live
   displayed az/el is not disturbed), applies the flip mech-coord
   mapping for high-elevation passes, and unwraps azimuth across
   0/360 in time order.
2. Asks the pursuit planner (`src/orbit/pursuit.c`) for a
   rate-feasible whole-pass antenna trajectory.
3. On each track-loop tick reads the next waypoint via
   `pursuit_aim_at(jul_utc)` and submits it through the same path
   today's aim-where-sat-is-now logic uses. The constant-rate slew
   between waypoints interpolates the segment.

stderr at pass start looks like:

```
pursuit: plan built 64 waypoints, max_err=1.42 mean_err=0.61 deg, 3 iter
```

If the planner produces a plan with max error > 30 deg the plan is
discarded with `pursuit: plan max_err=N.N deg > 30; disabled` and
the track loop falls back to the old aim-now behavior. Same
fallback fires when calibration is absent, the planner fails to
build, or `--without-rotator-pursuit` is set.

The plan is freed at LOS, on any mid-pass abort (out-of-bounds az,
operator presses `s`), and at process exit. See
[Pursuit planner internals](#pursuit-planner-internals).

## Pass scheduling: `next_in_queue`

A read-only CLI that reports upcoming passes over the observer
location.

```text
next_in_queue <satellite_name> [options]
next_in_queue <min_alt_km> <max_alt_km> [<satellite_name>] [options]
next_in_queue --tle <path> [<satellite_name>] [options]
next_in_queue --trajectory-id=<id> [options]
```

**Named form (the common case).** Give just a satellite name and
`next_in_queue` finds its TLE for you and reports the next week of
passes - no altitude limits needed, because you already know the
object. It looks, in order:

1. the newest dated TLE under `/<satellite_name>/TLEs` (the
   `<date>/tle-<date>.tle` convention - e.g. `FrontierSat` resolves
   to `/FrontierSat/TLEs/20260529/tle-20260529.tle`, picking the
   latest date);
2. failing that, the default `$HOME/.local/state/simple_sat_ops/active.tle`,
   filtered to the named object;
3. failing both, it errors. Override the whole search with `--tle=<path>`.

**Catalog-scan form.** Give an altitude band (`<min_alt_km>
<max_alt_km>`) to scan every object in the default TLE, optionally
narrowed by a trailing name prefix.

Common options (see `--help-full` for the full list):

| Flag | Purpose |
|------|---------|
| `--tle <path>` (also `--tle=<path>`) | TLE file. Default: `$HOME/.local/state/simple_sat_ops/active.tle`. |
| `<satellite_name>` | Literal case-sensitive name prefix. Bypasses the constellation filter, extends the window to a week. |
| `--regex=<pattern>` `--ignore-case` | Regex match on the satellite name. |
| `--list` | Print every matching pass (default is just the next one). |
| `--reverse` | Sort latest-first. |
| `--max-passes=<n>` | Limit the output. |
| `--max-minutes=<n>` | Window. Default 1440 (one day); satellites named via positional get a week. |
| `--min-elevation=<deg>` `--max-elevation=<deg>` | Peak-elevation filter. |
| `--min-altitude-km=<km>` `--max-altitude-km=<km>` | Orbital-altitude filter. |
| `--minutes-offset=<n>` `--t0=<yyyy-mm-ddThh:mm:ss>` | Advance `now` by `n` minutes, or pin an absolute UTC start. |
| `--all-satellites` | Include Starlink, OneWeb, and other commercial constellations (default off). |
| `--show-radio-info` | Annotate matches with amateur-radio info from `active_radios.txt` (looked up alongside the TLE). |
| `--lat= --lon= --alt=` | Observer override. |
| `--trajectory-id=<id>` | Use a propagated SSM trajectory instead of a TLE. |

Examples:

```sh
# Next week of FrontierSat passes - finds /FrontierSat/TLEs/<date>/tle-<date>.tle
next_in_queue FrontierSat --list

# Same, but force a specific TLE file instead of auto-discovery
next_in_queue FrontierSat --list --tle $TLES/20260529/tle-20260529.tle

# Soonest visible pass above 10 deg, in the next 2 hours (catalog scan)
next_in_queue 0 2000 --min-elevation=10 --max-minutes=120

# All passes for the next 3 hours above 30 deg (catalog scan)
next_in_queue 0 2000 --list --max-minutes=180 --min-elevation=30

# ISS-class passes with amateur info annotation.
# With --tle the altitude positionals are not allowed; use the
# --min/--max-altitude-km flags instead.
next_in_queue --tle $TLES/amateur.tle --list \
              --min-altitude-km=300 --max-altitude-km=500 \
              --regex='ISS|ZARYA' --ignore-case --show-radio-info
```

`--tle` accepts the space form (TAB-completable) and the legacy
`=`-form interchangeably. The help text shows the space form.

## Agenda review: `agenda_check`

Telecommand-list reviewer and linter. Reads a TC file (same format the
operator UI consumes via `--tc-file`) and prints it back with the
embedded `@tssent=` and `@tsexec=` unix-millisecond timestamps
converted to ISO 8601, so the operator can confirm the schedule
before keying anything on air. It also lints each telecommand against
the flight firmware's command set (see [Telecommand
linting](#telecommand-linting) below).

```text
agenda_check [--local-time] [--no-dup-check] [--no-tc-lint] [--errors-only] [--prune-dups] [--tle <file>] [<file>]
```

| Flag | Effect |
|------|--------|
| `--local-time` | Render times in the host's local timezone (default UTC). |
| `--no-dup-check` | Skip the duplicate-line audit (substitute the timestamps only). |
| `--no-tc-lint` | Skip the telecommand lint (see [Telecommand linting](#telecommand-linting)). |
| `--errors-only` | Print only the lines with lint errors (line number, command, and reason) to stdout and suppress the rest, so errors don't scroll away in a long agenda. |
| `--prune-dups` | Drop verbatim-duplicate command lines (keep the first occurrence). Prints a count of how many were pruned. |
| `--tle <file>` | (sgp4sdp4 builds only) Propagate the first satellite in `<file>` and prepend the execution date-time plus sub-satellite lat/lon/alt to each command. Leaves the command intact (raw unix-ms preserved). |

Verbatim duplicates are flagged inline with a red `DUP(line N)>`
prefix pointing at the line number of the first occurrence. Pipe
to `grep` and the plain prefix survives.

A line the linter rejects is tagged `ERROR>` and, on a terminal, the
whole line is shown in bold bright red so it can't scroll past
unnoticed; piped, the `ERROR>` tag stays as plain text. The
highlighting applies only to the normal listing — `--errors-only`
already isolates the bad lines, so it adds none there.

With `--tle`, each command line is prepended with `<iso-time>
lat=<deg> lon=<deg> alt=<km>` (in mech coords, lat one decimal,
fixed-width). Without `--tle`, the inline `@tssent=` and `@tsexec=`
timestamps are humanized in place.

Comments follow the same rule the operator UI uses (the rule lives in
one shared place, so the audit and the transmit path can't disagree).
Whole-line `#` comments are passed through verbatim. An inline trailing
comment (whitespace + `#...`) is split off the command: it is preserved
in the output, but excluded from the command count and the
duplicate/timed keys - so the same command carrying two different
comments is still flagged as a duplicate, and a `#` inside a command is
left intact.

After processing, a stderr summary lists:

```text
agenda_check: 918 commands total, 780 non-duplicate, 6 distinct timed telecommands
```

Where:

* *total* = every command line (non-comment, non-blank).
* *non-duplicate* = distinct verbatim lines. Same command at
  different times stays distinct.
* *distinct timed telecommands* = command identities (with `@tssent=`
  and `@tsexec=` values masked out) among lines that carry a timestamp.
  A `fs_list_directory_json(/,0,20)` scheduled 30 times across the
  day counts once.

`--prune-dups` returns 0 with `agenda_check: pruned N duplicate
line(s)` on stderr. The default flag mode returns 3 when duplicates
were present (so CI can gate on it). Telecommand lint errors take
precedence: the exit code is 4 when any were found.

### Telecommand linting

Every command line is checked against the flight firmware's telecommand
set before it could ever be transmitted. The command names, argument
counts, and risk levels come from a table generated from the firmware
(`scripts/gen_tcmd_spec.py`, currently tag `sat-1-rc3`), and the checks
mirror the firmware's own parser, so the verdict matches what the
satellite would do with the same bytes:

* the `CTS1+` prefix and the single terminating `!`,
* a known command name (case-sensitive),
* the parenthesised argument list with the exact argument count the
  command expects,
* the firmware length limits, and
* well-formed `@tssent=` / `@tsexec=` timestamps.

Findings come in three severities, printed to stderr as `line N:
warning|error|danger: ...`:

* **warning** - the satellite would run it, but it is worth a second
  look: a command not meant for routine flight operation (ground-only,
  high-risk, recovery/expert, or flight-testing), or a line too long for
  one radio frame. Warnings never block.
* **error** - the satellite's own parser would reject or mis-parse it:
  bad framing, an unknown name, the wrong argument count, an over-length
  line, a malformed timestamp. The line is tagged `ERROR>` (bold bright
  red on a terminal, plain text when piped) in the listing.
* **danger** - a perfectly well-formed command, one the satellite would
  accept without complaint, that is on a ground-side *brick-risk
  blacklist*. It is tagged `DANGER>` and ranks above an error, because
  the problem isn't the bytes - it's what running them does.

The blacklist is short and is the one check that does not mirror the
firmware parser; it is ground policy. Its first (and at the time of
writing only) entry is any command that names the boot-time agenda file
`default_tcmd_agenda.txt` - the file the satellite loads into its agenda
queue on every boot. Arm it with crashing commands and the spacecraft
can boot-loop forever (see the worked example under [Populating a
`--tc-file`](#populating-a---tc-file-for-auto-commanding)). The match is
on the filename substring alone, so it catches the command whichever way
it is written - `fs_write_file_str`, `fs_write_file_hex`,
`agenda_enqueue_from_file`, even `fs_delete_file` - none of which would
otherwise raise so much as a warning.

`--no-tc-lint` disables the check entirely. `--errors-only` flips the
output around: it prints just the error/danger lines (number, command,
and reason) to stdout and suppresses the echoed agenda, so the serious
findings in a long agenda don't scroll away.

The same lint gates `simple_sat_ops` startup, and the two blocking
severities have *separate* override gates so that accepting one never
silently accepts the other:

* lint **errors** refuse startup unless you pass
  `--ignore-at-your-peril-all-tc-errors`, and
* a **danger** finding refuses startup unless you pass
  `--ignore-at-your-peril-dangerous-tcmds`.

A word on what this is and isn't: the linter is a tripwire, not a
safety harness. `simple_sat_ops` is not designed to prevent you from
bricking the satellite, and it could not if it tried - it carries bytes;
it does not second-guess the mission. The blacklist catches the one
foot-gun we know the name of and makes you say "yes, really" before it
goes up. And sometimes "yes, really" is the right answer: there are
days when a satellite genuinely needs to be bricked on purpose - a
deliberate reflash, an end-of-life passivation, a recovery move that
looks destructive because it is - which is exactly why the gate is a
gate and not a wall. (We are, of course, mostly joking. Mostly.)

**Testing it - on the ground, never on the satellite.** The whole gate
is a static text check, so it is exercised entirely on the dev host with
temp files standing in for an agenda - no radio, no spacecraft. CI does
this on every push: `unit_tests/tcmd_lint_selftest.c` writes throwaway
agendas (a clean one, one that arms `default_tcmd_agenda.txt`, one with a
parse error, one with both) through the *real* linter and the *real*
gate decision (`tcmd_lint_gate_decision`, the same function
`simple_sat_ops` startup and the `A` modal call), and asserts every
override combination - including the one that matters most: passing
`--ignore-at-your-peril-all-tc-errors` alone does **not** let a brick
command through. The override only opens its own gate.

You can also rehearse the real binary end to end, safely, **when the
satellite is well outside any pass window**. Point `simple_sat_ops` at a
test `--tc-file` and launch: a `danger:` line refuses startup before the
UI even opens, and adding `--ignore-at-your-peril-dangerous-tcmds` lets
it proceed into the run so you can watch the auto-telecommand flow. With
no spacecraft in view, the LOS guard and the [TX safety
gates](#tx-safety-gates) mean nothing is keyed regardless - so it is a
free dress rehearsal of the exact path you would fly, with zero risk to
the bird. (Do it during a pass and you would be transmitting for real;
don't.)

To regenerate the command table after a firmware change, see the header
of `scripts/gen_tcmd_spec.py`; to add an entry to the brick-risk
blacklist, see `tcmd_dangerous_substrings[]` in `src/proto/tcmd_lint.c`.

## Offline analysis tools

### `gen_waterfall`

Renders a SatNOGS-style waterfall PNG from a raw int16 IQ file. No
libpng or libz dependency: DEFLATE is stored-only and the FFT is
inlined. The operator UI invokes it via `:spectrum N` and at end of
pass when an IQ sidecar exists. You can also run it directly:

```sh
gen_waterfall <iq-path> <sample-rate-hz> [<out-png>] [options]
gen_waterfall <audio.ogg> [<out-png>] [options]
```

The PNG path is an optional positional (not a flag). Useful options:
`--fft=<n>`, `--db-min=<dB>`, `--db-max=<dB>`, `--zoom-khz=<kHz>`,
`--dc-notch`, `--detrend=<mode>`, `--pdf=<path>`. It renders the **full
capture width by default**; `--zoom-khz=<kHz>` narrows the view. The
output uses a median-subtracted noise floor and a viridis palette.

A **SatNOGS `.ogg`** is accepted directly. It is FM-demodulated audio,
not IQ, and carries its own sample rate, so there is **no rate
positional** - the PNG is positional arg 2. It's rendered as an audio
spectrogram (the real signal is fed as IQ with Q=0, so the spectrum
mirrors about DC and the FSK tones appear at +/-f). Needs libsndfile.

### `rx_replay`

Plays a previously-captured `.iq` (or `.wav` FM-demoded) sidecar
through the same RX session and decode pipeline `simple_sat_ops`
uses live. Lets you re-run the decoder with different settings
(FIR, squelch, NCO) without taking a fresh capture.

```sh
rx_replay <iq-or-wav-path> [--rate=<Hz>] [--lo-shift-khz=<N>] [--viterbi] [--tle=<path>] [...]
```

(All options are `=`-form. There is no `--gain`; shift the loaded IQ
with `--lo-shift-khz=` and set the sample rate with `--rate=`.)

The input format is picked from the extension: `.iq` is headerless
int16 I/Q (the two-pass IQ decoder); `.raw` is headerless S16_LE PCM;
`.ogg` is a SatNOGS audio recording (see below); anything else is read
as a `.wav`.

A **SatNOGS `.ogg`** recording is accepted directly - no conversion
step. It is the SatNOGS ground station's FM-demodulated *audio* (the
discriminated voltages, just Vorbis-compressed), so `rx_replay` decodes
it to PCM in memory via libsndfile and runs the **FM-audio chain** -
the same code path as a `.wav`, not the IQ path. Requires the build to
have libsndfile; otherwise an `.ogg` errors with a hint.

Mind the **format difference**. The `.iq` and `.wav` sidecars that
`simple_sat_ops` writes are at this station's post-decimation rate of
**96 kHz** (the IQ is complex baseband; the WAV is mono FM-demod
audio). A SatNOGS `.ogg` is *not* in that format: it is somebody
else's recording, Vorbis-compressed, mono, at whatever rate that
ground station used - **commonly 48 kHz, but not guaranteed**, and
never 96 kHz. So `rx_replay` reads the sample rate out of the `.ogg`
header and uses that; you do **not** pass `--rate=` for an `.ogg` (and
the IQ-only `--lo-shift-khz=` does not apply). For our own `.iq`
captures `rx_replay` instead auto-detects the rate from the companion
`.wav` header, falling back to `--rate=48000` if there isn't one - so
a bare `.iq` from this station still wants `--rate=96000` if its `.wav`
is missing.

```sh
rx_replay /FrontierSat/SatNOGS/.../satnogs_<id>_<utc>.ogg
```

See [Replaying SatNOGS recordings](#replaying-satnogs-recordings).

With `WITH_SGP4SDP4` and a TLE, `rx_replay` can also re-derive
Doppler from the recorded sidecar timestamps. The TLE-based
predictions in some older `rx_replay` versions show inflated range
numbers; for new TLE / SGP4 code, model on `apps/next_in_queue.c`
and `src/orbit/prediction.c` (the known-good path).

### `decode_inspector`

Interactive raylib-backed viewer for offline IQ. Scrub through a
capture, draw boxes around bursts, watch the
decoder run stage by stage - ASM, Golay(24,12), descrambler,
Reed-Solomon, CSP - and tune the front end with the same flags
`gen_waterfall` accepts (`--zoom-khz=`, `--detrend=`, `--dc-notch`,
`--db-min=`/`--db-max=`, `--lo-shift-khz=`, ...). A `--live` mode tails
an in-flight capture. Optional; built only when raylib is installed.

Boxes you draw are written to `<iq>.boxes.csv` and, in an
`rx_replay`-compatible form, `<iq>.box_anchors.csv` - feed the latter
back with `rx_replay --anchor-csv=<iq>.box_anchors.csv` to re-decode
exactly the slice you marked.

### Replaying SatNOGS recordings

SatNOGS publishes a pass as an `.ogg` audio file - the ground station's
FM-demodulated baseband, i.e. the discriminated voltages, Vorbis-
compressed. Because it is already demodulated audio (not IQ), you run
it through the **FM-audio** path, the same one a `.wav` uses:

```sh
rx_replay /FrontierSat/SatNOGS/<date>/satnogs_<id>_<utc>.ogg
```

`rx_replay` detects the `.ogg`, decodes it to PCM in memory via
libsndfile, reads the sample rate **out of the file header**, and runs
the decoder. Add the usual flags as needed (`--bit-rate=`,
`--window-s=`, `--tle=`, `--satellite=`, ...). Three notes:

- It is a **different format** from the `.iq` / `.wav` files
  `simple_sat_ops` records here. Ours are 96 kHz (complex baseband IQ,
  or mono FM-demod audio). A SatNOGS `.ogg` is Vorbis-compressed mono
  audio at that station's rate - **commonly 48 kHz, sometimes other
  rates, never 96 kHz**. You do **not** pass `--rate=`; `rx_replay`
  takes the rate from the `.ogg` itself.
- It is **audio, not IQ** - do not pass `--iq`, and the IQ-only knobs
  (`--lo-shift-khz=`) don't apply. The discriminated audio is what the
  decoder's FM-audio chain expects.
- A clean decode still depends on the recording: a weak or off-tune
  SatNOGS pass may yield few or no frames even though the file replays
  fine.

For a visual, render the `.ogg` as an audio spectrogram (no rate arg):

```sh
gen_waterfall /FrontierSat/SatNOGS/<date>/satnogs_<id>_<utc>.ogg wf.png
```

(`decode_inspector` is IQ-native and does not yet take `.ogg` audio.)

### `beacon_detect`

Cadence-matched beacon detector for the FM-demod baseband WAV that
every pass produces. Prints candidate timestamps. Useful for
triaging "did we hear anything?" on a noisy pass without spinning
up the full decoder.

```sh
beacon_detect <wav-path> [--threshold-sigma=<k>] [--lattice-period-s=<s>] [--min-spacing-s=<s>] [--csv=<path>]
```

### `fm_preview`

Builds an audible FM-modulated WAV from a baseband WAV. Useful for
producing a "what you'd hear on a handheld" preview from a recorded
pass. Drop the output into VLC or any media player.

```sh
fm_preview --in=<baseband.wav> --out=<output.wav> [--deviation=<Hz>] [--carrier=<Hz>] [--time-stretch=<x>]
```

The input and output are `--in=`/`--out=` flags, not positional
arguments.

### `packet_query` and `packet_browser`

`packet_query` is a CLI grep over the shared SQLite packet database
(`<root>/packet_db.sqlite`, where `<root>` is `$FRONTIERSAT_ROOT` if set
and otherwise `/FrontierSat`; override the whole path with
`$SSO_PACKET_DB` or `--db=<path>`). Filter by satellite, frame type,
time range, source tool, or capture origin. Output as a table
(default), JSON, CSV, or raw bytes via `--format=table|json|csv|raw`.
The `json`, `csv`, and `raw` forms emit the **whole** payload and
decoded summary - there is no length cap, so a large packet comes out in
full. `table` is the one-line-per-frame summary view; use the other
three when you want every byte.

`packet_browser` is an ncurses TUI over the same database. Scroll
through frames, expand to inspect the raw bytes and decoded fields,
re-run the framer with different options. The detail pane shows the
full payload as a multi-line hex dump (with a `... N more bytes` note,
pointing at `packet_query --format=raw`, if it overflows the visible
rows) and wraps each decoded-body line across as many rows as it needs,
so a long `tcmd_response` is readable rather than clipped at the right
edge.

A telecommand response is text ended by a zero byte; the decoded display
stops at that end marker, so trailing framing/parity bytes don't show up
as a garbage tail after the message. The raw byte dump still shows
everything.

Rows whose decode had trouble - Reed-Solomon uncorrectable, an HMAC
mismatch, or a CRC failure - are flagged with a `!` and shown in red.
Press `e` to toggle hiding those erroneous decodes from the list; the
top bar reads `errors=shown` or `errors=hidden`, and like the type and
origin filters it applies to the main list only (the command group and
file-reconstruction sub-views always show every packet). Cycle the type
filter with `t`, the capture-origin filter with `o`, and start a
substring search of the decoded text with `/`. (In the file-reconstruction
view `e` means something different - it exports the reconstructed bytes,
described below.)

**Press `Enter` on a `tcmd_response` to see more** - it opens a
command-group sub-view, the rest of that command's lifecycle. Each
`tcmd_response` carries the originating command's `@tssent` value
(`ts_sent`, a unix-ms integer) - the satellite echoes back exactly the
`@tssent` it received - and the sub-view is built around that `ts_sent`:

* First, every `tcmd_response` packet sharing that `ts_sent`, in
  response-sequence order - the command's acknowledgement, execution
  status, and any multi-packet text result, grouped together.
* Then, the `log` and `bulk_file` packets from the *same capture run*
  received in the window just after the command (10 minutes). This second
  section is a **time heuristic, not a firmware guarantee**: the flight
  firmware (rc3) does not stamp `bulk_file`/`log` packets with the
  command's `ts_sent`, so a download's file chunks are matched by run and
  timing, not by a confirmed link. The header labels the two counts
  ("N responses, M related (...unconfirmed)").

The main-list type/origin/search filters are ignored inside the group, so
you always see the whole lifecycle regardless of the filter you were
browsing under. `Esc` / `Left` / `Backspace` step back to the list; `r`
rebuilds the group.

The header also resolves the command **text and arguments** for that
`ts_sent`. It prefers the `sent_tcmd` table - `simple_sat_ops` records
every telecommand it transmits that carries an `@tssent` (auto-telecommand
agenda runs) into that table at transmit time. If there is no row (e.g. a
command sent before the table existed), it falls back to scanning the
agenda files under the data root for `@tssent=<value>`, and otherwise
shows `(command unknown)`. Manually-composed commands carry no `@tssent`
and so are not recorded or resolved.

**Press `Enter` on a `bulk_file` packet to reconstruct the downloaded
file.** The satellite splits a file into 195-byte chunks, each tagged with
its `file_offset`, and downlinks them at 9600 baud; `packet_browser`
reassembles them in offset order into one buffer and shows any bytes it never
received as `?`. `v` cycles the view (hex / ASCII / base64) and `e` exports
the reconstructed bytes to a file.

Because a single capture run can hold more than one download (a second file,
or the same file fetched again), the reconstruction is scoped to the **one
download "burst"** the selected chunk belongs to, not every `bulk_file` chunk
in the run. Starting from the chosen chunk it walks outward in time and keeps
a neighbouring chunk only while the gap between the two is consistent with the
firmware streaming the chunks between their `file_offset`s at about 4 packets
per second (the radio transmits each ~244-byte framed packet during the
firmware's ~208 ms inter-packet pacing, so the two overlap to ~250 ms/packet) -
with a generous margin for
missed packets and link fades, and capped at 30 s so a clearly separate
download isn't pulled in. The header reports the offset range, the bytes
recovered out of the file's reconstructed span (an exact per-file figure, not
a chunk ratio), the number of packets used, the burst's wall-clock span, how
many other `bulk_file` packets remain in the run (separate downloads), and -
when a matching `sent_tcmd` row exists - how long after the triggering
telecommand (`@tssent`) the download began. A download split
across a long pause, or across separate commands, reconstructs as separate
bursts; open each from one of its own chunks.

### `tcmd_import`

Backfill old commands into the `sent_tcmd` table. The table is
only filled for passes flown after it existed. To populate it from
history, `tcmd_import` reads the per-pass `tx.log` files (each transmitted
command is a `tx-command-sent` line whose payload is the command that went
on the air) and inserts the ones carrying an `@tssent`. With no path it
scans the Operations directory under the data root; pass one or more
directories or `tx.log` files to scope it. Re-running is safe - duplicates
are ignored.

```sh
tcmd_import                              # scan <root>/Operations recursively
tcmd_import /FrontierSat/Operations      # an explicit tree
tcmd_import --db=/tmp/test.sqlite path/to/tx.log
```

### `tcmd_browser`

The command explorer - an ncurses TUI over the `sent_tcmd` table, the
mirror of `packet_browser`. It lists the telecommands that were
transmitted, newest first, with the count of responses the satellite
returned for each (a `?` marks commands with no response yet). The detail
pane shows the full command text, `ts_sent`, `tsexec`, frequency, gain,
and source run.

**Press `Enter` on a command to see more** - it opens that command's
responses: the `tcmd_response` packets sharing its `ts_sent`, in sequence
order, with the response code and text. `Esc` / `Left` step back. `f`
cycles a response filter (all -> answered, i.e. got a response ->
unanswered) so you can show just the commands the satellite acknowledged
or just the ones still outstanding; `/` searches the command text, `l`
toggles UTC/local time.

The two browsers are inverses of each other, and in both **`Enter` is the
"show me more" key**: in `packet_browser`, `Enter` on a `tcmd_response`
opens the [command group](#packet_query-and-packet_browser) (the command's
whole lifecycle, and it resolves the command text); in `tcmd_browser`,
`Enter` on a command opens its responses. From a response you find the
command; from a command you find the responses.

```sh
tcmd_browser
```

```sh
packet_query --satellite=FrontierSat --since=1h --format=json
packet_browser
```

### `tle_keps`

A one-glance summary of a TLE's orbital elements -- the "keps" you would
read off a Keplerian element set -- and the geometry they imply. For each
object it prints the mean elements straight from the two-line set
(inclination, RAAN, eccentricity, argument of perigee, mean anomaly, mean
motion, the drag terms), then what they work out to: semi-major axis,
apogee and perigee altitude, orbital period, the J2 nodal precession rate
(flagged when it matches a sun-synchronous orbit), and the local time of
the ascending and descending node (LTAN / LTDN).

It is read-only and static: it reports the elements at their epoch, so it
needs no observer location and runs no SGP4 propagation. For the live sky
position, Doppler, and the next pass, use `simple_sat_ops` or
`next_in_queue`.

```sh
# Newest dated FrontierSat TLE, every object in it
tle_keps

# A specific file and one or more name prefixes (case-sensitive)
tle_keps $TLES/amateur.tle "ISS (ZARYA)" FrontierSat

# One comma-separated row per object, for the passes sheet
tle_keps --csv
```

With no file it reads the newest `<root>/TLEs/YYYYMMDD/tle-YYYYMMDD.tle`
(the one [`fetch_tle.sh`](#first-run-setup) writes); with no names it
summarizes every object in the file. A positional that names an existing
file is taken as the TLE file, the rest as satellite-name prefixes.

The LTAN comes from the right ascension of the ascending node and of the
sun. It uses the apparent sun, so it is local apparent solar time --
within the equation of time (under ~16 minutes) of the mean local time
LTAN usually means. The mean elements are decoded by the same library the
rest of the toolchain uses; the geometry is closed-form from the mean
motion and eccentricity, not an SGP4 fit. Apogee and perigee are heights
above the mean Earth radius (6371 km), matching common online TLE tools;
the J2 precession term keeps the equatorial radius, as that formula
expects.

### `gnss_reports`

Reassembles the satellite's GNSS telecommand responses out of the packet
DB and checks each one. FrontierSat's NovAtel receiver answers a
`gnss_send_cmd_ascii` telecommand with an ASCII log -- a `BESTXYZA`
position/velocity solution, an `ITDETECTSTATUSA` interference report, a
configuration dump, and so on -- wrapped by the firmware as `GNSS
Response (N chars): ...`. A long log is split across several
`tcmd_response` packets, so `gnss_reports` groups the fragments by their
send time, stitches them back together in sequence order, drops the
per-packet CRC trailer, and verifies the NovAtel CRC on the recovered
log.

It prints each response with its CRC verdict and a count-by-type tally.
`--type=BESTXYZA` narrows to the position fixes, `--since` / `--until`
restrict the time range (default: all), and `--full` prints the whole
recovered log rather than a snippet.

```sh
gnss_reports --summary
gnss_reports --type=BESTXYZA --full
gnss_reports --since=7d
```

A `BESTXYZA` whose status reads `SOL_COMPUTED` is a real position fix;
`INSUFFICIENT_OBS` means the receiver could not see enough satellites to
solve. Those computed fixes are what feed the orbit upload below.

## Uploading the orbit to the space safety database

Operators whose objects share an orbit regime with the Starlink
constellation upload their predicted trajectory to SpaceX's
[Space Safety](https://docs.space-safety.starlink.com/) conjunction-
screening service, so SpaceX can flag close approaches and manoeuvre a
Starlink out of the way if one is predicted. FrontierSat does this from
its own onboard GNSS fix.

Two tools cooperate. `gnss_opm` (this project) reads a GNSS fix out of
the packet DB and writes it as an **OPM** -- a single-epoch state vector
with its uncertainty. `ssm`, the separate `space_safety_manager` tool,
propagates that OPM forward, turns the uncertainty into the **OEM**
covariance the API expects, and uploads it over the authenticated link.

### The workflow

```sh
# See which fixes are good enough (newest first)
gnss_opm --list

# Write the best one as an OPM
gnss_opm > frontiersat.opm

# Preview the propagated OEM, then upload
ssm propagate frontiersat.opm | less
ssm --pretty upload-opm frontiersat.opm --type definitive
```

`gnss_opm` picks the newest fix that passes the rules below; `--id=<n>`
forces a particular one, `--since` / `--until` restrict the search, and
`--name` / `--hard-body-radius` set the object metadata. It writes the
state in metres (ECEF) plus the receiver's per-axis 1-sigma; `ssm` then
rotates that uncertainty into the orbit's radial / along-track /
cross-track (RTN) frame and grows it over the propagation window, so the
covariance reflects the real fix instead of a fixed guess.

### When a fix is good enough to upload

A trajectory upload is only as trustworthy as the fix behind it. Before
you upload, the fix should clear every one of these. `gnss_opm` enforces
the hard ones and prints the rest in the OPM header so you can check them
at a glance.

- **It is a computed solution.** The `BESTXYZA` status must be
  `SOL_COMPUTED`. An `INSUFFICIENT_OBS` fix has no usable position --
  never upload it. `gnss_opm` refuses it unless you pass
  `--allow-insufficient`, which exists only for inspection.

- **It is an autonomous fix.** The solution type is `SINGLE` -- a
  standalone GNSS point solution. FrontierSat carries no differential or
  RTK corrections, so `SINGLE` is the only type you should ever see;
  treat anything else as suspect.

- **Enough satellites.** Read the second number of the `N/M SV` field --
  the satellites *used in the solution*. Four is the mathematical floor
  for a 3-D fix and leaves no margin; require **at least 6**, and prefer
  8 or more. (The good fixes on file used 11 and 17.) Below six, wait for
  a better pass.

- **The CRC checks.** The NovAtel CRC on the log must verify -- a
  corrupted log can carry a plausible-looking but wrong state. `gnss_opm`
  checks it and refuses a bad-CRC fix.

- **The uncertainty is sane.** A healthy `SINGLE` fix reports a few
  metres of 1-sigma per axis (the file fixes are about 2.4 m). Treat a
  position sigma above ~25 m on any axis as a red flag even when the
  status says `SOL_COMPUTED`, and do not upload it.

- **The clock keeps fine time.** `time_status` reports how well the
  receiver's clock is locked to GPS time -- which sets how trustworthy the
  fix's *epoch* is, not the geometry. The whole fine-precision family is
  fine for upload: `FINESTEERING` (disciplined and steered), and equally
  `FINE`, `FINEADJUSTING`, and `FINEBACKUPSTEERING` -- anything reported as
  `FINE...`. What you do not want is `FREEWHEELING` (the clock is coasting
  on its own oscillator; NovAtel's wording is "range bias cannot be
  calculated"), `COARSE` / `COARSESTEERING` (only millisecond-level time),
  or `SATTIME` (a startup state). The reason it matters: the satellite
  moves at about 7.5 km/s, so an error in the epoch turns straight into
  along-track position error -- roughly 7.5 m for every millisecond the
  time tag is off. That error is *systematic* and is **not** captured by
  the few-metre `r_ecef_sigma_m` the receiver reports (that sigma is the
  formal solution covariance and assumes the time tag is good), so a
  non-`FINE` fix makes the uploaded covariance optimistic in exactly the
  along-track direction conjunction screening cares about. Prefer a
  `FINE...` clock; upload a `FREEWHEELING` (or coarser) fix only with
  caution -- and never paired with a stale one. `gnss_opm` prints the
  clock status in the OPM header and flags a non-`FINE` clock in `--list`.

- **The fix is fresh.** The OEM is propagated forward from the fix with
  no atmospheric drag, so error grows with the fix's age: the along-track
  error is roughly the velocity uncertainty times the elapsed time (about
  0.06 m/s, so ~0.6 km after 3 hours and ~5 km after a day) before
  unmodelled drag adds more. **Upload within about 6 hours of the fix
  epoch for definitive quality, and within 24 hours at the outside;**
  past that, fetch a fresh fix rather than upload a stale one. The API
  also requires the epoch to fall inside a 504-hour (21-day) window;
  `ssm --epoch` can shift it for staging tests.

- **One fix, ideally corroborated.** `gnss_opm` uses a single fix -- a
  snapshot of position and Doppler velocity. If several recent fixes
  agree within their sigmas, confidence is higher; if they disagree by
  more than that, investigate before uploading.

Use `--type hypothetical` for a what-if trajectory (a planned deployment
state before launch, say) and `--type definitive` for a real
GNSS-derived state once the satellite is flying.

## Bring-up and test tools

The ALSA-based TX helpers that drove the legacy transceiver
(`tx_tone`, `tx_white_noise`, `tx_frame`) now live in
[Appendix A](#appendix-a-what-didnt-stick---the-ft-991a-and-ic-9700-radio-path)
with the rest of the radio path.

### `tx_frame_sdr`

Offline IQ rendering tool for an uplink burst. Builds the CSP and
AX100 frame from a payload and modulation parameters, then dumps
the resulting IQ via `--dump-iq=<path>` (or prints sizes via
`--dry-run`). The streamer path is gone. Live RF goes through
`simple_sat_ops`'s in-process TX burst; `tx_frame_sdr` is for bench
inspection and offline tooling.

### `b210_rx_capture` and `b210_gain_sweep`

Standalone RX capture (`b210_rx_capture`) for cases where you want
to record without spinning up the operator UI. Won't share the
B210 with a running `simple_sat_ops`.

`b210_gain_sweep` steps the AD9361 RX gain across the configured
range and logs the resulting peak and RMS levels. Useful for
finding the linear regime and for verifying that the 51 Hz comb
(impulsive spikes at mid gain settings, see
`src/hw/b210_rx_tx_core.h`) is or isn't present given the AD9361
tracking-loop settings.

### `live_waterfall`

A raylib viewer that tails the operator UI's live `.iq` and renders
a scrolling viridis spectrogram in a tall narrow window. Spawn it
automatically with `simple_sat_ops --live-waterfall`, or run it
standalone against any in-flight `.iq` file. Closing its window
leaves the recording running.

### `uplink_test`

End-to-end uplink test: builds a frame with a known HMAC,
modulates it, decodes it, asserts the round-trip is byte-clean.
Catches HMAC, AX100 framing, and modem regressions. Runs in CI on
every push.

### `rx_decode`

Library-style decoder driven from the command line. Reads a binary
frame blob, runs it through ASM, Golay, descrambler, Reed-Solomon,
and HMAC verification, and prints what each stage saw. Useful for
post-hoc inspection of bytes captured outside `simple_sat_ops`'s
live path.

### `lifetime`

Toy orbit-decay estimator. Reads a TLE, extrapolates a B*-derived
decay rate, prints an estimated time on orbit. The repository
README flags this as **inaccurate**. Treat the number as
ballpark-only and don't make scheduling decisions from it.

### Amateur-band voice: `ham_listen` and `ham_speak`

FrontierSat shares 436.15 MHz with the rest of the amateur community,
and `simple_sat_ops` is not the only thing that might be on the air. Two
small tools let you be a courteous neighbour around an ops session:
listen first to make sure the frequency is clear, say out loud that
you're about to start, run the pass, and afterward say you're done. The
modulation is ordinary narrowband FM, the same as a handheld on UHF
simplex, so anyone listening on a normal radio hears you and you hear
them. Both use the system default sound device for the speakers and
microphone.

A typical session, start to finish:

```sh
ham_listen                       # is anyone using 436.15? listen, then Ctrl-C
ham_speak --allow-tx             # "VA6RAO starting satellite ops, please stand by"
simple_sat_ops --control next    # ... fly the pass ...
ham_speak --allow-tx             # "VA6RAO finished, the frequency is yours, enjoy"
```

**`ham_listen`** tunes the SDR, FM-demodulates, and pipes the audio to
your speakers until you press Ctrl-C. It runs on either backend (a
receive-only RTL-SDR is fine for listening). Useful options:
`--freq-mhz=<f>` (default 436.15), `--gain-db=<n>`, `--squelch=<level>`
to mute below a carrier-strength threshold (default off - you hear the
open-channel hiss), and `--deemphasis-hz=<hz>` for the audio
high-frequency rolloff. A periodic `level` readout on the status line
shows the received carrier strength so you can pick a squelch value.

**`ham_speak`** records your voice from the microphone, and when you
press Ctrl-C transmits the whole clip as one continuous FM burst. You
speak first; it goes out a moment later (it is not a live, keyed-up
PTT). Because it transmits, it is **TX-inhibited by default** - pass
`--allow-tx` to actually key the radio. A receive-only SDR (RTL-SDR)
can't transmit and is refused. Useful options: `--freq-mhz=<f>`,
`--tx-level=<dB>` (TX gain, default 50), `--deviation-hz=<hz>` (default
5000, standard NBFM), `--max-talk-s=<s>` (default 60), `--review` to
play the recording back and confirm before transmitting, and
`--dump-iq=<path>` to render the modulated IQ to a file instead of
keying - a no-RF dry run that works without a transmit-capable radio
(handy on a dev host).

Etiquette and the rules:

- **Identify by voice.** `ham_speak` sends no automatic station ID - say
  your callsign as your licence requires, at the start and end.
- **Don't key during a pass.** 436.15 MHz is the satellite's own carrier.
  Use these tools *around* an ops session, not during one; check
  `next_in_queue` if you're unsure when the next pass is. There is no
  software interlock - this is on you.
- **Pick the right frequency for a real chat.** 436.15 is the satellite
  simplex carrier, fine for a brief "starting / finished" courtesy call.
  For an actual conversation, move to an appropriate local simplex
  frequency with `--freq-mhz=<f>`.

## Testing and validation

There is no QA department behind this code; there is one ground
station, a handful of operators, and a spacecraft that does exactly
what the last frame told it to. A continuous-integration job now
rebuilds the tree and runs the unit tests on every push - it arrived
late, and the story of standing it up, together with an audit of
whether the tests were even worth running, is in
[Appendix C](#appendix-c-the-test-suite-audit-and-continuous-integration).
But the testing approach was never about chasing a coverage percentage.
It is about pinning, permanently, the small set of behaviours that must
never regress - that the bytes which go on the air are the bytes you
typed and *only* those, that an over-long or malformed command is
refused rather than quietly trimmed, that every uplink is signed, that
the Doppler shift is applied. A bug in any of those is not a cosmetic
defect; it is a wrong command to a satellite you cannot reach out and
fix. The tests exist so those specific mistakes can be made exactly
once.

### How the code is validated: four layers

Validation happens at four levels, each catching what the one above it
cannot.

**Unit selftests** cover the pure, deterministic cores - the parts of
the code that take bytes in and produce bytes out with no hardware in
the loop. There are 31 `*_selftest.c` binaries, and between them they
exercise the whole signal chain on the bench: the DSP blocks (`modem`,
`fir_decim`, `sw_nco`, `biquad`, `monitor_squelch`, `iq_burst`); the
framing stack the link depends on (`rs` for Reed-Solomon, `golay24`
for the length-field code, the CCSDS scrambler and `ax100` framing,
`csp`, and the HMAC trailer via `ax100`); the orbit and pass math
(`prediction`, `pursuit`, `shortarc`, `tle_csv`, `pass_schedule`,
`bestxyz`); the command path (`tcmd_lint`, `sso_pseudo`, `agenda_line`,
`tx_burst`); and the plumbing (`packet_db`, `sso_ipc_codec`,
`ipc_fill`, `argparse`). The framing and command tests are the ones
that stand between an operator's keystroke and the antenna, so they
carry the most weight.

**Build-time static analysis** catches the class of bug the everyday
compiler does not. Apple's clang accepts `-Wformat-truncation` and then
silently does nothing with it, so `snprintf`-truncation mistakes - a
buffer one byte too small, a field that quietly loses its tail - sail
through a Mac build and only surface on the Linux ground machine. The
lint script rebuilds the whole tree under real GCC with those analyses
turned into errors.

**Runtime validation gates** run inside the live program, on real
input, every time you operate. The telecommand linter checks the
`--tc-file` agenda at startup and refuses to run on errors (see
[Telecommand linting](#telecommand-linting)); the TX safety gates keep
the transmitter inhibited until you clear them (see
[TX safety gates](#tx-safety-gates)); and the framing layer itself
refuses to build a frame whose pre-Reed-Solomon payload exceeds the
223-byte block, returning an error rather than silently dropping bytes
the way the reference Python implementation does. That last refusal is
the hard backstop under the whole uplink: even if every check above it
were bypassed, an over-long command cannot reach the air malformed - it
is reported as not-sent.

**Memory-safety instrumentation** is available on demand. The selftests
build cleanly under AddressSanitizer and UndefinedBehaviorSanitizer;
compiling one with `-fsanitize=address,undefined` turns a latent
out-of-bounds read or signed overflow into an immediate, located abort.
This is worth doing whenever a test touches buffer arithmetic.

### Running the unit tests

Each `*_selftest.c` binary emits TAP (Test Anything Protocol) - one
`ok`/`not ok` line per assertion, a `1..N` plan at the end, and a
non-zero exit on any failure. Run them straight from `build/`:

```sh
build/rs_selftest
build/antenna_rotator_selftest
build/tle_csv_selftest
build/modem_iq_selftest
build/modem_fsk_selftest
build/sw_nco_selftest
build/beacon_cts1_selftest
build/csp_selftest
build/biquad_selftest
build/fir_decim_selftest
build/monitor_squelch_selftest
build/iq_burst_selftest
build/ax100_selftest
build/tx_burst_selftest
build/sso_pseudo_selftest
build/tcmd_lint_selftest
build/packet_db_selftest        # needs SQLite
build/prediction_selftest       # needs sgp4sdp4
build/pursuit_selftest          # no extra deps
build/unit_test_runner          # optional ncurses aggregator
```

(That is a representative subset; the build produces all 31.)
`unit_test_runner` discovers every `*_selftest` binary under `build/`
and renders a collapsible group view of the TAP output, so a single
launch runs the whole suite. It is skipped if ncurses is absent, in
which case run the binaries directly - any harness that understands TAP
(for example `prove`) will aggregate them.

The same binaries are wired into CTest, so `ctest --test-dir build`
runs the whole set and fails on the first non-zero exit. That is the
exact command the continuous-integration job runs on every push; the
registration is automatic, so a newly added `*_selftest` is picked up
by both CTest and CI with no extra wiring. The narrative of how that
job came to exist - and the audit that preceded it - is
[Appendix C](#appendix-c-the-test-suite-audit-and-continuous-integration).

### Catching what the compiler hides

The lint script `scripts/lint_warnings.sh` rebuilds the whole tree
under gcc-15 with `-Wformat-truncation=2 -Werror=format-truncation`
(and the matching format-overflow analysis) to catch the truncation
bugs Apple clang misses. Run it on every nontrivial source change:

```sh
bash scripts/lint_warnings.sh           # full reconfigure + build
bash scripts/lint_warnings.sh --quick   # reuse the build-lint cache
```

It uses a separate `build-lint/` directory so it never disturbs your
working build, and exits non-zero (dumping the offending warning) if
anything would complain on the ground machine.

### Decode smoketest: known-good recordings

The unit selftests exercise the framing and DSP blocks in isolation, but
they do not prove that a real recording still decodes once those blocks
are wired together into `rx_replay`. `scripts/decode_regression.sh`
closes that gap. It pushes two committed snippets through the live
decoder - a 48 kHz mono WAV the way SatNOGS dumps it
(`test/decode_regression/audio_snippet.wav`) and a 96 kHz headerless IQ
capture from a local pass (`test/decode_regression/iq_snippet.iq`),
each known to carry one beacon frame - and checks that the
`rx_replay` decode summary still reports the same counts:

```sh
bash scripts/decode_regression.sh            # compare against the baseline
bash scripts/decode_regression.sh --update   # re-pin after a known-good change
```

It pins the integer counts from the summary block - frames detected,
valid CSP headers, Reed-Solomon corrected/uncorrectable, and the
recognized-by-type breakdown - rather than a hash of the demodulated
bytes. At the low SNR of these clips the beacon payloads are
Reed-Solomon-uncorrectable, so the raw bytes wobble with sub-bit
floating-point differences between machines while the counts hold steady;
a count that moves is a real regression in the decode chain. Run it
before and after any change to `modem`, `modem_fsk`, `decode_loop`, or
`ax100`; it exits non-zero with a unified diff if anything drifts. The
baseline lives in `scripts/decode_regression.expected`, and the
fixtures' provenance is in `test/decode_regression/README.md`.

### Validation that runs on every pass

Two of the four layers are not something you invoke - they run whether
you think about them or not, and they are what makes the uplink safe to
operate. The startup telecommand lint reads your `--tc-file`, parses
each line the same way the flight firmware does, and refuses to fully
start if any line is malformed or over the radio length limit, unless
you override with `--ignore-at-your-peril-all-tc-errors` (the name is
the warning). It also refuses on a brick-risk `danger:` finding, behind
a second, deliberately separate override
`--ignore-at-your-peril-dangerous-tcmds`, so accepting a typo never also
accepts a boot-loop. The framing layer's 223-byte refusal then sits underneath
the auto-telecommand and compose paths as a last line of defence. The
practical consequence: a command that is too long, mistyped, or
unsigned does not become a corrupted transmission - it becomes a
visible "not sent" in the TX log, and the satellite hears nothing.

### Every bug leaves a test behind

The standing rule is that a bug worth fixing is worth a test that would
have caught it, written so it fails on the old code and passes on the
fix. The point is not the one bug; it is that the same mistake cannot
return unnoticed.

The TX command-history display is the worked example. It mis-rendered
the transmitted command twice - once truncating it mid-string, once
printing a stray extra character past its end - because the summary
formatter assumed a terminator the payload buffer never carried. Both
were display-only (the framing always used the real byte count, so
nothing extra ever went on the air), but a wrong readout of what was
just sent is its own hazard. The fix was a one-line correction; the
durable part is that the formatter is now exercised directly by
`tx_burst_selftest`, with cases for the truncation, the stray byte
(a sentinel planted just past the command that must never appear in the
output), empty and hex payloads, and the real send path. Those
assertions were confirmed to fail on the pre-fix code before the fix
landed - a test that cannot fail proves nothing.

### What isn't covered

The honest limits are worth stating. There is no automated
hardware-in-the-loop test: the live RF path needs a B210 and a radio,
so it cannot run on a build server, and the end-to-end uplink is
validated by hand against the bench tools instead - `tx_frame_sdr
--dump-iq` to inspect exactly what a frame would modulate to,
`rx_replay` and `decode_inspector` to push recorded passes back through
the decoder with different settings (see
[Offline analysis tools](#offline-analysis-tools)). The downlink half
is better off: the decode smoketest above pins the recorded-recording
path, so a regression there is caught on the bench even though a live
pass is not. The orbital
propagator is the vendored sgp4sdp4, which traces to 2001 and has not
been re-checked against the Vallado et al. 2006 revisions, so its
accuracy claims are qualified accordingly. And coverage is deliberately
uneven: the framing, command, and orbit cores are tested hard because a
mistake there is expensive, while the ncurses rendering and the
hardware drivers - where a mistake is visible and recoverable in the
moment - lean on the operator's eyes and a teammate in viewer mode.

## Architecture notes

### IPC: one operator at a time

A satellite admits one hand at a time. Two operators keying the same
rotator and radio would be a brief and expensive comedy, so the
software elects a single operator and makes everyone else a spectator.

The operator UI binds a Unix-domain socket at
`/run/sso/simple_sat_ops.sock` and writes its PID to
`/run/sso/simple_sat_ops.pid`. Viewers (and helpers like
`tx_frame_sdr`'s operator mode) connect, send a `hello`, and read
back a `welcome` that carries the operator's Unix user and pass
folder. Helpers verify the user matches their own `$USER` before
proceeding. This stops `bob`'s `tx_frame_sdr` from accidentally
posting events to `alice`'s operator session.

`simple_sat_ops --control` refuses to start when another operator
is already running. It probes first (as a transient viewer
connection), then prints the live operator's user and PID and
points at the force-claim path:

```
simple_sat_ops: --control refused: operator already running as user=alice pid=12345.
  To take over, run a viewer (no --control) and press
  'c' then 'y' to force-claim; the running operator
  will yield and your viewer will re-exec into --control.
```

The force-claim sends `SIGUSR1` to the running operator (which
treats it as a clean-exit signal), waits up to 5 s for the socket
to disappear, then `execv`s itself into `--control` with the same
TLE and pass folder. No hardware race.

If the bind fails with no operator detected by the probe, the most
likely cause is a stale socket and pid file from a crashed previous
operator. The error message tells you to remove
`/run/sso/simple_sat_ops.{sock,pid}` and retry.

### Worker threads

The main loop is cooperative single-threaded for the ncurses
surface but pushes three blocking-I/O domains onto dedicated
pthreads:

* **RX session worker** (`src/pipeline/rx_session.c`). Owns the
  B210 / UHD pump and the WAV / IQ writers, and receives continuously
  for the whole pass. The main loop submits TX bursts asynchronously via
  `rx_session_submit_burst` and `rx_session_poll_burst`; each burst runs
  on its own thread and brings the transmit chain up and back down
  *without* stopping, retuning, or reclocking receive - so the USB link
  keeps draining and the radio's buffer can't overflow during a burst
  (an earlier stop-the-world transmit flooded the log right when the
  reply was due). Powering the transmit chain down between bursts still
  guarantees nothing is left on the air while receiving.

* **Rotator worker** (`src/hw/antenna_rotator_async.c`). Polls the
  SPID STATUS at 2 Hz and drains a latest-wins SET slot. The UI
  reads a snapshot non-blockingly. A 500 ms VTIME hang from an
  unplugged cable no longer freezes the operator panel.

* **Audit-log writer** (`src/ipc/sso_audit.c`). A 256-slot ring
  buffer drained by a writer thread that opens, flocks, writes, and
  closes `runs.log` per event. The producer side
  (`sso_audit_event`) is non-blocking and stays sub-microsecond per
  call.

The spectrum-render path (`:spectrum N`) is also threaded: an
on-demand worker forks `gen_waterfall` and waits for completion
without blocking the UI.

### TX safety gates

All TX-capable tools default to TX-inhibited so refactors and
bring-up runs can't accidentally key the PA. Three gates, all
opt-in:

| Gate | Required when |
|------|----------------|
| `--allow-tx` | Any PTT-on or burst-commit. Without it the tool configures the radio (frequency, mode, MOD source, power) but stops at PTT with a clear stderr message. |
| `--allow-high-power` | `--tx-power` above 10%. |
| `--allow-hf-tx` | PTT below 100 MHz (HF). |

PTT-off is always passed through, even with the inhibit set;
releasing TX is never blocked.

In `simple_sat_ops` the same three gates appear as checkboxes on
the TX compose modal and the auto-tcmd modal. The values are
broadcast to viewers in the `tx-preview` event so a watching
operator can see them before commit.

### Pursuit planner internals

`src/orbit/pursuit.c` is the pure-function planner. Inputs:

* pass window `[jul_aos, jul_los]`,
* rotator slew rates `r_az_dps`, `r_el_dps`,
* antenna bounds (`ANTENNA_ROTATOR_MINIMUM_*`, `MAXIMUM_*`),
* antenna's starting position `(a0_unwrapped, e0)`,
* a `pursuit_sat_sample_fn_t` callback that returns the satellite's
  unwrapped mech-frame `(az, el)` at an arbitrary Julian date.

The planner discretises the pass into waypoints at
`waypoint_dt_s = 5 s` and the cost grid at `dense_dt_s = 1 s`.
Initial waypoints seed at the satellite's predicted positions; the
first waypoint is pinned to `(a0, e0)`. Iteration is alternating
forward and backward sweeps. For each interior waypoint the
planner projects the unconstrained optimum (`sat_pos(T_k)`) into
the intersection of the two rate-feasible boxes implied by its
neighbors. When the intersection is empty (the sat is moving
faster than the rotator can keep up), it falls back to the
forward-feasible box only and aims its boundary closest to the next
waypoint. Iteration stops once the cost stops improving by
`cost_improvement_eps = 0.5 %` or after `max_iter = 6` (clamped at
16).

The result is a rate-feasible trajectory whose waypoints are
already in unwrapped mech coords. `pursuit_aim_at(plan, jul_utc)`
returns the *next* waypoint after `jul_utc`. That's where the
operator UI tells the rotator to go, and the constant-rate slew
between waypoints interpolates the segment for free.

The selftest at `unit_tests/pursuit_selftest.c` covers 38 cases:
linear within rate, linear above rate, an apex pass (which beats a
naive trail-the-sat baseline), az wrap, boundary clamps,
degenerate inputs (NULL ptrs, empty window, zero rate), the
iteration cap, plan-lookup window, and the rate-file I/O
round-trip (with HOME redirected to a tempdir so the user's saved
values are not disturbed).

## File layout

| Path | What's there |
|------|---------------|
| `/FrontierSat/` | Top-level data root. Default when `$FRONTIERSAT_ROOT` is unset; set `$FRONTIERSAT_ROOT` to use a different tree (e.g. on a dev host). If the default root doesn't exist, every tool prints a one-time notice with the options (create it, or set `$FRONTIERSAT_ROOT`). |
| `/<satname>/TLEs/` (e.g. `/FrontierSat/TLEs/`, `$TLES`) | Dated TLE files, conventionally `<date>/tle-<date>.tle`, populated daily by `fetch_tle.sh` (see [First-run setup](#first-run-setup)). `simple_sat_ops --control` with no `<satellite_id>`, and `next_in_queue <satellite_name>`, both load the newest dated `*.tle` found here (searched recursively). Note: an explicit `--tle <path>` is **not** resolved under this directory - it is taken relative to the working directory (a CSV TLE is auto-converted to a temp `.tle`). |
| `/FrontierSat/Operations/<yyyymmdd>/<HHMMLT>/` | Per-pass folder. Holds the pass's WAV, IQ, decoded frames, TLE snapshot, doppler / lo_offset / burst sidecars, and the end-of-pass spectrogram and waterfall PNG. |
| `/FrontierSat/Operations/current` | Symlink the operator UI keeps pointing at the most recent pass folder. |
| `/FrontierSat/captures/` | One-off `b210_rx_capture` outputs. |
| `/FrontierSat/Testing/` | Bench captures and analysis. |
| `~/.local/state/simple_sat_ops/active.tle` | Default TLE file (the `--tle` default). |
| `~/.local/share/simple_sat_ops/rotator_az_rate_dps` | Calibrated rotator azimuth slew rate (deg/s). |
| `~/.local/share/simple_sat_ops/rotator_el_rate_dps` | Calibrated rotator elevation slew rate (deg/s). |
| `~/.local/share/simple_sat_ops/carrier-trim-hz` | Per-host carrier-trim offset that lands the B210's analog LO on the requested frequency. |
| `/FrontierSat/packet_db.sqlite` | Shared SQLite packet database: received packets plus a `sent_tcmd` table of transmitted telecommands (keyed by `@tssent` for response correlation). Under `$FRONTIERSAT_ROOT` when set; override the path with `$SSO_PACKET_DB` or `--db=`. |
| `~/.local/share/simple_sat_ops/runs.log` | Audit log (dev-host fallback when `/var/log/sso/` isn't writable). |
| `/run/sso/simple_sat_ops.sock` | Operator IPC socket. |
| `/run/sso/simple_sat_ops.pid` | Operator PID. |
| `/var/log/sso/runs.log` | Audit log (production path). |

## Troubleshooting

Before guessing, three tools narrow almost anything down:

- **`simple_sat_ops --self-test`** prints the fully resolved
  configuration and exits without touching hardware: the active mode,
  the SDR backend and TLE, the HMAC keyfile path *and whether it
  loaded*, Doppler/frequency settings, and the rotator/observer config.
  A missing file or a flag that didn't take shows up here - including
  the exact path a keyfile or TLE was expected at.
- **`sdr_probe`** (see [SDR backends](#sdr-backends)) confirms which SDR
  is attached, which FPGA image will load, and which ports are RX vs TX.
- **The TX log panel** (and the `tx.log` in the pass folder) record one
  line per composed command with the outcome, and - when a burst didn't
  reach the air - the reason. Most TX problems are answered right there.

The audit log and the stderr printed at startup are the next places to
look. Specific symptoms:

**`simple_sat_ops: --control refused: operator already running as user=alice pid=12345`**

Another operator is already there. Confirm with `ps -p 12345 -o
user,pid,cmd`. If you legitimately need to take over, run a viewer
(`simple_sat_ops` with no `--control`) and press `c` then `y`.

**`simple_sat_ops: --control: socket bind failed. If this is from a crashed previous operator, remove /run/sso/simple_sat_ops.{sock,pid} and retry.`**

The probe saw no operator but the bind still failed. Almost always
a stale socket and pid file from a crashed previous operator.
Confirm with `ls -la /run/sso/simple_sat_ops.*`, then `rm` both
files and retry.

**`pursuit: no calibration on disk; run \`simple_sat_ops --calibrate-rotator --confirm-rotator-calibrate\` to enable lead-aim`**

The pursuit planner needs rotator slew rates to plan a pass
trajectory. Without them, tracking falls back to today's
aim-where-sat-is-now logic. That works fine; it's just slightly
less accurate at apex on high-elevation passes. Run calibration
once to enable the planner.

**`pursuit: plan max_err=N.N deg > 30; disabled`**

The planner produced a plan whose worst-case pointing error exceeds
$30^\circ$. Most often this means the calibrated rates are way off (the
SPID can't actually keep up with the predicted angular velocity).
Re-run `--calibrate-rotator`. Tracking falls back to the aim-now
logic for this pass.

**`Warning: could not read SPID position; check that the Rot2ProG is in 'A' mode`**

The async rotator worker didn't see an OK STATUS reply in the
1500 ms startup window. Check the SPID front panel; it should read
something like `A 0000 0000` (mode A = remote control).
Power-cycle the SPID if it's stuck. With the cable unplugged or
the controller off, the operator UI still starts; it just renders
`?` on the rotator panel and the track loop won't drive the
antenna.

**`(BAD - see --self-test)` or `(MISSING)` on the HMAC banner (or `hmac: ... status=bad` in `--self-test`)**

The HMAC key signs every uplink, so a key problem means the next TX
request is refused before the PA is keyed. It does *not* affect
receiving - the downlink is verified by its CSP CRC32, not the HMAC -
so you can keep tracking and decoding while you sort the key out.

- **`(MISSING)`** - no keyfile was found at any of the three
  locations. Create one (see the keyfile step under
  [First-run setup](#first-run-setup)) or point the run at an
  existing key with `--hmac-keyfile <path>`.
- **`(BAD)`** - a keyfile was found but rejected. The exact reason is
  printed once at startup, before the UI paints over it, so the
  reliable way to read it is the dry run, which never takes the
  screen:

  ```sh
  simple_sat_ops <tle> <satellite> --self-test
  ```

  Look for the `hmac_keyfile:` line in that output. The two common
  causes:

  - **Permissions or a stray ACL** - "`must be chmod 0600 or 0640 (got
    0NNN)`". Repair it in place:

    ```sh
    uplink_test --fix-permissions          # default keyfile path
    uplink_test --fix-permissions --keyfile=/FrontierSat/HMAC/frontiersat_hmac
    ```

    That strips any ACL (`setfacl -b`) - which can grant access the
    plain mode bits don't show - and then `chmod`s to `0600`, or
    `0640` for the shared keyfile. (Fixing the shared key under
    `/FrontierSat` needs write permission on it, so run it as the
    owner/admin.)
  - **Malformed contents** - "`non-hex or lowercase char ...`",
    "`... hex chars (must be even)`", or "`is empty`". The key must be
    plain uppercase hex with no spaces or separators; recreate it per
    the keyfile setup step.

A quick existence-and-length check, independent of all the above:

```sh
ls -la /FrontierSat/HMAC/frontiersat_hmac
ls -la "$HOME/.local/state/simple_sat_ops/frontiersat_hmac"
wc -c "$HOME/.local/state/simple_sat_ops/frontiersat_hmac"
```

**TX-inhibit messages**

The TX compose modal won't commit until the `allow-tx` checkbox is
ticked (and `allow-high-power` if `--tx-power > 10%`, and
`allow-hf-tx` below 100 MHz). The auto-tcmd modal works the same
way. Toggle them once per pass.

**TX composed but nothing transmits (no TRX LED, no RF)**

The PA only keys for the brief moment a burst is on the air, and a
burst that's refused never keys at all - so this is almost always a
refused burst, not a hardware fault. Read the TX log line for that
command (or `tx.log`): `sent>` means it went out (the LED flash is just
short - raise the repeat count to see it); `notsent>` / `err>` carry the
reason. The usual refusals:

- **Missing/invalid HMAC key** - CTS1 requires every uplink to be
  signed, so an absent or bad keyfile blocks *every* burst. The banner
  reads `(MISSING)` / `(BAD)`, and `--self-test` prints the exact path
  it looked for on the `hmac:` line. Put the key there, or pass
  `--hmac-keyfile <path>`.
- **`allow-tx` not ticked** in the compose modal (it resets each pass).
- **`--no-tx`** (preview only) or **`--tx-dry-run`** (composes but never
  keys) on the command line - both show in `--self-test`.
- An **RX-only SDR** (RTL-SDR): transmit is unavailable by design; the
  RX panel shows `(RX-only)`.

If the log says `sent>` yet there is genuinely no RF or TRX LED on a
B210 clone, that points at the clone's FPGA not driving the transmit
ATR/PA the way stock UHD expects - re-check the
[clone FPGA setup](#sdr-backends).

**RX shows only noise / nothing decodes**

- Confirm the antenna is on the port the software receives on - for a
  B210 that is the `RX2` antenna of channel 0 (a clone may silk-screen
  it differently; `sdr_probe` names the ports).
- Check the status panel's carrier/Doppler is near the satellite's
  downlink, and the predictions panel shows the pass is actually
  overhead.
- Tune RX gain with `:gain <dB>` while watching the IQ peak/RMS on the
  RX panel - too low buries the signal, too high clips it.
- Render the last N seconds with `:spectrum <sec>` to see whether the
  carrier is even present before chasing the decoder.

**SDR unplugged mid-pass / `Abort trap: 6`**

Pulling the SDR's USB cable (or losing its power) while receiving raises
a fatal fault inside the radio library that can't be caught and resumed
from within the process. `simple_sat_ops` no longer dies raw and leaves
a wrecked terminal: it restores the terminal directly, closes the
waterfall window, prints one line about what happened and where the log
is, and exits cleanly. (If two threads hit the fault at once, only the
first runs the cleanup; the second waits for it to finish, so the
message prints and the screen is restored reliably.) Ctrl-C and
termination signals also quit through this same clean shutdown. If a
device error ever surfaces as an ordinary error rather than an abort,
the receive worker parks, the RX panel shows
`SDR DISCONNECTED - RX stopped`, and the rest of the program keeps
running. Fully *surviving* an unplug (rather than exiting cleanly) would
need the SDR in a separate process; that is designed and parked, not yet
built.

**`simple_sat_ops: data root /FrontierSat does not exist`**

The shared data root isn't present on this host. Tools still resolve to
`/FrontierSat` (the default when `$FRONTIERSAT_ROOT` is unset), so output
would fail to land there. Either create it
(`sudo mkdir -p /FrontierSat && sudo chown "$(whoami)" /FrontierSat`) or
point the tools at an existing tree with
`export FRONTIERSAT_ROOT=/path/to/your/tree`. For the packet database
specifically you can instead pass `--db=<path>` or set `$SSO_PACKET_DB`.

**`audit-overflow dropped=N`**

The audit-log writer thread fell behind the producer. Almost always
means another process holds an exclusive flock on `runs.log` for an
extended period (10s+). Check for stuck CI or other tooling
appending to the same file. The overflow line is informational;
in-flight events were dropped, not the persisted log.

## A note on feel

Everything in this manual can be read in an afternoon. The part that
takes longer, and that no manual can hand you, is feel: the operator's
growing sense of when a pass is going well and when it is quietly going
wrong. You start to read the Doppler drift without thinking about it.
You notice when the frame counter stalls a beat too long. You learn
the particular hesitation the rotator makes when it is about to lose
the satellite over the back of a high pass, and you reach for the stop
key before the panel has caught up.

This comes from passes, not pages. Run them as a viewer until the
rhythm is familiar, then run them with the controls. Keep the
captures; a pass that decoded badly is the best teacher you have.
Before long the software fades into the background, which is exactly
what good tools are supposed to do, and you are simply talking to a
satellite.

## Glossary

| Term | Meaning |
|------|---------|
| **AOS / LOS** | Acquisition of Signal / Loss of Signal. Pass start and end times. |
| **AX100** | The frame format used on FrontierSat's UHF link: 32-bit ASM, Golay-coded length, CCSDS scrambling, Reed-Solomon FEC. |
| **B210** | Ettus USRP B210 software-defined radio. Two-channel, 70 MHz to 6 GHz; receives on `RX2` and transmits on `TX/RX`, with receive running continuously through a transmit burst. |
| **CAT** | Computer-Aided Transceiver. The serial control protocol on the FT-991A (see [Appendix A](#appendix-a-what-didnt-stick---the-ft-991a-and-ic-9700-radio-path)). |
| **CSP** | Cubesat Space Protocol. The frame inside the AX100 wrapper. |
| **CTS-SAT-1 / FrontierSat** | Calgary-to-Space's first satellite. Often called "FrontierSat" in this codebase. |
| **Doppler** | The frequency shift from line-of-sight velocity. Computed each tick from the SGP4 range-rate. |
| **Flip mode** | Tracking mode for high-elevation passes ($>75^\circ$ peak). The boom swings through zenith on the back hemisphere instead of slewing $180^\circ$ in azimuth at apex. |
| **HMAC** | Hash-based Message Authentication Code. Every uplink frame carries one; the satellite drops unsigned frames. |
| **Pass folder** | `/FrontierSat/Operations/<yyyymmdd>/<HHMMLT>/`. Holds every artifact from one pass. |
| **PCM shadow** | The FM-demod-side decoder that runs in parallel to the live IQ chain. Counts frames only; doesn't drive the DB. |
| **Pursuit** | The whole-pass antenna trajectory planner that aims slightly ahead of the satellite so the rotator's constant-rate slew tracks the curve. |
| **Rot2ProG** | The SPID rotator controller. Talks W-frame BCD-ish over USB-serial. |
| **SPID** | The rotator manufacturer. |
| **SGP4 / SDP4** | The standard NORAD orbit propagators. SDP4 is the deep-space variant. |
| **TC (telecommand)** | A single uplink command, e.g. `CTS1+fs_list_directory_json(/,0,20)@tsexec=1779961244000!`. |
| **TLE** | Two-Line Element set, the standard format for NORAD orbital elements. We use the 3-line variant (name plus two element lines). |
| **TX compose modal** | The `t`-key modal in `simple_sat_ops` for hand-composing an uplink burst. |
| **Viewer** | A read-only `simple_sat_ops` instance connected to a running operator via IPC. |

---

## Appendix A: What didn't stick - the FT-991A and IC-9700 radio path

Every project has a road not taken, and this is ours. Before the B210
became the live RF path, the plan was the obvious one: drive a real
amateur transceiver. The code still builds, the tools below still run,
and on a good day you can key a frame through one of these radios. We
do not operate this way anymore, and it is worth saying plainly why,
because the reasons are instructive rather than embarrassing.

We started with an Icom IC-9700 and a used SignaLink into the Linux
box. The IC-9700 is advertised as having a "satellite mode," and for
working FM voice repeaters through a bird that is exactly true. But
its bandwidth for 9600-baud MSK data simply was not there. We worked
through every configuration the radio exposes - every menu, every
audio routing, every filter setting - short of opening the case to
look for capacitors to swap, and never got a clean data path out of
it.

So we moved to the Yaesu FT-991A, which became the default backend in
the code (the IC-9700 stayed as a secondary one, `radio-type=icom-civ`).
The Yaesu did better: moderate success on receive, enough that you
could *hear* the satellite's beacon bursts come up out of the noise as
clear pulses of static. But we could almost never decode them - the
frames came through corrupted nearly every time. We tried the obvious
fixes, then the unobvious ones, and eventually found ourselves
searching the forums for *hidden* FT-991A configuration settings.
That is usually the sign that you are asking a piece of equipment to
do something it was never built to do.

And that is the real lesson of this appendix. These are excellent
radios for what they are designed for: lower data rates, comfortable
signal-to-noise, or simple FM voice through a satellite repeater. A
9600-baud MSK cubesat downlink at the edge of the link budget is none
of those things. The B210 succeeded where both transceivers were
marginal because it puts the whole signal chain - filtering, timing,
demodulation - under software we control, instead of behind a front
panel and a fixed IF.

The FT-991A's specific frustrations on the data path are worth
recording, because they are concrete and they recur on any voice
radio pressed into data service:

* **The USB CODEC is low-pass filtered.** Tones up to about 12 kHz
  pass cleanly, but the 9600 GFSK pre-emphasis above roughly 5 kHz is
  attenuated, which rounds off exactly the edges the modem cares about.
  The intended cure is to route audio through the rear DATA jack with a
  SignaLink instead of the radio's own USB audio. That works, and it is
  also one more box, one more cable, and one more thing to get wrong at
  3 a.m.
* **The radio rewrites your settings behind your back.** The FT-991A
  force-resets Menu 072 (DATA PORT SELECT) to USB every single time it
  enters DATA-FM, regardless of what you wrote over CAT a moment
  earlier. `radio_yaesu_cat.c` deliberately does not fight this; the
  menu has to be pinned on the front panel by hand. This is the radio
  quirk promised in the foreword, and it is the kind that no amount of
  software will fix for you.
* **Half-duplex by hand.** Keying TX, releasing it, and getting back to
  RX cleanly meant choreographing CAT, audio, and PTT against a radio
  that has its own ideas about timing.

The B210 dissolved all three at once, and brought one quirk of its own
(an analog TX-LO that leaks if the transmit chain idles hot, see
`src/hw/b210_rx_tx_core.h`) - but that one we can hold inside our own
code rather than behind a front panel.

The radio path is preserved, not deleted: it is the right tool for
bench bring-up of a conventional station, and if a future ground
station needs a hardware transceiver the scaffolding is here. What
follows is how to use it.

### `radio_ctl`

CAT-control verification tool for the FT-991A. Most-used subcommands:

```sh
radio_ctl --radio-device=/dev/ttyUSB1 --radio-serial-speed=38400 \
          --store-device --store-serial-speed identify

radio_ctl set-freq 145.900e6
radio_ctl set-mode FM
radio_ctl set-mod-input acc      # rear DATA jack (SignaLink), default
radio_ctl set-mod-input usb      # rear USB CODEC
radio_ctl set-power 10
radio_ctl ptt on --allow-tx
radio_ctl ptt off
radio_ctl power on
radio_ctl power off
radio_ctl uplink-prep            # tune, FM mode, MOD source, DATA mode
```

`--store-device` and `--store-serial-speed` persist the values to
`~/.local/share/simple_sat_ops/radio_device` and `radio_serial_speed`.
Subsequent unadorned invocations pick them up.

The IC-9700 backend (`radio_icom_civ.c`) speaks binary CI-V with
BCD-packed payloads instead of the FT-991A's ASCII CAT, but presents
the identical command set to `radio_ctl`. Select it with
`--radio-type=icom-civ`; the FT-991A is `--radio-type=yaesu-cat`
(default).

**Safety.** TX-capable subcommands honour the same three TX safety
gates the operator UI does (see [TX safety gates](#tx-safety-gates)).
`ptt on` does nothing without `--allow-tx`; `ptt off` is always
allowed.

### FT-991A front-panel checklist

A handful of menu items must be set once on the radio's front panel.
They persist across power cycles, and several of them cannot be driven
reliably over CAT (Menu 072 is the one the radio keeps resetting, as
above). The authoritative list lives in the project root `CLAUDE.md`;
in brief:

| Menu | Setting | Why |
|------|---------|-----|
| 031 CAT RATE | match `--radio-serial-speed` (38400 is comfortable) | CAT link baud. |
| 033 CAT RTS | `DISABLE` | The USB-serial driver can't toggle RTS reliably; with it enabled the radio drops every CAT command. |
| 070 DATA IN SELECT | `REAR` | Pinned over CAT, but set it here too. |
| 071 DATA PTT SELECT | `DAKY` | Gates the rear-DATA audio path open under CAT keying. |
| 072 DATA PORT SELECT | `DATA` | The radio force-resets this to USB on entry to DATA-FM; the software does not fight it, so you must pin it here. |
| 079 PKT MODE | `9600` | The uplink baud. |

### ALSA TX helpers: `tx_tone`, `tx_white_noise`, `tx_frame`

ALSA-based TX helpers from the legacy transceiver path. `tx_tone` keys
the radio and plays a sine (or stepped sweep) through the modulator,
which is how the USB-CODEC bandwidth limit above was first measured.
`tx_white_noise` is the same idea with full-band noise for combined
TX/RX bandshape characterisation. `tx_frame` is a single-shot CSP and
AX100 frame transmitter from the command line. All three honour the TX
safety gates and use `radio_ctl`'s persistent device and baud.

### Persistent files

| Path | What's there |
|------|---------------|
| `~/.local/share/simple_sat_ops/radio_device` | Persisted transceiver tty path. |
| `~/.local/share/simple_sat_ops/radio_serial_speed` | Persisted transceiver baud. |

---

## Appendix B: Trial and error - the git repository

This whole project was built by trial and error, and the git history
is the record of it: not a tidy after-the-fact narrative but the
actual logbook of what was tried, what broke, and what fixed it. That
makes the repository unusually worth reading - and unusually hard to
walk into cold, because the one thing it will not tell you on its own
is where to start. This appendix is that starting point.

**The headline: it is all on `main` now.** For most of this project's
life `main` was a frozen, published baseline - parked at one reviewed
merge, `1dd038a` (2026-05-02, "multi-radio: pluggable backends, FT-991A
canonical, RX/TX tooling (#3)") - while the live ground station grew on
a long-lived development line off to the side. That split is over. The
development line has been fast-forwarded onto `main` and the side
branches deleted, so `main` now carries the full history this manual
describes. Clone the repo, build `main`, and you have the SDR ground
station - there is nothing else to check out. The radio-first design
from
[Appendix A](#appendix-a-what-didnt-stick---the-ft-991a-and-ic-9700-radio-path)
is still in there, as the early stretch of the same history, not a
different branch.

### The line of development

The SDR ground station grew commit by commit into one long, nearly
*linear* chain: very little rebasing and, for most of its length, no
merges - just one small change after another. (The one exception is the
last stretch, which folds the refactoring work in through a single
merge commit, `e0bab3c`; everything else is a straight line.) That
chain is now the bulk of `main`'s history. Read end to end it is the
development history of the SDR ground station, in six phases. Each phase
was written on its own branch at the time - all since folded into `main`
and deleted - so the names below are descriptions, not branches you can
check out. The commit ranges are `git log` arguments you can paste
directly.

| Phase | Commit range | Dates | What it introduced |
|-------|--------------|-------|--------------------|
| Multi-operator / SDR RF path | `1dd038a..afe29d4` (~107) | 05-02 to 05-15 | The B210 becomes the live RF path. SDR RX/TX chain, the `rx_session` decode loop, the shared packet database and `packet_browser`, the SatNOGS audio pull, and the SatNOGS-style IQ waterfall. |
| Coherent IQ demod | `afe29d4..394b7c7` (~66) | 05-15 to 05-19 | A shadow IQ-domain demodulator running alongside the FM path for A/B sensitivity, coherent demod work, and a long run of format-truncation fixes. |
| Live graphics | `394b7c7..00f6ba6` (~67) | 05-19 to 05-24 | raylib graphics: the `live_waterfall` real-time spectrogram and the `iq_annotator` waveform panel (phase and split-channel views), plus `--tx-dry-run`. |
| Decode inspector | `00f6ba6..92f428a` (~91) | 05-24 to 05-29 | `decode_inspector` (the renamed `iq_annotator`): a staged decoder visualizer walking ASM, Golay, descrambler, Reed-Solomon, and CSP, with a Viterbi slicer and a `--live` mode. Then AD9361 tracking-loop defaults, the T/R switch driver (`14b8d23`), the B210 TX-LO-leak fix (`86c5c6d`, power the TX chain down between bursts), `agenda_check --tle`/`--prune-dups`, `next_in_queue --tle`, the async threading pass (TX, rotator, and audit log on worker threads), elevation jog keys, rotator slew-rate calibration with the pursuit planner, the one-operator `--control` refusal, and the Mission Operations ICD docs. |
| Refactoring / code-sharing | `92f428a..6bf39b5` (~144) | 05-29 to 06-16 | Mostly `move`/`extract` commits paying down duplication: one shared ISO-UTC timestamp formatter, shared modal text-field editing across the two editors, one ASM finder across the four demods, shared GNSS fragment helpers, `live_waterfall` linked against `waterfall_core`. Plus hardening: validate numeric CLI options instead of silently accepting garbage, bound CLI-driven size math, and the first removal of dead RX-side HMAC plumbing. |
| SDR-only cutover and hardening | `6bf39b5..a970b01` (~87) | 06-16 to 06-23 | Drop HMAC from the live and offline decoders and always validate the downlink CSP CRC32 instead (`cfb4c1f`, `0206aa8`, `ebdad4b`); pin the uplink frame to the pycsplink reference (`e214aa7`); serialize UHD device access between the RX and TX threads and close the cross-thread races (`01b6116`, `3252f68`). Then the GNSS->OPM space-safety upload pipeline (`gnss_opm`, `gnss_reports`, short-arc OPM determination), the `ham_listen`/`ham_speak` amateur-band voice tools (with CW mode and Ogg/Vorbis streaming for remote monitoring), live receiver audio relayed to viewers, pausable auto-command runs, the test-suite audit + continuous integration of [Appendix C](#appendix-c-the-test-suite-audit-and-continuous-integration), and the round of new selftests that followed it. This phase folds in the refactoring work via the one merge commit on the line. |

To walk any phase:

```sh
git log --oneline 1dd038a..afe29d4          # the multi-operator / SDR RF phase
git log -p --reverse 6bf39b5..HEAD -- apps/main.c   # the latest phase, one file
```

### How the commits are meant to be read

The commit history is not bookkeeping; it is the troubleshooting
record. Development happened across two machines. Why a Mac dev host,
you ask? No grand reason - it happened to be Burchill's work laptop,
and it had Claude Code installed, so that is where the code got
written. It builds the non-hardware tools (no ALSA, no SGP4SDP4, no
UHD by default); the actual radio, rotator, and antenna live on the
RAO ground station. So the loop was: write and commit on the Mac,
push, pull on RAO, test against the hardware, and watch for errors,
limitations, or features worth adding - then back to the Mac to do it
again. The commits are
therefore deliberately small, one observable change each, so that a
regression can be cornered with `git bisect` and a behavior can be
explained with `git log -p`. A run of forty near-identical
`decode_inspector:` commits is not noise; it is one debugging session,
written down. When you want to know *why* a line is the way it is, the
commit message that introduced it is usually the answer:

```sh
git log -p -S'the string or symbol you are chasing' main
git bisect start HEAD 1dd038a       # good=baseline, find the change
```

For most of the project's life `main` advanced only by reviewed pull
request, squash-merged, which is why the multi-radio baseline near the
bottom of its history is a single squashed commit (`1dd038a`, "(#3)")
rather than a record of the branch behind it. The development line
caught up with `main` only once it was complete; from here the same
review discipline applies to new work landing on `main`.

If you are new and reading this: clone the repo, build `main`
per [Build and install](#build-and-install), and run a pass as a
viewer per the drill in
[Modes: operator vs. viewer](#modes-operator-vs-viewer). The phase map
above is here for the day you need to know why something changed, which
on a system debugged one commit at a time is a day that comes often.

---

## Appendix C: The test-suite audit and continuous integration

[Section 14](#testing-and-validation) describes what the tests check and
how to run them. This appendix is the other half of the story: how we
came to *trust* the tests, and how they came to run on their own. It is
written for transparency - the suite was largely machine-written, and a
suite you do not understand the weaknesses of is a suite you should not
lean on.

### The question: are machine-written C tests worth anything?

Most of this code, and most of its tests, were written with an AI
assistant. There is a widely repeated claim about that arrangement:
that large language models write *plausible but meaningless* tests for C
- assertions that read well, that pass, and that would never have caught
the bug they purport to guard against. Tautological checks
(`x == x` dressed up), round-trips that only prove a function is the
inverse of itself, loose tolerances that swallow real errors, "golden"
values copied out of the very code under test. A green suite full of
those is worse than no suite, because it buys false confidence about a
link to a satellite you cannot recall.

Rather than argue the point, we audited it. The method had to be
empirical, because reading a test and judging it "looks fine" is exactly
the failure mode under suspicion.

### How the audit was done: mutation testing

The audit ran several independent reviewers in parallel, each taking a
slice of the suite, and each held to one discipline: **mutation
testing**. For a test to earn its place it is not enough that it passes;
it must *fail* when the code it covers is deliberately broken. So for
each block of behaviour the reviewer introduced a small, realistic bug
into the source - flip a comparison, swap two struct fields, drop a
byte, widen a window - rebuilt, and watched whether some assertion went
red. A mutation that survives a green suite is a hole in the suite,
named and located. A mutation that is caught is a test doing its job,
proven rather than asserted.

Around two dozen such mutations were injected across the framing, orbit,
DSP, database, and command modules. All but a few were caught
immediately. The headline finding was therefore the opposite of the
claim: **the suite was, in the main, effective** - these were not
decorative tests. But the survivors were not random, and they pointed at
a specific, instructive weakness.

### What the survivors had in common: symmetric bugs

Every mutation that slipped through shared one shape. It was a
*symmetric* error in a piece of code tested only by round-trip - encode
then decode, write then read, frame then unframe. When both directions
run the same code, a matching pair of mistakes cancels: the AX100
scrambler with one wrong table byte still descrambles its own output;
an HMAC with the wrong truncation still matches the trailer it just
produced. The round-trip stays green while the bytes that would go on
the air are wrong. A round-trip proves a function is consistent with
itself; it cannot prove the function is *correct*. For anything that has
to interoperate with the spacecraft, that gap matters, because the
spacecraft does not run our encoder.

The audit also turned up coverage simply missing rather than weak: the
Golay length code was exercised only through the framing that wraps it,
never directly; the rotator's two-step homing logic - which once wound
the dish to 360 deg because it trusted the first status reply after a
move - had no test for the echo it must reject; the packet database had
never been hit by two writers at once, the way two replay jobs sharing
one file would; and the orbit propagator's topocentric output was
checked only for *plausible ranges*, not against an independent
calculation, so a swap of azimuth for elevation or range for range-rate
would have sat inside the bands.

And above all of it: **there was no continuous integration.** The
selftests existed, but nothing ran them unless an operator remembered
to. The only aggregator was an interactive ncurses runner - useless on a
build server. Tests that no one runs are tests that rot.

### What got fixed

Each finding became a concrete change, and each change was held to the
same mutation standard before it was committed - shown to fail on the
old or broken code first, the way the TX command-history fix was in
[Section 14](#every-bug-leaves-a-test-behind). A test that cannot fail
proves nothing.

- **External oracles where round-trips were blind.** The CCSDS scrambler
  is now checked against the randomizer sequence generated independently
  from its shift-register polynomial (and the published CCSDS opening
  bytes), all 255 bytes of it; a single wrong table byte now fails even
  though the round-trip still passes. The HMAC trailer is pinned to
  values computed separately with the `openssl` command line, for fixed
  key/data pairs, with the derivation in a comment so anyone can redo
  it; a wrong truncation or key step now fails even though determinism,
  key-sensitivity, and the round-trip all still pass.
- **The orbit propagator gained an independent oracle.** The prediction
  selftest now re-propagates the satellite, rebuilds the observer
  position, redoes the south-east-zenith projection by hand, and compares
  azimuth/elevation/range/range-rate to what the code reported - on both
  the near-Earth (SGP4) and deep-space (SDP4) branches - and exercises
  the pass-search interface end to end. A field swap now reddens the
  oracle on both branches.
- **The missing direct tests were written:** Golay(24,12) on its own
  (weight-enumerator and correction-radius checks against the code's
  external definition), the rotator's status-echo rejection as a pure
  predicate with its own test, and a parallel-writer test for the packet
  database.
- **The parallel-writer test found a real, latent bug.** With four
  writers on one database file, one of the concurrent opens could fail
  and silently lose its rows, because the busy-timeout was being set
  *after* the WAL-mode pragma rather than before it. The fix was to set
  the timeout first; the test now passes repeatably and would have
  caught the original.

### The continuous-integration job

The orphaning was closed by wiring the selftests into CTest (see
[Running the unit tests](#running-the-unit-tests)) and adding a GitHub
Actions workflow, `.github/workflows/unit-tests.yml`, that runs them on
every push and pull request to `main`. The job is deliberately lean: it installs only the handful of
libraries the selftests actually need (a toolchain, OpenSSL, SQLite,
ncurses headers), turns the SDR and voice backends off at configure time
so it never probes for UHD or ALSA on the runner, builds the aggregate
`selftests` target, and runs `ctest`. New selftests are picked up
automatically, so the workflow file itself rarely changes.

One wrinkle is worth recording, because it will bite anyone who forks
the build. The orbit math depends on the bundled `sgp4sdp4` library,
which is a separate CMake project with no install rule and was
originally kept out of version control. CI cannot build what is not
committed, so the library - a small C port of the NORAD SGP4/SDP4
models, carried with its original attribution and licensing intact - is
now tracked in the repository, and the workflow builds and caches it
before configuring the main tree. Only its local build directory is
ignored.

### What this appendix does not claim

The audit raised the floor; it did not make the suite complete.
Mutation testing samples; it does not enumerate every possible bug, and
a reviewer can only mutate code they thought to look at. The
limits in [What isn't covered](#what-isnt-covered) still stand - no
hardware-in-the-loop on the build server, an orbit propagator whose
absolute accuracy is unverified against modern references, and thin
coverage of the UI and drivers by deliberate choice. What changed is
narrower and worth stating plainly: the tests that exist have been shown
to fail when the code they cover is broken, the ones that were blind to
symmetric errors now have outside references, and the whole set runs on
its own on every push. That is the claim, and no more than it.
