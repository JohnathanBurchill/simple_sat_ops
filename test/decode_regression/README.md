# Decode regression fixtures

Two tiny captures pinned into the repo so `scripts/decode_regression.sh`
can replay them deterministically across refactors of the FSK / AX100
chain. Each contains exactly one beacon frame.

| Fixture                | Bytes | Source                                                                     | Path through rx_replay                |
|------------------------|-------|----------------------------------------------------------------------------|---------------------------------------|
| `iq_snippet.iq`        | 768 K | 2-second window from a local 96 kHz IQ capture, around a known beacon     | `chain=iq+slicer+anchored-fsk`        |
| `audio_snippet.wav`    | 288 K | 3-second 48 kHz mono WAV transcoded from a SatNOGS .ogg observation       | `chain=fm-audio`                      |

The IQ slice exercises `modem_fsk.c` (the file `iq_inspector` is built
around); the WAV slice exercises `modem.c`'s FM-audio path. Together
they catch the two main flavours of accidental decoder regression.

Run the regression with `scripts/decode_regression.sh`. The expected
hashes live in `scripts/decode_regression.expected`; refresh them with
`scripts/decode_regression.sh --update` only after you've manually
verified the new output is what you want.

## Replacing or extending the fixtures

If you want to swap in a fresh capture, the recipe is:

```bash
# IQ: pick a file where rx_replay decodes ≥1 frame at known t=Tsec, then
# carve out a window of ~2 s of samples (96 kHz IQ = 384 kB/s, so 2 s
# is ~768 kB).
dd if=<source.iq> of=test/decode_regression/iq_snippet.iq \
   bs=384000 skip=$((T-1)) count=2

# WAV: SatNOGS observations are .ogg; ffmpeg both extracts the slice and
# transcodes to the 48 kHz mono WAV rx_replay's FM-audio path expects.
ffmpeg -ss $((T-1)) -t 3 -i <source.ogg> \
       -ar 48000 -ac 1 -f wav test/decode_regression/audio_snippet.wav

# Verify each new snippet decodes ≥1 frame:
build/rx_replay test/decode_regression/iq_snippet.iq --no-db --rate=96000
build/rx_replay test/decode_regression/audio_snippet.wav --no-db --channels=1

# Refresh the expected hashes:
scripts/decode_regression.sh --update
```
