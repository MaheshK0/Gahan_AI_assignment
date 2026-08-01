# Design Notes

## Task 1 — Streaming UART Protocol Parser

### Architecture

Explicit state machine (`WaitSof1 → WaitSof2 → Length → Type → Payload →
CrcLo → CrcHi`), implemented in `UartParser::feed(byte)`. One byte in, at
most one decoded packet out (via callback), per call — this is what makes
it a true streaming parser: it never needs to see more than the current
byte to make progress, and it never buffers more than one in-flight
packet's payload (bounded at 255 bytes, the maximum a single Length byte
can express, stored in a fixed `std::array`). There is no dynamic
allocation anywhere on the feed() hot path.

No global/shared mutable state: everything lives in the `UartParser`
instance, so multiple parsers (e.g. one per UART peripheral) can run
independently and even concurrently without synchronization, as long as
each instance is only touched from one thread at a time.

### Finding: the CRC coverage in the written spec does not match the sample data

The assignment text states: *"The CRC covers Length + Type + Payload."*
I verified this against `uart_stream.bin` before writing any parsing code
(brute-forcing every contiguous byte range around each packet's trailing
two bytes against CRC-16/CCITT-FALSE) and found:

- No packet in the sample validates with `Length + Type + Payload` as the
  CRC input, under either byte order.
- Every structurally well-formed packet **does** validate if the CRC
  input is `Type + Payload` (Length excluded), with the two CRC bytes
  transmitted **little-endian** (low byte first).

Example: the first packet is `AA 55 03 01 10 20 30 A2 81`. CRC-16/CCITT-FALSE
over `01 10 20 30` (Type + Payload) is `0x81A2`; the wire bytes are `A2
81` — i.e. `0x81A2` read low-byte-first. CRC over `03 01 10 20 30`
(Length + Type + Payload, as the spec states) is `0xFABC`, which appears
nowhere in the stream.

This was cross-checked against every packet in the file, not just the
first one (see the exploration notes in project history) — the parser is
implemented against the empirically-verified framing, with this
discrepancy called out explicitly in `include/uart_parser.hpp` so it
isn't mistaken for a bug.

### Statistics semantics

- `valid_packets` / `crc_failures` — self-explanatory.
- `sync_losses` — incremented when an *in-flight* packet (past SOF1+SOF2)
  is abandoned due to a CRC failure. Distinct from `discarded_bytes`,
  which counts bytes skipped while still hunting for the next SOF pair
  (i.e., bytes that were never part of any packet attempt at all).
- `partial_packets` — incremented by `finish()` if the stream ends
  mid-packet.

One important characteristic of this implementation is that it operates as a true single-pass streaming parser. As each incoming byte is processed, it becomes part of the current parsing sequence and is not reconsidered as the beginning of a new packet, even if the current packet later turns out to be invalid. This behavior reflects the operation of real UART communication systems, where received bytes are processed sequentially without the ability to rewind or reprocess previously received data. Concretely, `uart_stream.bin` contains a corrupted packet at
offset `0x20` whose bogus payload happens to contain the byte sequence
`AA 55` (a fake SOF). A buffering/backtracking parser could try
re-interpreting starting from that inner `AA 55`; this parser does not —
it treats those bytes as already consumed by the outer (failed) packet
attempt and resumes SOF-hunting only after them. This is the correct,
realistic behavior for byte-at-a-time streaming and is why
`crc_failures` is 4, not 5, if you naively scan the file for every
`AA 55` occurrence and treat each as an independent attempt.

## Task 2 — Multi-Sensor Data Aggregator

### Architecture

One `SensorReplayProducer` thread per CSV (GPS, IMU, Encoder) — genuinely
independent producers, each with its own timeline, per the assignment
note that these should be treated as independent producers rather than a
single merged stream. Each producer writes into a single shared
`VehicleStateManager`, which is the only piece of shared mutable state in
the system, protected by one `std::mutex`. A separate printer/consumer
thread polls `VehicleStateManager::snapshot()` on a fixed cadence and
prints the current state + staleness flags.

**Why a single mutex is fine here:** update rates are in the tens of Hz
(IMU at 100Hz is the fastest), so lock contention is a non-issue, and a
single lock means there is exactly one lock in the whole component — no
lock-ordering to reason about, no possibility of deadlock between
producers.

**No busy-waiting:** producers sleep via
`std::condition_variable::wait_until(lock, target, stopPredicate)` against
a shared replay-start time, rather than `sleep_for` in a loop. This means
threads are fully descheduled between events and — importantly — can be
woken immediately by `requestStop()` rather than having to finish a sleep
first, which is what makes prompt, graceful shutdown possible. The
printer thread uses a plain periodic `sleep_for`, which is standard and
non-busy (the thread blocks in the kernel).

**Graceful shutdown:** every producer's `run()` loop checks an atomic
`stop_requested_` flag before *and* is interruptible *during* each sleep;
`main()` calls `requestStop()` + `join()` on all three producers
uniformly, whether they finished naturally or were stopped early.

### Replay timing model assumption

Default replay speed is **1.0x (real time)**. This is a deliberate choice,
not just the path of least resistance: the staleness thresholds given in
the spec (GPS >500ms, IMU >50ms, Encoder >100ms) are wall-clock values. If
the replay ran at an arbitrary accelerated speed by default, those
thresholds would need to be scaled too (e.g. 500ms becomes meaningless at
50x speed, since the *entire* 8-second GPS log would blow past it), which
the spec doesn't ask for. A `--speed` flag is provided for convenience
(e.g. for the stall-injection bonus demo, where waiting out a full 8
seconds isn't necessary to see the effect), but real-time is the default
and the mode in which the staleness thresholds mean what they say.

### Bonus: stall injection

`SensorReplayProducer::injectStall(afterMs, duration)` makes a producer
go quiet for `duration` once its replay reaches source-timestamp
`afterMs`, without emitting any updates during that window — this
directly exercises `VehicleStateManager`'s staleness detector rather than
faking a "stale" flag. After the stall, the producer re-anchors its
internal clock so it resumes at the correct relative pace rather than
firing a burst of "overdue" events to catch up.

## Task 3 — CAN Decoder & Vehicle State

### Architecture

- `JsonValue` (`json_value.hpp/.cpp`) — a small hand-written recursive-descent
  JSON parser, just capable enough for `dbc.json`'s schema (objects,
  strings, numbers, nested objects).
- `DbcDatabase` (`dbc_database.hpp/.cpp`) — loads `dbc.json` into
  `MessageDef`/`SignalDef` structs (id, name, `period_ms`, byte order,
  signals with start bit / length / scale / offset / sign / optional enum
  table).
- `SignalCodec` (`signal_codec.hpp/.cpp`) — pure, stateless bit-extraction
  and physical-value decoding functions (Intel and Motorola, signed and
  unsigned). No dependency on the rest of the system, which made it easy
  to unit test in isolation against hand-built byte arrays.
- `FrameLogReader` (`frame_log_reader.hpp/.cpp`) — parses `frames.log`'s
  three-line-per-frame format into `RawFrame`s.
- `DiagnosticsEngine` (`diagnostics.hpp/.cpp`) — counter-gap, checksum, and
  timeout detection, plus rolling per-message stats and unknown-ID
  handling. Framework-free: takes a `DbcDatabase` reference and is fed one
  frame at a time.
- `VehicleStateUpdater` (`vehicle_state.hpp/.cpp`) — maps a decoded,
  DBC-known frame onto the flat `CanVehicleState` struct (RPM, Speed,
  Gear, Brake Status, Steering Angle).
- `main.cpp` — a single-threaded replay driver that ties the above
  together, prints per-frame status, and prints a final summary.

Unlike Task 2, nothing in the spec requires concurrent producers for this
task — `frames.log` is a single interleaved trace, so it's decoded and
applied in timestamp order on one thread. Nothing above is inherently
single-threaded, though: `SignalCodec` is pure/stateless and
`DiagnosticsEngine`'s per-ID maps could be sharded or mutex-protected with
no interface change if a future live source (e.g. SocketCAN) needed a
dedicated reader thread.

### Using `dbc.json` over `vehicle.dbc`

The assignment says both files describe the same messages/signals and
only one needs to be used. `dbc.json` was chosen because its `start_bit`
values are plain, already-normalized bit indices rather than the
DBC-text-format's own big-endian bit-numbering convention (`7|16@0-` for
a Motorola signal) — using it sidesteps re-implementing that convention
from scratch just to end up at the same bit layout. `vehicle.dbc` was
still valuable: its `CM_` comment on the checksum signals is what
motivated the checksum assumption below, and its `[min|max]` ranges (e.g.
steering angle `[-780, 780]` degrees) were used as an independent sanity
check on the decoded values.

### Finding: `dbc.json`'s bit-numbering conventions, verified empirically

Rather than assume a convention, I brute-force-verified the bit layout
directly against `uart_stream.bin`'s CAN counterpart, `frames.log`, the
same way Task 1's CRC coverage was verified (see above) — by checking
which extraction scheme reproduces values that are internally consistent
(sensible physical ranges, and — decisively — a checksum that validates):

- **Intel (`byte_order: "intel"`):** `start_bit` is the signal's
  least-significant bit, in linear `byte*8 + bit` numbering counting from
  bit 0 = LSB of byte 0. Bits are read `start_bit, start_bit+1, ...` and
  accumulated LSB-first. This is the conventional DBC Intel scheme and
  matched immediately (confirmed via `EngineData.rpm` and, decisively, via
  the `VehicleSpeed` counter-gap analysis below lining up frame-for-frame).
- **Motorola (`byte_order: "motorola"`):** `start_bit` is the signal's
  most-significant bit, but in `dbc.json` it's expressed as a plain
  sequential bit index counting from bit 0 = MSB of byte 0, increasing
  through the byte array in normal reading order (bits 0–7 = byte 0
  MSB→LSB, bits 8–15 = byte 1 MSB→LSB, ...) — i.e. the whole 8-byte
  payload treated as one big-endian bit stream, with `length` bits read
  starting at `start_bit` and accumulated MSB-first.

  I confirmed this by hand-decoding `SteeringData`'s first frame
  (`FF 8B 00 74 00 00 00 00`) three different ways and checking which one
  produced self-consistent results: with `start_bit=0, length=16` under
  the scheme above, `steering_angle` = bytes `FF 8B` read as a 16-bit
  big-endian value (`0xFF8B` = **−117** signed) × scale `0.1` = **−11.7°**
  — comfortably inside the DBC's stated `[-780, 780]` range. The 4-bit
  `counter` (`start_bit=16`) then lands exactly on the top nibble of byte
  2, and — most convincingly — the 8-bit `checksum` (`start_bit=24`)
  lands exactly on byte 3, whose value (`0x74`) equals the XOR of the
  other 7 bytes of that same frame (see below). All three signals only
  land on clean, byte/nibble-aligned boundaries under this one
  interpretation, which is strong evidence it's the intended convention
  rather than a coincidence.

  This convention is documented in `signal_codec.hpp` and implemented in
  `SignalCodec::extractRaw()`.

### Checksum validation assumption

`vehicle.dbc` documents the checksum via a comment on each `checksum`
signal: *"XOR of the other 7 bytes in the frame."* This was verified,
not taken on faith: computing the XOR of the 7 non-checksum bytes for
every frame in `frames.log` and comparing against the transmitted
checksum byte matches on every frame **except one** — exactly the signal
you'd want from a real checksum validator (see "Injected faults" below).

`DiagnosticsEngine::onKnownFrame()` implements this generically: the
checksum byte's index is derived from the signal's own `start_bit / 8`
(every checksum signal in this DBC happens to be a single, byte-aligned
8-bit field, confirmed across all four messages), not hard-coded per
message — this matters because the byte position differs: byte 3 for
`EngineData`/`VehicleSpeed`/`SteeringData`, but byte 1 for `BrakeStatus`
(whose `brake_pressed` + `counter` signals only occupy the first 8 bits).

### Timeout thresholds, derived from `period_ms`

Each message's timeout threshold is **`period_ms × 3`**
(`DiagnosticsEngine::kTimeoutMultiplier`), taken directly from `dbc.json`'s
`period_ms` field per the assignment's instruction to use it as the
baseline rather than picking an arbitrary value. Rationale for the 3×
multiplier: it comfortably tolerates the replay loop's own ~20ms poll
granularity plus a missed frame or two without false-positiving, while
still catching the deliberately injected multi-frame gap (see below) in
well under half the gap's duration. For `SteeringData` (10ms period)
that's a 30ms threshold; for the three 20ms messages, 60ms.

### Injected faults found in `frames.log`

Found by exhaustive cross-checking against the decoded signals (not just
spot-checked on the first few frames) before the diagnostics code
existed, then re-confirmed by that same code once written:

1. **Counter discontinuity** on `VehicleSpeed` (`0x220`) at t=500ms: the
   4-bit rolling counter jumps from 8 to 11 (expected 9).
2. **Checksum mismatch** on `EngineData` (`0x180`) at t=600ms: computed
   XOR is `0x20`, transmitted checksum is `0xDF`.
3. **Extended timing gap** on `SteeringData` (`0x2B0`): a 310ms gap
   between t=390ms and t=700ms against its normal 10ms period — 31× its
   nominal period, and well past the 30ms timeout threshold. This gap is
   *timing-only*: the counter resumes at the correct next value
   afterwards (no discontinuity), so it's cleanly distinguishable from
   fault #1 by which detector fires.

All three are caught by the demo: faults 1 and 2 print as `!!` diagnostic
lines the moment their frame is processed (`t=500ms` and `t=600ms`
respectively); fault 3 is caught proactively — the poll loop notices the
gap and prints a `TIMEOUT` line at `t≈439ms`, well before the next real
`SteeringData` frame arrives at `t=700ms`, and the continuous status line
reports `WARNING: SteeringData missing` for the duration.

### Unknown CAN IDs

`DiagnosticsEngine::onUnknownFrame()` is called whenever `DbcDatabase::find()`
returns null for a frame's CAN ID: it counts the frame and returns a
diagnostic `Anomaly` without touching `CanVehicleState` or crashing — the
demo prints an `??  UNKNOWN` line and moves on. The provided `frames.log`
doesn't contain any unknown IDs (verified: only `0x180`, `0x1A0`, `0x220`,
`0x2B0` appear), but this path is covered directly by a unit test
(`test_diagnostics_unknown_id_does_not_crash`) rather than only by
inspection.
