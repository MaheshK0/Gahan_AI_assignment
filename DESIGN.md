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

One subtlety worth calling out for the interview discussion: because this
is a genuine single-pass streaming parser, once a byte has been consumed
as part of a (possibly-failing) packet attempt, it cannot be
retroactively reinterpreted as the start of a different packet — a real
UART receiver has no way to "rewind" bytes it has already shifted in
either. Concretely, `uart_stream.bin` contains a corrupted packet at
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

`DbcDatabase` (message/signal definitions) + `can_decoder.{hpp,cpp}` (pure
bit-extraction and physical-value decode functions, no state) +
`VehicleStateManager` (owns the live `VehicleState` plus per-message
`MessageHealth` diagnostics) + `FrameLogReader` (parses `frames.log`) +
a single-threaded replay driver in `main.cpp`.

Unlike Task 2, nothing in the spec requires concurrent producers for this
task — `frames.log` is one interleaved trace, decoded and applied in
timestamp order on a single thread. `VehicleStateManager` still uses a
mutex internally (negligible cost at this frame rate) so it would need no
changes to support a future live/multi-threaded source (e.g. SocketCAN).

### Finding: `dbc.json` and `vehicle.dbc` use different `start_bit` conventions

The assignment states `dbc.json` and `vehicle.dbc` "describe the exact
same set of messages and signals" and that only one needs to be used. I
initially reached for `dbc.json` (simpler to parse), but before trusting
its `start_bit` values for Motorola (big-endian) signals, I verified them
against `frames.log` the same way as Task 1 — and found `dbc.json`'s
`start_bit` for `SteeringData.steering_angle` (`0`) does **not** match the
standard Vector/DBC big-endian bit-numbering convention used by
`vehicle.dbc`'s own text (`7|16@0-`), even though both files describe the
same physical bit layout.

Rather than reverse-engineer a second, undocumented convention, this
project uses **`vehicle.dbc`** as the sole decode source of truth,
via a small hand-written parser (`dbc_text.cpp`) for the `BO_`/`SG_`/
`CM_ BO_`/`VAL_` lines it needs. (`dbc.json` is still supported —
`DbcDatabase::loadFromJson()` exists and produces the same `MessageDef`
shape — but the demo/tests use the DBC-text loader.)

**Bit extraction, as implemented and verified:**
- **Intel (little-endian, `@1`):** `start_bit` is the signal's LSB
  position in linear `byte*8 + bit` numbering; bits are read from
  `start_bit` upward, least-significant-first.
- **Motorola (big-endian, `@0`):** `start_bit` is the signal's MSB
  position, where `byte_idx = start_bit / 8` and `bit_in_byte = start_bit
  % 8` (7 = MSB of that byte); bits are read from there towards the LSB,
  wrapping to bit 7 of the next byte when a byte boundary is crossed.

Verification method: decoded `steering_angle` for every `SteeringData`
frame in `frames.log` and confirmed every value falls inside the DBC's
own stated range (`[-780, 780]` degrees) — a wrong bit-numbering
convention produces values wildly outside that range on most frames, so
this is a strong correctness signal, not a coincidence. The same
extraction was then independently corroborated by the counter and
checksum checks below all agreeing frame-for-frame.

### Checksum validation assumption

`vehicle.dbc` documents the checksum via a `CM_ SG_` comment: *"XOR of
the other 7 bytes in the frame."* I did not take this on faith either —
I computed, for every message and every frame in `frames.log`, the XOR
of the 7 non-checksum bytes and compared it to the transmitted checksum
byte. Result: it matches on every frame **except one** (see "Injected
faults" below), which is exactly the signal you'd want from a real
checksum validator. `computeXorChecksum()` implements this generically:
it XORs all 8 bytes except whichever byte the message's `checksum` signal
occupies (looked up from the DBC, not hard-coded per message — this
matters because the checksum byte's position differs per message:
byte 3 for `EngineData`/`VehicleSpeed`/`SteeringData`, but byte 1 for
`BrakeStatus`).

### Timeout thresholds, derived from `period_ms`

Each message's timeout threshold is **2.5 × its `period_ms`** (parsed
from the `CM_ BO_` comment text, e.g. *"transmitted every 20 ms"*, via a
small regex — `vehicle.dbc`'s text format doesn't carry a structured
period field the way `dbc.json` does). Rationale: 2.5x tolerates one
missed frame plus normal scheduling jitter without a false positive,
while still catching two or more consecutive misses quickly. For
`SteeringData` (10ms period) that's a 25ms threshold; for the three 20ms
messages, 50ms.

### Injected faults found in `frames.log`

Found by exhaustive cross-checking against the DBC (not just spot-checked
on the first few frames), before writing the diagnostics code that now
detects them automatically:

1. **Counter discontinuity** on `VehicleSpeed` (`0x220`) at t=500ms: the
   4-bit rolling counter jumps from 8 to 11 (expected 9).
2. **Checksum mismatch** on `EngineData` (`0x180`) at t=600ms: computed
   XOR is `0x20`, transmitted checksum is `0xDF`.
3. **Extended timing gap** on `SteeringData` (`0x2B0`): a 310ms gap
   between t=390ms and t=700ms, against its normal 10ms period — over 12x
   its nominal period and well past the 25ms timeout threshold. Note this
   gap is *timing-only*: the counter resumes at the correct next value
   afterwards (no discontinuity), so it's cleanly distinguishable from
   fault #1 by which detector fires.

All three are caught by the demo in real time: faults 1 and 2 appear
immediately as `!!` diagnostic lines when their frame is processed; fault
3 causes `overallHealth()` to report `WARNING: SteeringData timeout` for
the duration of the gap (confirmed by running the demo at real-time
speed and observing the warning appear at t≈420ms and clear once frames
resume).

### Unknown CAN IDs

`VehicleStateManager::applyFrame()` looks up the frame's CAN ID in the
`DbcDatabase` first; if absent, `noteUnknownId()` records it (counted,
logged as a fault line) and the function returns without touching
`VehicleState` or any `MessageHealth` entry — no crash, no exception,
just a diagnostic. (The provided `frames.log` doesn't happen to contain
any unknown IDs, but this path is exercised by inspection/code review and
would be straightforward to add a synthetic-frame unit test for if
desired.)
