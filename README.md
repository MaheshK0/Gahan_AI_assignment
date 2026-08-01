# GAHAN AI — Embedded Technical Assignment

C++17 solutions for all three tasks: a streaming UART protocol parser, a
multi-threaded sensor aggregator, and a CAN frame decoder with vehicle
state + diagnostics.

See [`DESIGN.md`](DESIGN.md) for architecture notes, threading models, and
the assumptions made for each task (including two data-format
discrepancies discovered by cross-checking the written spec against the
provided sample data).

## Demo videos

| Task | Description | Video |
|------|--------------|-------|
| Task 1 — UART Parser | Decodes `uart_stream.bin`, prints packets + parser stats | _add link_ |
| Task 2 — Sensor Aggregator | Live GPS/IMU/Encoder replay with staleness detection | _add link_ |
| Task 3 — CAN Decoder | Replays `frames.log`, flags all 3 injected faults live | _add link_ |

**How to add these:** record a short terminal session of each task's demo
(the exact commands are in each task's own README, linked above), then
either:

1. **Easiest — drag-and-drop into GitHub:** open this README for editing
   directly on github.com (pencil icon), then drag your `.mp4`/`.mov`
   file into the edit box. GitHub uploads it to its asset CDN and inserts
   a working embed link automatically — replace the `_add link_`
   placeholders above with what it generates. This only works through the
   web UI, not a plain `git push` of a video file.
2. **Hosted elsewhere:** upload to YouTube (unlisted is fine), Loom, or
   similar, and just paste the link in place of `_add link_` above.
3. **Terminal recording instead of screen capture:** tools like
   [`asciinema`](https://asciinema.org/) or
   [`vhs`](https://github.com/charmbracelet/vhs) record your terminal
   session as a small, scrubbable recording (asciinema hosts it for you
   with one command: `asciinema rec`, then `asciinema upload`) — often
   nicer than a screen-recorded video for a CLI demo like these three.

Each task's own `README.md` also has a placeholder "Demo video" section
at the bottom if you'd rather link the recording there instead of (or in
addition to) this table.

## Repository layout

```
task1_uart_parser/       Streaming UART packet parser + CRC-16 validation
task2_sensor_aggregator/ Multi-threaded GPS/IMU/Encoder replay + VehicleState
task3_can_decoder/       CAN frame decoder (DBC-driven) + VehicleState + diagnostics
DESIGN.md                Architecture, assumptions, and empirical findings (all 3 tasks)
```

Each task folder is self-contained: its own `CMakeLists.txt`, `include/`,
`src/`, `tests/` (where applicable), a copy of its reference `data/`, and
its own `README.md` with task-specific build/run instructions and design
highlights:

- [`task1_uart_parser/README.md`](task1_uart_parser/README.md)
- [`task2_sensor_aggregator/README.md`](task2_sensor_aggregator/README.md)
- [`task3_can_decoder/README.md`](task3_can_decoder/README.md)

This root README covers prerequisites and the quickest path to building
and running all three; `DESIGN.md` is the single place with the full
architecture writeup and the empirical findings (CRC framing, bit-layout
conventions, checksum algorithm, injected faults) for all three tasks
together.

## Prerequisites (Ubuntu 22.04 or 24.04)

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git
```

Requires a C++17 compiler (GCC 9+; verified on Ubuntu 24.04 running GCC
13.3) and CMake 3.16+. No third-party libraries are used anywhere in the
codebase beyond the C++ standard library and `pthread` (via
`Threads::Threads`, needed only by Task 2 for its producer threads —
Task 1 and Task 3 are single-threaded).

## Task 1 — Streaming UART Protocol Parser

```bash
cd task1_uart_parser
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

./uart_parser_tests                        # unit tests (30 checks)
./uart_parser_demo ../data/uart_stream.bin  # decode the sample stream + print stats
```

## Task 2 — Multi-Sensor Data Aggregator

```bash
cd task2_sensor_aggregator
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Real-time replay (~8s), prints VehicleState + staleness every 200ms
./sensor_aggregator_demo --data-dir ../data

# Bonus: inject a 3s stall on the IMU stream starting at source-time 2000ms,
# and export the synchronized state stream to CSV
./sensor_aggregator_demo --data-dir ../data --speed 20 \
    --stall imu --stall-after 2000 --stall-duration 3000 \
    --export /tmp/vehicle_state.csv
```

Run `./sensor_aggregator_demo --help` for all options.

## Task 3 — CAN Decoder & Vehicle State

```bash
cd task3_can_decoder
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

./can_decoder_tests                          # unit tests (9 checks)
./can_decoder_demo --data-dir ../data         # real-time replay (~1s)
./can_decoder_demo --data-dir ../data --speed 50   # accelerated replay
./can_decoder_demo --data-dir ../data --summary-only  # just the anomaly lines + final summary
```

This prints per-frame decoded vehicle state (RPM / Speed / Gear / Brake /
Steering) with a continuous OK/WARNING health line, flags the three
faults deliberately injected into `frames.log` as they occur (a counter
discontinuity, a checksum mismatch, and an extended timing gap — see
`DESIGN.md`), and prints a per-message summary at the end.

## Tests

Each task with meaningful unit-testable logic (Tasks 1 and 3) ships a
small, dependency-free test binary (no GoogleTest/Catch2 — consistent
with the assignment's "avoid third-party libraries" guidance). Run them
directly or via `ctest` from inside each task's `build/` directory:

```bash
ctest --output-on-failure
```

Task 2 is exercised end-to-end via its demo (concurrency-heavy code is
more informatively verified by running the real producers/consumer than
by unit-testing individual pieces in isolation); the stall-injection
bonus flag doubles as a repeatable staleness-detection test.
