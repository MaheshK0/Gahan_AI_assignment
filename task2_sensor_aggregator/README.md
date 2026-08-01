# Task 2 — Multi-Sensor Data Aggregator

Replays `gps_reference.csv`, `imu_reference.csv`, and
`wheel_encoder_reference.csv` concurrently — one producer thread per
sensor, each respecting its own original inter-sample timing — and
maintains a single, latest `VehicleState` combining all three, with
staleness detection per the assignment's thresholds.

See the root [`DESIGN.md`](../DESIGN.md#task-2--multi-sensor-data-aggregator)
for the full architecture writeup (threading model, why a single mutex is
fine here, the replay-timing assumption, and the stall-injection bonus).

## Layout

```
include/vehicle_state.hpp   VehicleStateManager: thread-safe latest-state holder
include/sensor_producer.hpp SensorReplayProducer: one thread per sensor CSV
include/csv_reader.hpp      Minimal hand-rolled CSV reader
src/main.cpp                Demo driver: wires producers + printer thread together
data/                        gps/imu/wheel_encoder reference CSVs (provided sample data)
```

## Build

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## Run

```bash
# From inside build/

# Real-time replay (~8s), prints VehicleState + staleness every 200ms
./sensor_aggregator_demo --data-dir ../data

# Bonus: inject a stall on the IMU stream starting at source-time 2000ms,
# and export the synchronized state stream to CSV
./sensor_aggregator_demo --data-dir ../data --speed 20 \
    --stall imu --stall-after 2000 --stall-duration 3000 \
    --export /tmp/vehicle_state.csv
```

Run `./sensor_aggregator_demo --help` for all options.

## Design highlights

- **One thread per sensor**: genuinely independent producers, each with
  its own timeline, per the assignment's note that GPS/IMU/Encoder should
  be treated as independent producers rather than a single merged stream.
- **Single shared mutex** protecting `VehicleStateManager` — the only
  shared mutable state in the system. Update rates are tens of Hz at
  most, so contention is a non-issue, and one lock means no
  lock-ordering/deadlock concerns.
- **No busy-waiting**: producers sleep via
  `condition_variable::wait_until()` against a shared replay-start time,
  so they're fully descheduled between events and can be woken instantly
  by `requestStop()` for prompt, graceful shutdown.
- **Replay timing assumption**: default speed is 1.0x (real time) — the
  spec's staleness thresholds (GPS 500ms / IMU 50ms / Encoder 100ms) are
  wall-clock values, so real-time is the mode in which they mean what
  they say. `--speed` is provided for convenience.
- **Bonus items implemented**: stall injection (demonstrates staleness
  detection firing and clearing correctly) and CSV export of the
  synchronized state stream.

## Demo video

[Add a link or embedded video of this task's demo here — see the root
README's "Demo Videos" section for instructions.]
