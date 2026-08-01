# Task 2 — Multi-Sensor Data Aggregator

Replays `gps_reference.csv`, `imu_reference.csv`, and
`wheel_encoder_reference.csv` concurrently — one producer thread per
sensor, each respecting its own original inter-sample timing — and
maintains a single, latest `VehicleState` combining all three, with
staleness detection per the assignment's thresholds.

See the root [`DESIGN.md`](../DESIGN.md#task-2--multi-sensor-data-aggregator)
for the full architecture writeup (threading model, why a single mutex is
fine here, the replay-timing assumption, and the stall-injection bonus).

## Architecture

Three fully independent producer threads (one per sensor CSV), a single
shared, mutex-protected state holder, and a printer/consumer thread that
polls it on a fixed cadence. This is a classic producer-consumer
arrangement with *multiple* producers and one logical consumer that
always reads the latest value per source — no merging or interpolation
across sensors, since each has its own sampling rate and timeline.

```mermaid
flowchart TB
    subgraph Producers["Producer threads (independent timelines)"]
        GPS["SensorReplayProducer\n(gps_reference.csv, 200ms period)"]
        IMU["SensorReplayProducer\n(imu_reference.csv, 10ms period)"]
        ENC["SensorReplayProducer\n(wheel_encoder_reference.csv, 20ms period)"]
    end

    GPS -->|updateGps + timestamp| VSM
    IMU -->|updateImu + timestamp| VSM
    ENC -->|updateEncoder + timestamp| VSM

    VSM["VehicleStateManager\n(single std::mutex)\nlatest GPS / IMU / Encoder sample\n+ last-update timestamps"]

    VSM -->|snapshot| Printer["Printer/consumer thread\n(polls every 200ms, no busy-wait)"]
    Printer --> Console["Console: state + staleness flags"]
    Printer -->|optional| CSV["--export path.csv"]
```

**Shutdown path:** each producer's `run()` loop checks an atomic
`stop_requested_` flag before, and is interruptible during, every sleep
(via `condition_variable::wait_until`). `main()` calls `requestStop()` +
`join()` on all three uniformly, whether they finished naturally at the
end of their CSV or were stopped early — no thread is ever left running
or blocked past shutdown.

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

[`demos/task2_sensor_aggregator.mp4`](../demos/task2_sensor_aggregator.mp4)
(repo root) — see [`demos/README.md`](../demos/README.md) if it's not
there yet.
