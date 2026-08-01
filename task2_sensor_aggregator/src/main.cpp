// Multi-sensor data aggregator demo.
//
// Replays gps_reference.csv, imu_reference.csv and wheel_encoder_reference.csv
// concurrently -- one producer thread per sensor -- each respecting its own
// original inter-sample timing, and maintains a single latest VehicleState
// combining all three. A separate printer thread polls that state and
// reports staleness per the assignment's thresholds (GPS 500ms, IMU 50ms,
// Encoder 100ms).
//
// Usage:
//   sensor_aggregator_demo [--speed N] [--stall gps|imu|encoder --stall-after MS --stall-duration MS]
//                           [--export path.csv] [--data-dir path]
//
// Assumptions (see DESIGN.md for the full writeup):
//   - Default replay speed is 1.0 (real time), because the staleness
//     thresholds given in the spec (500/50/100 ms) are wall-clock values;
//     running the replay at any other speed would require the thresholds
//     to be scaled too, which the spec doesn't ask for. --speed is
//     provided for convenience but is not the default.
//   - "Latest VehicleState" means last-value-wins per sensor, with no
//     synchronization/interpolation across sensors -- each producer
//     writes independently and readers see whatever is freshest at read
//     time, as called out in the assignment notes.

#include "csv_reader.hpp"
#include "sensor_producer.hpp"
#include "vehicle_state.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

using namespace agg;

namespace {

struct Args {
    double speed = 1.0;
    std::string stall_sensor;
    long stall_after_ms = -1;
    long stall_duration_ms = 0;
    std::string export_path;
    std::string data_dir = "task2_sensor_aggregator/data";
};

Args parseArgs(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&](const char* flag) -> std::string {
            if (i + 1 >= argc) { std::cerr << "Missing value for " << flag << "\n"; std::exit(1); }
            return argv[++i];
        };
        if (arg == "--speed") a.speed = std::stod(next("--speed"));
        else if (arg == "--stall") a.stall_sensor = next("--stall");
        else if (arg == "--stall-after") a.stall_after_ms = std::stol(next("--stall-after"));
        else if (arg == "--stall-duration") a.stall_duration_ms = std::stol(next("--stall-duration"));
        else if (arg == "--export") a.export_path = next("--export");
        else if (arg == "--data-dir") a.data_dir = next("--data-dir");
        else if (arg == "--help") {
            std::cout << "Usage: " << argv[0]
                      << " [--speed N] [--stall gps|imu|encoder --stall-after MS --stall-duration MS]"
                         " [--export path.csv] [--data-dir path]\n";
            std::exit(0);
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            std::exit(1);
        }
    }
    return a;
}

std::vector<ReplayEvent> buildGpsEvents(const std::string& path, VehicleStateManager& mgr) {
    auto rows = CsvReader::readAll(path);
    std::vector<ReplayEvent> events;
    for (size_t i = 1; i < rows.size(); ++i) { // skip header
        const auto& r = rows[i];
        if (r.size() < 5) continue;
        long ts = std::stol(r[0]);
        GpsSample s;
        s.lat = std::stod(r[1]);
        s.lon = std::stod(r[2]);
        s.alt_m = std::stod(r[3]);
        s.fix_quality = std::stoi(r[4]);
        events.push_back({ts, [&mgr, s, ts] { mgr.updateGps(s, ts); }});
    }
    return events;
}

std::vector<ReplayEvent> buildImuEvents(const std::string& path, VehicleStateManager& mgr) {
    auto rows = CsvReader::readAll(path);
    std::vector<ReplayEvent> events;
    for (size_t i = 1; i < rows.size(); ++i) {
        const auto& r = rows[i];
        if (r.size() < 7) continue;
        long ts = std::stol(r[0]);
        ImuSample s;
        s.accel_x = std::stod(r[1]); s.accel_y = std::stod(r[2]); s.accel_z = std::stod(r[3]);
        s.gyro_x = std::stod(r[4]);  s.gyro_y = std::stod(r[5]);  s.gyro_z = std::stod(r[6]);
        events.push_back({ts, [&mgr, s, ts] { mgr.updateImu(s, ts); }});
    }
    return events;
}

std::vector<ReplayEvent> buildEncoderEvents(const std::string& path, VehicleStateManager& mgr) {
    auto rows = CsvReader::readAll(path);
    std::vector<ReplayEvent> events;
    for (size_t i = 1; i < rows.size(); ++i) {
        const auto& r = rows[i];
        if (r.size() < 4) continue;
        long ts = std::stol(r[0]);
        EncoderSample s;
        s.left_ticks = std::stol(r[1]);
        s.right_ticks = std::stol(r[2]);
        s.speed_mps = std::stod(r[3]);
        events.push_back({ts, [&mgr, s, ts] { mgr.updateEncoder(s, ts); }});
    }
    return events;
}

const char* flag(bool stale) { return stale ? "STALE" : "ok"; }

void printSnapshot(const VehicleStateSnapshot& s, double elapsed_s) {
    std::printf("[t=%6.2fs] ", elapsed_s);
    if (s.gps) {
        std::printf("GPS(%s src=%ldms lat=%.6f lon=%.6f)  ", flag(s.gps_stale), s.gps_source_ts_ms, s.gps->lat, s.gps->lon);
    } else {
        std::printf("GPS(no data)  ");
    }
    if (s.imu) {
        std::printf("IMU(%s src=%ldms az=%.3f)  ", flag(s.imu_stale), s.imu_source_ts_ms, s.imu->accel_z);
    } else {
        std::printf("IMU(no data)  ");
    }
    if (s.encoder) {
        std::printf("ENC(%s src=%ldms spd=%.2fm/s)", flag(s.encoder_stale), s.encoder_source_ts_ms, s.encoder->speed_mps);
    } else {
        std::printf("ENC(no data)");
    }
    std::printf("\n");
}

} // namespace

int main(int argc, char** argv) {
    Args args = parseArgs(argc, argv);

    VehicleStateManager manager;

    std::vector<ReplayEvent> gpsEvents, imuEvents, encoderEvents;
    try {
        gpsEvents = buildGpsEvents(args.data_dir + "/gps_reference.csv", manager);
        imuEvents = buildImuEvents(args.data_dir + "/imu_reference.csv", manager);
        encoderEvents = buildEncoderEvents(args.data_dir + "/wheel_encoder_reference.csv", manager);
    } catch (const std::exception& e) {
        std::cerr << "Error loading reference data: " << e.what() << "\n";
        return 1;
    }

    std::printf("Loaded %zu GPS, %zu IMU, %zu Encoder samples. Replay speed = %.2fx\n",
                gpsEvents.size(), imuEvents.size(), encoderEvents.size(), args.speed);

    const auto start = Clock::now();
    SensorReplayProducer gpsProducer("gps", std::move(gpsEvents), start, args.speed);
    SensorReplayProducer imuProducer("imu", std::move(imuEvents), start, args.speed);
    SensorReplayProducer encoderProducer("encoder", std::move(encoderEvents), start, args.speed);

    if (args.stall_sensor == "gps") gpsProducer.injectStall(args.stall_after_ms, std::chrono::milliseconds(args.stall_duration_ms));
    else if (args.stall_sensor == "imu") imuProducer.injectStall(args.stall_after_ms, std::chrono::milliseconds(args.stall_duration_ms));
    else if (args.stall_sensor == "encoder") encoderProducer.injectStall(args.stall_after_ms, std::chrono::milliseconds(args.stall_duration_ms));

    std::ofstream exportFile;
    if (!args.export_path.empty()) {
        exportFile.open(args.export_path);
        exportFile << "elapsed_s,gps_stale,gps_lat,gps_lon,imu_stale,imu_az,encoder_stale,encoder_speed_mps\n";
    }

    gpsProducer.start();
    imuProducer.start();
    encoderProducer.start();

    // Printer/consumer loop: polls the aggregator at a fixed cadence and
    // reports current state + staleness. This is a periodic sleep, not a
    // busy-wait -- the thread is fully descheduled between prints.
    const auto printInterval = std::chrono::milliseconds(200);
    while (!(gpsProducer.finished() && imuProducer.finished() && encoderProducer.finished())) {
        std::this_thread::sleep_for(printInterval);
        auto snap = manager.snapshot();
        double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
        printSnapshot(snap, elapsed);
        if (exportFile.is_open()) {
            exportFile << elapsed << ","
                       << snap.gps_stale << "," << (snap.gps ? snap.gps->lat : 0.0) << "," << (snap.gps ? snap.gps->lon : 0.0) << ","
                       << snap.imu_stale << "," << (snap.imu ? snap.imu->accel_z : 0.0) << ","
                       << snap.encoder_stale << "," << (snap.encoder ? snap.encoder->speed_mps : 0.0) << "\n";
        }
    }

    // Graceful shutdown: all producers have naturally reached the end of
    // their logs by this point, but we still call requestStop()+join() on
    // each for a uniform, deterministic shutdown path (this is also what
    // runs if the loop above were exited early, e.g. via a future
    // signal-handling extension).
    gpsProducer.requestStop();
    imuProducer.requestStop();
    encoderProducer.requestStop();
    gpsProducer.join();
    imuProducer.join();
    encoderProducer.join();

    auto finalSnap = manager.snapshot();
    double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
    std::printf("\n--- Final state ---\n");
    printSnapshot(finalSnap, elapsed);
    std::printf("All producers stopped cleanly.\n");

    return 0;
}
