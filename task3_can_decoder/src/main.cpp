// CAN decoder + vehicle state demo.
//
// Replays frames.log (a captured CAN bus trace), decodes every frame it
// recognizes against vehicle.dbc, maintains a live VehicleState (RPM,
// Speed, Gear, Brake Status, Steering Angle), and reports per-message
// diagnostics: counter-sequence gaps, checksum mismatches, timeouts
// derived from each message's period_ms, and unknown CAN IDs.
//
// Usage:
//   can_decoder_demo [--speed N] [--data-dir path] [--dbc path/to/vehicle.dbc]
//
// See DESIGN.md for the checksum-validation and timeout-threshold
// assumptions, and for why vehicle.dbc (not dbc.json) is used as the
// decode source of truth here.

#include "dbc.hpp"
#include "can_decoder.hpp"
#include "frame_log_reader.hpp"
#include "vehicle_state.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <thread>

using namespace can;

namespace {

struct Args {
    double speed = 1.0;
    std::string data_dir = "task3_can_decoder/data";
    std::string dbc_path; // defaults to data_dir + "/vehicle.dbc"
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
        else if (arg == "--data-dir") a.data_dir = next("--data-dir");
        else if (arg == "--dbc") a.dbc_path = next("--dbc");
        else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [--speed N] [--data-dir path] [--dbc path]\n";
            std::exit(0);
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            std::exit(1);
        }
    }
    if (a.dbc_path.empty()) a.dbc_path = a.data_dir + "/vehicle.dbc";
    return a;
}

void printState(const VehicleState& s) {
    std::printf("RPM=%-8s Speed=%-10s Gear=%-4s Brake=%-4s Steering=%-9s",
        s.rpm ? (std::to_string(static_cast<int>(*s.rpm)) + "rpm").c_str() : "--",
        s.speed_kmh ? (std::to_string(*s.speed_kmh).substr(0, 5) + "km/h").c_str() : "--",
        s.gear ? s.gear->c_str() : "--",
        s.brake_pressed ? (*s.brake_pressed ? "ON" : "OFF") : "--",
        s.steering_deg ? (std::to_string(*s.steering_deg).substr(0, 6) + "deg").c_str() : "--");
}

} // namespace

int main(int argc, char** argv) {
    Args args = parseArgs(argc, argv);

    DbcDatabase db;
    std::vector<LogFrame> frames;
    try {
        db = DbcDatabase::loadFromDbcText(args.dbc_path);
        frames = FrameLogReader::readAll(args.data_dir + "/frames.log");
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    std::printf("Loaded %zu message definitions from vehicle.dbc:\n", db.messages().size());
    for (const auto& [id, msg] : db.messages()) {
        std::printf("  0x%03X %-14s period=%3ums  byte_order=%s  signals=%zu\n",
            id, msg.name.c_str(), msg.period_ms,
            msg.byte_order == ByteOrder::Intel ? "intel" : "motorola", msg.signals.size());
    }
    std::printf("Loaded %zu frames from frames.log. Replay speed = %.2fx\n\n", frames.size(), args.speed);

    VehicleStateManager manager(db);

    const auto start = Clock::now();
    std::mutex cv_mutex;
    std::condition_variable cv;

    for (const auto& frame : frames) {
        auto target = start + std::chrono::milliseconds(
            static_cast<long long>(frame.timestamp_ms / args.speed));
        std::unique_lock<std::mutex> lock(cv_mutex);
        cv.wait_until(lock, target); // sleeps without busy-waiting; nothing ever notifies it early, so it simply wakes at `target`
        lock.unlock();

        manager.applyFrame(frame.can_id, frame.data, frame.timestamp_ms);

        auto snap = manager.snapshot();
        for (const auto& fault : snap.recentFaults) {
            std::printf("  !! %s\n", fault.c_str());
        }
        std::printf("[t=%5ldms] ", frame.timestamp_ms);
        printState(snap.state);
        std::printf("  health=%s\n", manager.overallHealth().c_str());
    }

    std::printf("\n--- Per-message summary ---\n");
    auto finalSnap = manager.snapshot();
    for (const auto& [id, h] : finalSnap.health) {
        std::printf("0x%03X %-14s frames_received=%-4llu counter_faults=%-3llu checksum_faults=%-3llu\n",
            id, h.name.c_str(),
            static_cast<unsigned long long>(h.frames_received),
            static_cast<unsigned long long>(h.counter_faults),
            static_cast<unsigned long long>(h.checksum_faults));
    }
    std::printf("\nFinal overall health: %s\n", manager.overallHealth().c_str());

    return 0;
}
