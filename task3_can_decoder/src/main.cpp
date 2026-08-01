// CAN decoder & vehicle state demo.
//
// Replays frames.log respecting its timestamps, decoding each frame
// against the DBC-like signal database (dbc.json), maintaining a live
// VehicleState, and continuously reporting overall system health.
//
// Usage:
//   can_decoder_demo [--speed N] [--data-dir path] [--summary-only]
//
// Design notes (see DESIGN.md for the full writeup):
//   - Byte layout / CRC-style conventions for Intel vs Motorola signals
//     were verified empirically against the sample frames (see
//     signal_codec.hpp).
//   - Checksum = XOR of the other 7 bytes in the frame, taken directly
//     from vehicle.dbc's CM_ comment on the checksum signals and
//     confirmed against the sample data.
//   - Timeout threshold per message = period_ms * 3, to tolerate replay
//     jitter while still catching the deliberately injected ~310ms gap
//     on SteeringData (period 10ms) well before the next frame arrives.
//   - frames.log only contains the 4 documented CAN IDs, but unknown IDs
//     are still handled defensively (logged, counted, never crash).

#include "dbc_database.hpp"
#include "diagnostics.hpp"
#include "frame_log_reader.hpp"
#include "signal_codec.hpp"
#include "vehicle_state.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

using namespace can;
using Clock = std::chrono::steady_clock;

namespace {

struct Args {
    double speed = 1.0;
    std::string data_dir = "task3_can_decoder/data";
    bool summary_only = false;
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
        else if (arg == "--summary-only") a.summary_only = true;
        else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [--speed N] [--data-dir path] [--summary-only]\n";
            std::exit(0);
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            std::exit(1);
        }
    }
    return a;
}

std::string gearOrDash(const CanVehicleState& s) { return s.gear.value_or("-"); }

void printStatusLine(long log_ms, const CanVehicleState& state, bool healthy, const std::string& reason) {
    std::printf("[t=%5ldms] RPM=%-8s Speed=%-9s Gear=%-3s Brake=%-5s Steer=%-9s  %s%s\n",
        log_ms,
        state.rpm ? (std::to_string(static_cast<int>(*state.rpm)) + "rpm").c_str() : "-",
        state.speed_kmh ? (std::to_string(*state.speed_kmh).substr(0, 5) + "km/h").c_str() : "-",
        gearOrDash(state).c_str(),
        state.brake_pressed ? (*state.brake_pressed ? "ON" : "off") : "-",
        state.steering_angle_deg ? (std::to_string(*state.steering_angle_deg).substr(0, 6) + "deg").c_str() : "-",
        healthy ? "OK" : "WARNING",
        reason.empty() ? "" : (": " + reason).c_str());
}

} // namespace

int main(int argc, char** argv) {
    Args args = parseArgs(argc, argv);

    DbcDatabase db;
    std::vector<RawFrame> frames;
    try {
        db = DbcDatabase::loadFromJsonFile(args.data_dir + "/dbc.json");
        frames = FrameLogReader::readAll(args.data_dir + "/frames.log");
    } catch (const std::exception& e) {
        std::cerr << "Error loading input data: " << e.what() << "\n";
        return 1;
    }
    std::sort(frames.begin(), frames.end(),
              [](const RawFrame& a, const RawFrame& b) { return a.timestamp_ms < b.timestamp_ms; });

    std::printf("Loaded %zu messages from DBC, %zu frames from log. Replay speed = %.2fx\n\n",
                db.messages().size(), frames.size(), args.speed);

    DiagnosticsEngine diag(db);
    VehicleStateUpdater updater;
    CanVehicleState state;

    const auto wallStart = Clock::now();
    auto logMsNow = [&]() -> long {
        auto elapsed = std::chrono::duration<double, std::milli>(Clock::now() - wallStart).count();
        return static_cast<long>(elapsed * args.speed);
    };

    // Poll cadence for timeout checks / status prints between frame
    // arrivals (a blocking sleep, not a busy loop).
    const auto pollInterval = std::chrono::milliseconds(20);

    size_t frameIdx = 0;
    long lastPrintedMs = -1000;
    std::string lastHealthReason;

    while (frameIdx < frames.size()) {
        const RawFrame& next = frames[frameIdx];
        const auto targetWall = wallStart +
            std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double, std::milli>(next.timestamp_ms / args.speed));

        // Wait for the next frame's arrival time, but wake periodically to
        // run timeout checks / status prints in the meantime (this is how
        // a >period_ms gap, like the injected SteeringData stall, gets
        // surfaced promptly instead of only being noticed when the next
        // frame finally shows up).
        while (Clock::now() < targetWall) {
            std::this_thread::sleep_for(std::min(pollInterval,
                std::chrono::duration_cast<std::chrono::milliseconds>(targetWall - Clock::now())));
            long nowMs = logMsNow();
            auto timeoutAnomalies = diag.checkTimeouts(nowMs);
            for (const auto& a : timeoutAnomalies) {
                std::printf("  !! [t=%5ldms] TIMEOUT  %-14s 0x%03X  %s\n",
                            a.timestamp_ms, a.message_name.c_str(), a.can_id, a.detail.c_str());
            }
            if (!args.summary_only && (nowMs - lastPrintedMs >= 100)) {
                bool anyTimedOut = false;
                std::string reason;
                for (const auto& [id, msg] : db.messages()) {
                    if (diag.isCurrentlyTimedOut(id, nowMs)) {
                        anyTimedOut = true;
                        reason = msg.name + " missing";
                        break;
                    }
                }
                printStatusLine(nowMs, state, !anyTimedOut, reason);
                lastPrintedMs = nowMs;
            }
        }

        // Frame has "arrived": decode it.
        const MessageDef* msg = db.find(next.can_id);
        if (!msg) {
            Anomaly a = diag.onUnknownFrame(next);
            std::printf("  ?? [t=%5ldms] UNKNOWN  0x%03X  %s\n", a.timestamp_ms, a.can_id, a.detail.c_str());
        } else {
            auto anomalies = diag.onKnownFrame(*msg, next);
            updater.apply(*msg, next, state);
            for (const auto& a : anomalies) {
                const char* label = (a.kind == AnomalyKind::CounterGap) ? "CTR_GAP " : "CHKSUM  ";
                std::printf("  !! [t=%5ldms] %s %-14s 0x%03X  %s\n",
                            a.timestamp_ms, label, a.message_name.c_str(), a.can_id, a.detail.c_str());
            }
            if (!args.summary_only) {
                long nowMs = next.timestamp_ms;
                bool anyTimedOut = diag.isCurrentlyTimedOut(next.can_id, nowMs);
                printStatusLine(nowMs, state, !anyTimedOut, anyTimedOut ? (msg->name + " missing") : "");
                lastPrintedMs = nowMs;
            }
        }
        ++frameIdx;
    }

    std::printf("\n--- Per-message summary ---\n");
    for (const auto& [id, msg] : db.messages()) {
        const auto it = diag.stats().find(id);
        MessageStats st = (it != diag.stats().end()) ? it->second : MessageStats{};
        std::printf("0x%03X %-14s frames=%-4llu counter_faults=%-3llu checksum_faults=%-3llu timeout_events=%-3llu\n",
                    id, msg.name.c_str(),
                    static_cast<unsigned long long>(st.frames_received),
                    static_cast<unsigned long long>(st.counter_faults),
                    static_cast<unsigned long long>(st.checksum_faults),
                    static_cast<unsigned long long>(st.timeout_events));
    }
    std::printf("Unknown-ID frames: %llu\n", static_cast<unsigned long long>(diag.unknownFrameCount()));

    return 0;
}
