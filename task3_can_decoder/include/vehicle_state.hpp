#pragma once
#include "dbc.hpp"

#include <chrono>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace can {

using Clock = std::chrono::steady_clock;

// The subset of vehicle state the assignment asks us to maintain.
struct VehicleState {
    std::optional<double> rpm;
    std::optional<double> speed_kmh;
    std::optional<std::string> gear;
    std::optional<bool> brake_pressed;
    std::optional<double> steering_deg;
};

// Rolling health/diagnostics for a single CAN message ID.
struct MessageHealth {
    std::string name;
    uint32_t can_id = 0;
    unsigned period_ms = 0;
    // Timeout threshold derived from period_ms (see DESIGN.md for the
    // reasoning): 2.5x the nominal period, so an occasional single missed
    // frame plus normal jitter does not falsely trigger a timeout, but two
    // or more consecutive misses does.
    std::chrono::milliseconds timeoutThreshold() const {
        return std::chrono::milliseconds(static_cast<long long>(period_ms * 2.5));
    }

    Clock::time_point last_seen{};
    bool has_data = false;

    uint64_t frames_received = 0;
    uint64_t counter_faults = 0;
    uint64_t checksum_faults = 0;

    std::optional<int> last_counter;
};

struct SystemSnapshot {
    VehicleState state;
    std::map<uint32_t, MessageHealth> health; // copy, safe to read without a lock
    std::vector<std::string> recentFaults; // faults raised since the previous snapshot
};

// Thread-safety note: this project's Task 3 demo decodes and updates state
// from a single replay thread (see DESIGN.md -- unlike Task 2, nothing in
// the spec requires concurrent producers here), so a mutex is technically
// not required for correctness. One is still used, at negligible cost,
// so the manager is safe to extend to a multi-threaded producer (e.g. a
// live SocketCAN reader) without further changes.
class VehicleStateManager {
public:
    explicit VehicleStateManager(const DbcDatabase& db);

    // Applies one decoded frame. `known` is false for CAN IDs not present
    // in the DBC -- those are still counted/logged but do not update
    // VehicleState fields.
    void applyFrame(uint32_t can_id, const std::array<uint8_t, 8>& data, long timestamp_ms);
    void noteUnknownId(uint32_t can_id);

    SystemSnapshot snapshot();

    // Overall health string, e.g. "OK" or "WARNING: SteeringData timeout".
    // Computed from current staleness + any fault seen in the most recent
    // snapshot window.
    std::string overallHealth();

private:
    const DbcDatabase& db_;
    std::mutex mutex_;

    VehicleState state_;
    std::map<uint32_t, MessageHealth> health_;
    uint64_t unknown_id_frames_ = 0;
    std::vector<std::string> pendingFaults_;
};

} // namespace can
