#pragma once
#include <chrono>
#include <mutex>
#include <optional>
#include <string>

namespace agg {

using Clock = std::chrono::steady_clock;

struct GpsSample {
    double lat = 0;
    double lon = 0;
    double alt_m = 0;
    int fix_quality = 0;
};

struct ImuSample {
    double accel_x = 0, accel_y = 0, accel_z = 0;
    double gyro_x = 0, gyro_y = 0, gyro_z = 0;
};

struct EncoderSample {
    long left_ticks = 0;
    long right_ticks = 0;
    double speed_mps = 0;
};

// Staleness thresholds from the assignment spec.
constexpr std::chrono::milliseconds kGpsStaleAfter{500};
constexpr std::chrono::milliseconds kImuStaleAfter{50};
constexpr std::chrono::milliseconds kEncoderStaleAfter{100};

// A point-in-time, consistent snapshot of the aggregated vehicle state,
// safe to read/print/export without holding any lock.
struct VehicleStateSnapshot {
    std::optional<GpsSample> gps;
    std::optional<ImuSample> imu;
    std::optional<EncoderSample> encoder;

    bool gps_stale = true;
    bool imu_stale = true;
    bool encoder_stale = true;

    // Source timestamp (from the CSV) of the last update per sensor, and
    // how long ago (wall clock) that update was received -- useful for
    // display/diagnostics.
    long gps_source_ts_ms = -1;
    long imu_source_ts_ms = -1;
    long encoder_source_ts_ms = -1;
};

// Thread-safe holder for the latest sample from each independent sensor
// producer, plus staleness computation. Each producer calls update*()
// from its own thread; any number of reader threads may call snapshot()
// concurrently. Internally protected by a single mutex -- the update rate
// here (tens of Hz) makes contention a non-issue, so a single mutex keeps
// the design simple and easy to reason about (no risk of inconsistent
// cross-field reads, no lock-ordering/deadlock concerns since there is
// only one lock in the whole component).
class VehicleStateManager {
public:
    void updateGps(const GpsSample& s, long source_ts_ms);
    void updateImu(const ImuSample& s, long source_ts_ms);
    void updateEncoder(const EncoderSample& s, long source_ts_ms);

    // Returns a consistent snapshot with staleness flags computed against
    // "now".
    VehicleStateSnapshot snapshot() const;

private:
    mutable std::mutex mutex_;

    std::optional<GpsSample> gps_;
    std::optional<ImuSample> imu_;
    std::optional<EncoderSample> encoder_;

    Clock::time_point gps_last_update_{};
    Clock::time_point imu_last_update_{};
    Clock::time_point encoder_last_update_{};

    long gps_source_ts_ms_ = -1;
    long imu_source_ts_ms_ = -1;
    long encoder_source_ts_ms_ = -1;
};

} // namespace agg
