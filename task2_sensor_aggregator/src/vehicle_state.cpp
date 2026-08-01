#include "vehicle_state.hpp"

namespace agg {

void VehicleStateManager::updateGps(const GpsSample& s, long source_ts_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    gps_ = s;
    gps_last_update_ = Clock::now();
    gps_source_ts_ms_ = source_ts_ms;
}

void VehicleStateManager::updateImu(const ImuSample& s, long source_ts_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    imu_ = s;
    imu_last_update_ = Clock::now();
    imu_source_ts_ms_ = source_ts_ms;
}

void VehicleStateManager::updateEncoder(const EncoderSample& s, long source_ts_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    encoder_ = s;
    encoder_last_update_ = Clock::now();
    encoder_source_ts_ms_ = source_ts_ms;
}

VehicleStateSnapshot VehicleStateManager::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    VehicleStateSnapshot snap;
    const auto now = Clock::now();

    snap.gps = gps_;
    snap.imu = imu_;
    snap.encoder = encoder_;
    snap.gps_source_ts_ms = gps_source_ts_ms_;
    snap.imu_source_ts_ms = imu_source_ts_ms_;
    snap.encoder_source_ts_ms = encoder_source_ts_ms_;

    snap.gps_stale = !gps_.has_value() ||
        (now - gps_last_update_) > kGpsStaleAfter;
    snap.imu_stale = !imu_.has_value() ||
        (now - imu_last_update_) > kImuStaleAfter;
    snap.encoder_stale = !encoder_.has_value() ||
        (now - encoder_last_update_) > kEncoderStaleAfter;

    return snap;
}

} // namespace agg
