#include "sensor_producer.hpp"

#include <algorithm>

namespace agg {

SensorReplayProducer::SensorReplayProducer(std::string name,
                                            std::vector<ReplayEvent> events,
                                            Clock::time_point replay_start,
                                            double speed_factor)
    : name_(std::move(name)),
      events_(std::move(events)),
      replay_start_(replay_start),
      speed_factor_(speed_factor) {
    std::sort(events_.begin(), events_.end(),
              [](const ReplayEvent& a, const ReplayEvent& b) { return a.ts_ms < b.ts_ms; });
}

void SensorReplayProducer::injectStall(long stall_after_ms, std::chrono::milliseconds stall_duration) {
    stall_after_ms_ = stall_after_ms;
    stall_duration_ = stall_duration;
}

void SensorReplayProducer::start() {
    thread_ = std::thread(&SensorReplayProducer::run, this);
}

void SensorReplayProducer::requestStop() {
    stop_requested_.store(true);
    cv_.notify_all();
}

void SensorReplayProducer::join() {
    if (thread_.joinable()) thread_.join();
}

void SensorReplayProducer::run() {
    bool stalled_once = false;

    for (const auto& ev : events_) {
        if (stop_requested_.load()) break;

        // Fault injection: once we cross the configured source timestamp,
        // hold off (without emitting anything) for stall_duration. This
        // deliberately makes the sensor look "quiet" from the aggregator's
        // point of view, exercising the staleness detector.
        if (!stalled_once && stall_after_ms_ >= 0 && ev.ts_ms >= stall_after_ms_) {
            stalled_once = true;
            std::unique_lock<std::mutex> lock(cv_mutex_);
            cv_.wait_for(lock, stall_duration_, [this] { return stop_requested_.load(); });
            if (stop_requested_.load()) break;
            // Re-anchor the replay clock so we don't try to "catch up" by
            // firing a burst of overdue events immediately after the stall.
            replay_start_ = Clock::now() -
                std::chrono::milliseconds(static_cast<long long>(ev.ts_ms / speed_factor_));
        }

        const auto target = replay_start_ +
            std::chrono::milliseconds(static_cast<long long>(ev.ts_ms / speed_factor_));

        std::unique_lock<std::mutex> lock(cv_mutex_);
        cv_.wait_until(lock, target, [this] { return stop_requested_.load(); });
        lock.unlock();

        if (stop_requested_.load()) break;
        ev.apply();
    }

    finished_.store(true);
}

} // namespace agg
