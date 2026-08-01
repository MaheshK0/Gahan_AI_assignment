#pragma once
#include "vehicle_state.hpp"

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace agg {

// One row's worth of replay work: a source timestamp (as read from the
// CSV, in ms since the start of that sensor's log) and a callback that
// applies it to the VehicleStateManager.
struct ReplayEvent {
    long ts_ms = 0;
    std::function<void()> apply;
};

// Replays a single sensor's CSV log on its own thread, respecting the
// original inter-sample timing (optionally accelerated by speedFactor),
// and pushes each sample into the shared VehicleStateManager as it
// "arrives". Each producer is a fully independent unit: its own thread,
// its own timeline, no shared mutable state with other producers other
// than the (already thread-safe) VehicleStateManager they all write into.
//
// Sleeping uses condition_variable::wait_until against a shared start
// time, rather than sleep_for in a loop, so:
//   (a) there is no busy-waiting -- the thread blocks in the kernel
//       between events, and
//   (b) stop()/pause() can wake the thread immediately instead of it
//       having to finish a sleep first, which is what makes graceful,
//       prompt shutdown possible.
class SensorReplayProducer {
public:
    SensorReplayProducer(std::string name,
                          std::vector<ReplayEvent> events,
                          Clock::time_point replay_start,
                          double speed_factor);

    // Optional fault injection (bonus requirement): once the replay
    // reaches source-timestamp stall_after_ms, pause emitting updates for
    // stall_duration before resuming the timeline. Used to demonstrate
    // that staleness detection in VehicleStateManager correctly flags a
    // sensor that goes quiet mid-run.
    void injectStall(long stall_after_ms, std::chrono::milliseconds stall_duration);

    void start();
    void requestStop();  // asynchronous, idempotent
    void join();
    bool finished() const { return finished_.load(); }
    const std::string& name() const { return name_; }

private:
    void run();

    std::string name_;
    std::vector<ReplayEvent> events_;
    Clock::time_point replay_start_;
    double speed_factor_;

    long stall_after_ms_ = -1;
    std::chrono::milliseconds stall_duration_{0};

    std::thread thread_;
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> finished_{false};

    // Used only to make sleeps interruptible; no data it "protects" is
    // shared beyond the predicate itself.
    std::mutex cv_mutex_;
    std::condition_variable cv_;
};

} // namespace agg
