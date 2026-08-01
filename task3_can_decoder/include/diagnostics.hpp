#pragma once
#include "dbc_database.hpp"
#include "frame_log_reader.hpp"
#include "signal_codec.hpp"

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace can {

enum class AnomalyKind { CounterGap, ChecksumMismatch, Timeout, UnknownId };

struct Anomaly {
    AnomalyKind kind;
    uint32_t can_id = 0;
    std::string message_name; // empty for UnknownId
    std::string detail;
    long timestamp_ms = 0;
};

// Rolling per-message diagnostics, used for the end-of-run summary (bonus
// requirement).
struct MessageStats {
    uint64_t frames_received = 0;
    uint64_t counter_faults = 0;
    uint64_t checksum_faults = 0;
    uint64_t timeout_events = 0;
};

// Detects the three classes of fault the assignment calls out:
//   1. Counter discontinuities -- every message carries a 4-bit rolling
//      counter signal; a gap (anything other than +1 mod 16 from the
//      previous frame of that ID) is flagged.
//   2. Checksum anomalies -- see DESIGN.md for how the checksum algorithm
//      was determined (vehicle.dbc's CM_ comments document it directly as
//      "XOR of the other 7 bytes in the frame", which was then verified
//      against the sample data).
//   3. Missing frames / timeouts -- derived from each message's
//      period_ms, per the assignment's instruction to use it as the
//      timeout baseline rather than picking an arbitrary value. A
//      multiplier is applied on top of the raw period to tolerate normal
//      scheduling/replay jitter without generating false positives; see
//      DESIGN.md for the chosen value and reasoning.
//
// Unknown CAN IDs (not present in the DBC) are reported as a distinct
// anomaly kind rather than being silently dropped or crashing the
// decoder.
class DiagnosticsEngine {
public:
    static constexpr double kTimeoutMultiplier = 3.0;

    explicit DiagnosticsEngine(const DbcDatabase& db) : db_(db) {}

    // Call once per received, DBC-known frame (after decoding). Returns
    // any anomalies detected for this specific frame (counter/checksum).
    // `now_ms` is the frame's own timestamp (used to update the
    // last-seen clock for timeout purposes).
    std::vector<Anomaly> onKnownFrame(const MessageDef& msg, const RawFrame& frame);

    // Call for a CAN ID with no matching DBC entry.
    Anomaly onUnknownFrame(const RawFrame& frame);

    // Call periodically (e.g. once per replayed frame, or on a timer) to
    // check every known message for a missed-frame timeout relative to
    // `now_ms`. Only returns *new* timeout anomalies (i.e. it won't
    // re-report the same ongoing timeout on every call) -- see
    // isCurrentlyTimedOut() for the live/continuous state instead.
    std::vector<Anomaly> checkTimeouts(long now_ms);

    // Live state query: is this message currently considered missing,
    // as of the last checkTimeouts()/onKnownFrame() call? Used to drive
    // the "overall health" indicator continuously, independent of the
    // edge-triggered checkTimeouts() anomaly list above.
    bool isCurrentlyTimedOut(uint32_t id, long now_ms) const;

    const std::map<uint32_t, MessageStats>& stats() const { return stats_; }
    uint64_t unknownFrameCount() const { return unknown_frame_count_; }

private:
    const DbcDatabase& db_;

    std::map<uint32_t, int> last_counter_;       // last seen counter value per ID
    std::map<uint32_t, long> last_seen_ms_;       // last frame timestamp per ID
    std::map<uint32_t, bool> currently_timed_out_; // edge-tracking for checkTimeouts()
    std::map<uint32_t, MessageStats> stats_;
    uint64_t unknown_frame_count_ = 0;
};

} // namespace can
