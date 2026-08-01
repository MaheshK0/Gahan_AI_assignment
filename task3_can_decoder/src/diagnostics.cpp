#include "diagnostics.hpp"

#include <sstream>

namespace can {

namespace {

std::string fmtAnomaly(const std::string& msg) { return msg; }

} // namespace

std::vector<Anomaly> DiagnosticsEngine::onKnownFrame(const MessageDef& msg, const RawFrame& frame) {
    std::vector<Anomaly> anomalies;
    auto& st = stats_[msg.id];
    ++st.frames_received;

    // --- Counter / sequence-gap check ---
    const SignalDef* counterSig = msg.findSignal("counter");
    if (counterSig) {
        const int counter = static_cast<int>(SignalCodec::extractRaw(frame.data, *counterSig, msg.byte_order));
        auto it = last_counter_.find(msg.id);
        if (it != last_counter_.end()) {
            const int expected = (it->second + 1) % 16;
            if (counter != expected) {
                ++st.counter_faults;
                std::ostringstream oss;
                oss << "counter discontinuity: expected " << expected << ", got " << counter
                    << " (prev=" << it->second << ")";
                anomalies.push_back({AnomalyKind::CounterGap, msg.id, msg.name, fmtAnomaly(oss.str()), frame.timestamp_ms});
            }
        }
        last_counter_[msg.id] = counter;
    }

    // --- Checksum check ---
    // Every checksum signal in this DBC is a single, byte-aligned 8-bit
    // field (verified across all four messages), so its byte index is
    // simply start_bit/8 regardless of byte order -- see SignalCodec's
    // header comment for why the same "linear byte index" applies to both
    // Intel and Motorola signals under the bit-numbering convention used
    // here.
    const SignalDef* checksumSig = msg.findSignal("checksum");
    if (checksumSig) {
        const int checksumByteIdx = checksumSig->start_bit / 8;
        uint8_t computed = 0;
        for (int i = 0; i < 8; ++i) {
            if (i == checksumByteIdx) continue;
            computed ^= frame.data[i];
        }
        const uint8_t received = frame.data[checksumByteIdx];
        if (computed != received) {
            ++st.checksum_faults;
            std::ostringstream oss;
            oss << "checksum mismatch: expected 0x" << std::hex << int(computed)
                << ", got 0x" << int(received) << std::dec;
            anomalies.push_back({AnomalyKind::ChecksumMismatch, msg.id, msg.name, fmtAnomaly(oss.str()), frame.timestamp_ms});
        }
    }

    last_seen_ms_[msg.id] = frame.timestamp_ms;
    currently_timed_out_[msg.id] = false;

    return anomalies;
}

Anomaly DiagnosticsEngine::onUnknownFrame(const RawFrame& frame) {
    ++unknown_frame_count_;
    std::ostringstream oss;
    oss << "unrecognized CAN ID 0x" << std::hex << frame.can_id << std::dec
        << " -- no entry in DBC, frame ignored for decoding";
    return Anomaly{AnomalyKind::UnknownId, frame.can_id, "", fmtAnomaly(oss.str()), frame.timestamp_ms};
}

std::vector<Anomaly> DiagnosticsEngine::checkTimeouts(long now_ms) {
    std::vector<Anomaly> anomalies;
    for (const auto& [id, msg] : db_.messages()) {
        const double threshold = msg.period_ms * kTimeoutMultiplier;
        auto seenIt = last_seen_ms_.find(id);
        const bool neverSeen = (seenIt == last_seen_ms_.end());
        const double sinceLast = neverSeen ? 1e18 : static_cast<double>(now_ms - seenIt->second);

        const bool isTimedOut = sinceLast > threshold;
        const bool wasTimedOut = currently_timed_out_.count(id) ? currently_timed_out_[id] : false;

        if (isTimedOut && !wasTimedOut) {
            // Rising edge: just crossed into timeout.
            auto& st = stats_[id];
            ++st.timeout_events;
            std::ostringstream oss;
            oss << "no frame for " << static_cast<long>(sinceLast) << "ms"
                << " (period=" << msg.period_ms << "ms, threshold=" << static_cast<long>(threshold) << "ms)";
            anomalies.push_back({AnomalyKind::Timeout, id, msg.name, fmtAnomaly(oss.str()), now_ms});
        }
        currently_timed_out_[id] = isTimedOut;
    }
    return anomalies;
}

bool DiagnosticsEngine::isCurrentlyTimedOut(uint32_t id, long now_ms) const {
    const MessageDef* msg = db_.find(id);
    if (!msg) return false;
    auto seenIt = last_seen_ms_.find(id);
    if (seenIt == last_seen_ms_.end()) return true;
    const double threshold = msg->period_ms * kTimeoutMultiplier;
    return static_cast<double>(now_ms - seenIt->second) > threshold;
}

} // namespace can
