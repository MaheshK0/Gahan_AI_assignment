#include "vehicle_state.hpp"
#include "can_decoder.hpp"

#include <sstream>

namespace can {

VehicleStateManager::VehicleStateManager(const DbcDatabase& db) : db_(db) {
    for (const auto& [id, msg] : db.messages()) {
        MessageHealth h;
        h.name = msg.name;
        h.can_id = id;
        h.period_ms = msg.period_ms;
        health_[id] = h;
    }
}

void VehicleStateManager::applyFrame(uint32_t can_id, const std::array<uint8_t, 8>& data, long timestamp_ms) {
    std::lock_guard<std::mutex> lock(mutex_);

    const MessageDef* msg = db_.find(can_id);
    if (!msg) {
        noteUnknownId(can_id);
        return;
    }

    MessageHealth& h = health_[can_id];
    h.frames_received++;
    h.last_seen = Clock::now();
    h.has_data = true;

    // --- Counter sequence check ---
    const SignalDef* counterSig = msg->findSignal("counter");
    if (counterSig) {
        int counter = static_cast<int>(decodeSignal(data, *counterSig, msg->byte_order));
        if (h.last_counter.has_value()) {
            int expected = (*h.last_counter + 1) % 16; // counter is always a 4-bit field in this DBC
            if (counter != expected) {
                h.counter_faults++;
                std::ostringstream ss;
                ss << "[t=" << timestamp_ms << "ms] " << msg->name
                   << " (0x" << std::hex << can_id << std::dec << "): counter discontinuity"
                   << " (expected " << expected << ", got " << counter << ")";
                pendingFaults_.push_back(ss.str());
            }
        }
        h.last_counter = counter;
    }

    // --- Checksum check ---
    const SignalDef* checksumSig = msg->findSignal("checksum");
    if (checksumSig) {
        uint8_t expected = computeXorChecksum(data, *checksumSig);
        int actual = static_cast<int>(decodeSignal(data, *checksumSig, msg->byte_order));
        if (actual != expected) {
            h.checksum_faults++;
            std::ostringstream ss;
            ss << "[t=" << timestamp_ms << "ms] " << msg->name
               << " (0x" << std::hex << can_id << std::dec << "): checksum mismatch"
               << " (expected 0x" << std::hex << static_cast<int>(expected)
               << ", got 0x" << actual << std::dec << ")";
            pendingFaults_.push_back(ss.str());
        }
    }

    // --- Update VehicleState fields ---
    if (msg->name == "EngineData") {
        if (auto* s = msg->findSignal("rpm")) state_.rpm = decodeSignal(data, *s, msg->byte_order);
    } else if (msg->name == "VehicleSpeed") {
        if (auto* s = msg->findSignal("speed")) state_.speed_kmh = decodeSignal(data, *s, msg->byte_order);
        if (auto* s = msg->findSignal("gear")) {
            int g = static_cast<int>(decodeSignal(data, *s, msg->byte_order));
            auto it = s->enum_values.find(g);
            state_.gear = (it != s->enum_values.end()) ? it->second : ("UNKNOWN(" + std::to_string(g) + ")");
        }
    } else if (msg->name == "SteeringData") {
        if (auto* s = msg->findSignal("steering_angle")) state_.steering_deg = decodeSignal(data, *s, msg->byte_order);
    } else if (msg->name == "BrakeStatus") {
        if (auto* s = msg->findSignal("brake_pressed")) state_.brake_pressed = decodeSignal(data, *s, msg->byte_order) != 0.0;
    }
}

void VehicleStateManager::noteUnknownId(uint32_t can_id) {
    unknown_id_frames_++;
    std::ostringstream ss;
    ss << "Unknown CAN ID 0x" << std::hex << can_id << std::dec << " -- no DBC entry, frame ignored for state purposes.";
    pendingFaults_.push_back(ss.str());
}

SystemSnapshot VehicleStateManager::snapshot() {
    std::lock_guard<std::mutex> lock(mutex_);
    SystemSnapshot snap;
    snap.state = state_;
    snap.health = health_;
    snap.recentFaults = pendingFaults_;
    pendingFaults_.clear();
    return snap;
}

std::string VehicleStateManager::overallHealth() {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = Clock::now();
    for (auto& [id, h] : health_) {
        if (!h.has_data) continue;
        if ((now - h.last_seen) > h.timeoutThreshold()) {
            return "WARNING: " + h.name + " timeout (no frame for over " +
                   std::to_string(h.timeoutThreshold().count()) + "ms)";
        }
        if (h.counter_faults > 0 || h.checksum_faults > 0) {
            return "WARNING: " + h.name + " has reported anomalies (counter=" +
                   std::to_string(h.counter_faults) + " checksum=" + std::to_string(h.checksum_faults) + ")";
        }
    }
    if (unknown_id_frames_ > 0) {
        return "WARNING: " + std::to_string(unknown_id_frames_) + " frame(s) with unknown CAN ID observed";
    }
    return "OK";
}

} // namespace can
