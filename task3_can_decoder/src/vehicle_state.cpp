#include "vehicle_state.hpp"
#include "signal_codec.hpp"

namespace can {

void VehicleStateUpdater::apply(const MessageDef& msg, const RawFrame& frame, CanVehicleState& state) {
    if (msg.name == "EngineData") {
        if (const auto* s = msg.findSignal("rpm")) {
            state.rpm = SignalCodec::decodePhysical(frame.data, *s, msg.byte_order);
        }
    } else if (msg.name == "VehicleSpeed") {
        if (const auto* s = msg.findSignal("speed")) {
            state.speed_kmh = SignalCodec::decodePhysical(frame.data, *s, msg.byte_order);
        }
        if (const auto* s = msg.findSignal("gear")) {
            const int raw = static_cast<int>(SignalCodec::extractRaw(frame.data, *s, msg.byte_order));
            auto it = s->enum_values.find(raw);
            state.gear = (it != s->enum_values.end()) ? it->second : ("UNKNOWN(" + std::to_string(raw) + ")");
        }
    } else if (msg.name == "BrakeStatus") {
        if (const auto* s = msg.findSignal("brake_pressed")) {
            state.brake_pressed = SignalCodec::extractRaw(frame.data, *s, msg.byte_order) != 0;
        }
    } else if (msg.name == "SteeringData") {
        if (const auto* s = msg.findSignal("steering_angle")) {
            state.steering_angle_deg = SignalCodec::decodePhysical(frame.data, *s, msg.byte_order);
        }
    }
    // Any other/unmapped message name: known to the DBC but not tied to a
    // vehicle-state field -- decoded fine, just doesn't affect state.
}

} // namespace can
