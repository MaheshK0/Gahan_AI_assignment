#pragma once
#include "dbc_database.hpp"
#include "frame_log_reader.hpp"

#include <optional>
#include <string>

namespace can {

// Live vehicle state, as required by the assignment: RPM, Speed, Gear,
// Brake Status, Steering Angle. Each field is independently updated as
// its owning message is decoded, and independently known/unknown
// (std::optional) until the first frame for that message arrives.
struct CanVehicleState {
    std::optional<double> rpm;
    std::optional<double> speed_kmh;
    std::optional<std::string> gear;
    std::optional<bool> brake_pressed;
    std::optional<double> steering_angle_deg;
};

// Applies a decoded EngineData (0x180) / VehicleSpeed (0x220) /
// BrakeStatus (0x1A0) / SteeringData (0x2B0) frame to the live state.
// Unknown message names are ignored (the message simply doesn't map to
// any vehicle-state field).
class VehicleStateUpdater {
public:
    void apply(const MessageDef& msg, const RawFrame& frame, CanVehicleState& state);
};

} // namespace can
