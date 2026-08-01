#include "can_decoder.hpp"
#include "dbc.hpp"
#include "frame_log_reader.hpp"
#include "vehicle_state.hpp"

#include <cstdio>

namespace {

int g_failures = 0;
int g_checks = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        ++g_checks;                                                         \
        if (!(cond)) {                                                       \
            ++g_failures;                                                   \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);      \
        }                                                                    \
    } while (0)

using namespace can;

void test_intel_unsigned_16bit() {
    // rpm: start_bit=0, length=16, intel, scale=0.25 -> byte0=LSB, byte1=MSB
    RawFrame data{0x00, 0x27, 0, 0, 0, 0, 0, 0}; // 0x2700 = 9984 raw -> 9984*0.25=2496 rpm
    SignalDef sig; sig.start_bit = 0; sig.length = 16; sig.scale = 0.25; sig.offset = 0; sig.is_signed = false;
    double v = decodeSignal(data, sig, ByteOrder::Intel);
    CHECK(v == 2496.0);
}

void test_intel_unsigned_nibble() {
    // counter: start_bit=16, length=4, intel -> low nibble of byte2
    RawFrame data{0, 0, 0x0B, 0, 0, 0, 0, 0}; // low nibble = 0xB = 11
    SignalDef sig; sig.start_bit = 16; sig.length = 4; sig.scale = 1; sig.offset = 0; sig.is_signed = false;
    double v = decodeSignal(data, sig, ByteOrder::Intel);
    CHECK(v == 11.0);
}

void test_motorola_signed_16bit_positive() {
    // steering_angle: start_bit=7 (MSB of byte0), length=16, motorola, signed, scale=0.1
    // byte0=0x01 (MSB byte), byte1=0x2C (LSB byte) -> raw = 0x012C = 300 -> 30.0 deg
    RawFrame data{0x01, 0x2C, 0, 0, 0, 0, 0, 0};
    SignalDef sig; sig.start_bit = 7; sig.length = 16; sig.scale = 0.1; sig.offset = 0; sig.is_signed = true;
    double v = decodeSignal(data, sig, ByteOrder::Motorola);
    CHECK(v > 29.99 && v < 30.01);
}

void test_motorola_signed_16bit_negative() {
    // raw = 0xFF8B (two's complement 16-bit) = -117 -> -11.7 deg
    RawFrame data{0xFF, 0x8B, 0, 0, 0, 0, 0, 0};
    SignalDef sig; sig.start_bit = 7; sig.length = 16; sig.scale = 0.1; sig.offset = 0; sig.is_signed = true;
    double v = decodeSignal(data, sig, ByteOrder::Motorola);
    CHECK(v > -11.71 && v < -11.69);
}

void test_motorola_vs_intel_disagree_on_same_bytes() {
    // Sanity check that byte order actually matters: same raw bytes,
    // opposite orderings, should generally produce different raw values
    // for a multi-byte field (unless the bytes happen to be symmetric).
    RawFrame data{0x12, 0x34, 0, 0, 0, 0, 0, 0};
    SignalDef sig; sig.start_bit = 0; sig.length = 16; sig.scale = 1; sig.offset = 0; sig.is_signed = false;
    double intelSig = decodeSignal(data, sig, ByteOrder::Intel);

    SignalDef motSig; motSig.start_bit = 7; motSig.length = 16; motSig.scale = 1; motSig.offset = 0; motSig.is_signed = false;
    double motoVal = decodeSignal(data, motSig, ByteOrder::Motorola);

    CHECK(intelSig == 0x3412); // byte0 = LSB -> 0x34 | (0x12<<8)
    CHECK(motoVal == 0x1234);  // byte0 = MSB -> (0x12<<8) | 0x34
}

void test_checksum_xor_excludes_own_byte() {
    RawFrame data{0x01, 0x02, 0x03, 0xAA, 0x05, 0x06, 0x07, 0x08};
    // checksum occupies byte index 3 (start_bit=24, length=8)
    SignalDef checksumSig; checksumSig.start_bit = 24; checksumSig.length = 8;
    uint8_t x = computeXorChecksum(data, checksumSig);
    uint8_t expected = 0x01 ^ 0x02 ^ 0x03 ^ 0x05 ^ 0x06 ^ 0x07 ^ 0x08;
    CHECK(x == expected);
    CHECK(x != (expected ^ 0xAA)); // sanity: byte3 itself must not be included
}

void test_dbc_text_parses_real_file() {
    DbcDatabase db = DbcDatabase::loadFromDbcText("task3_can_decoder/data/vehicle.dbc");
    CHECK(db.messages().size() == 4);

    const MessageDef* engine = db.find(0x180);
    CHECK(engine != nullptr);
    if (engine) {
        CHECK(engine->name == "EngineData");
        CHECK(engine->period_ms == 20);
        CHECK(engine->byte_order == ByteOrder::Intel);
        CHECK(engine->findSignal("rpm") != nullptr);
    }

    const MessageDef* steering = db.find(0x2B0);
    CHECK(steering != nullptr);
    if (steering) {
        CHECK(steering->name == "SteeringData");
        CHECK(steering->period_ms == 10);
        CHECK(steering->byte_order == ByteOrder::Motorola);
        const SignalDef* angle = steering->findSignal("steering_angle");
        CHECK(angle != nullptr);
        if (angle) CHECK(angle->is_signed == true);
    }

    const MessageDef* speed = db.find(0x220);
    CHECK(speed != nullptr);
    if (speed) {
        const SignalDef* gear = speed->findSignal("gear");
        CHECK(gear != nullptr);
        if (gear) {
            CHECK(gear->enum_values.at(0) == "P");
            CHECK(gear->enum_values.at(1) == "R");
        }
    }
}

void test_frame_log_parses_real_file() {
    auto frames = FrameLogReader::readAll("task3_can_decoder/data/frames.log");
    CHECK(frames.size() == 224);
    CHECK(frames.front().timestamp_ms == 0);
    CHECK(frames.front().can_id == 0x180);
}

void test_known_gap_and_counter_fault_are_detected() {
    // End-to-end regression test pinned to the two faults deliberately
    // injected into frames.log (see DESIGN.md): a counter discontinuity
    // on VehicleSpeed (0x220) and an extended gap on SteeringData (0x2B0).
    DbcDatabase db = DbcDatabase::loadFromDbcText("task3_can_decoder/data/vehicle.dbc");
    auto frames = FrameLogReader::readAll("task3_can_decoder/data/frames.log");
    VehicleStateManager manager(db);

    int counterFaultsSeen = 0;
    for (const auto& frame : frames) {
        manager.applyFrame(frame.can_id, frame.data, frame.timestamp_ms);
        auto snap = manager.snapshot();
        for (const auto& f : snap.recentFaults) {
            if (f.find("counter discontinuity") != std::string::npos) ++counterFaultsSeen;
        }
    }
    CHECK(counterFaultsSeen == 1); // exactly one injected counter discontinuity in the sample

    auto finalSnap = manager.snapshot();
    CHECK(finalSnap.health.at(0x220).counter_faults == 1);
    CHECK(finalSnap.health.at(0x2B0).counter_faults == 0); // its gap is timing-only, not a counter jump

    // A third injected fault: EngineData (0x180) has exactly one checksum
    // mismatch, at t=600ms (found by exhaustive cross-check against the
    // sample data -- see DESIGN.md). Every other message's checksums
    // should validate on every frame.
    CHECK(finalSnap.health.at(0x180).checksum_faults == 1);
    for (const auto& [id, h] : finalSnap.health) {
        if (id == 0x180) continue;
        CHECK(h.checksum_faults == 0);
    }
}

} // namespace

int main() {
    test_intel_unsigned_16bit();
    test_intel_unsigned_nibble();
    test_motorola_signed_16bit_positive();
    test_motorola_signed_16bit_negative();
    test_motorola_vs_intel_disagree_on_same_bytes();
    test_checksum_xor_excludes_own_byte();
    test_dbc_text_parses_real_file();
    test_frame_log_parses_real_file();
    test_known_gap_and_counter_fault_are_detected();

    std::printf("\n%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
