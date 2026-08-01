// Minimal, dependency-free unit tests (same style as task1's tests --
// no external framework, just CHECK macros and a pass/fail summary).

#include "signal_codec.hpp"
#include "diagnostics.hpp"
#include "dbc_database.hpp"

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

void test_intel_unsigned_decode() {
    // rpm: start_bit=0, length=16, intel, scale=0.25.
    // Bytes: 9C 0C -> little-endian 16-bit = 0x0C9C = 3228; *0.25 = 807.0
    FramePayload data = {0x9C, 0x0C, 0x00, 0x90, 0x00, 0x00, 0x00, 0x00};
    SignalDef rpm;
    rpm.start_bit = 0; rpm.length = 16; rpm.scale = 0.25; rpm.offset = 0; rpm.is_signed = false;
    double val = SignalCodec::decodePhysical(data, rpm, ByteOrder::Intel);
    CHECK(val == 807.0);
}

void test_intel_counter_and_checksum_fields() {
    // counter: start_bit=16,length=4 -> nibble at byte2 bits0-3.
    FramePayload data = {0x9C, 0x0C, 0x05, 0x90, 0x00, 0x00, 0x00, 0x00};
    SignalDef counter;
    counter.start_bit = 16; counter.length = 4; counter.scale = 1; counter.is_signed = false;
    uint64_t c = SignalCodec::extractRaw(data, counter, ByteOrder::Intel);
    CHECK(c == 0x5);
}

void test_motorola_signed_decode() {
    // steering_angle: start_bit=0,length=16,motorola,signed,scale=0.1.
    // Bytes FF 8B -> big-endian 16-bit = 0xFF8B, signed = -117; *0.1 = -11.7
    FramePayload data = {0xFF, 0x8B, 0x00, 0x74, 0x00, 0x00, 0x00, 0x00};
    SignalDef steer;
    steer.start_bit = 0; steer.length = 16; steer.scale = 0.1; steer.offset = 0; steer.is_signed = true;
    double val = SignalCodec::decodePhysical(data, steer, ByteOrder::Motorola);
    CHECK(val > -11.71 && val < -11.69);
}

void test_motorola_counter_nibble() {
    // counter: start_bit=16,length=4 -> top nibble of byte2 in motorola scheme.
    FramePayload data = {0xFF, 0x8B, 0xA0, 0x74, 0x00, 0x00, 0x00, 0x00};
    SignalDef counter;
    counter.start_bit = 16; counter.length = 4; counter.scale = 1; counter.is_signed = false;
    uint64_t c = SignalCodec::extractRaw(data, counter, ByteOrder::Motorola);
    CHECK(c == 0xA);
}

void test_checksum_xor_matches_sample_frame() {
    // Real sample frame for 0x2B0 at t=0: FF 8B 00 74 00 00 00 00.
    // Checksum byte is byte index 3 (start_bit 24 / 8). XOR of the other
    // 7 bytes should equal 0x74.
    FramePayload data = {0xFF, 0x8B, 0x00, 0x74, 0x00, 0x00, 0x00, 0x00};
    uint8_t computed = 0;
    for (int i = 0; i < 8; ++i) {
        if (i == 3) continue;
        computed ^= data[i];
    }
    CHECK(computed == 0x74);
}

void test_signed_sign_extension_negative_small_field() {
    // A 4-bit signed field with raw value 0b1000 (=8) should sign-extend
    // to -8.
    FramePayload data = {0x08, 0, 0, 0, 0, 0, 0, 0}; // low nibble = 0x8
    SignalDef sig;
    sig.start_bit = 0; sig.length = 4; sig.scale = 1; sig.offset = 0; sig.is_signed = true;
    int64_t val = SignalCodec::extractSigned(data, sig, ByteOrder::Intel);
    CHECK(val == -8);
}

void test_diagnostics_counter_gap_detection() {
    DbcDatabase db; // empty DB is fine; we build MessageDef by hand
    MessageDef msg;
    msg.id = 0x220;
    msg.name = "VehicleSpeed";
    msg.period_ms = 20;
    msg.byte_order = ByteOrder::Intel;
    SignalDef counter;
    counter.name = "counter"; counter.start_bit = 20; counter.length = 4; counter.scale = 1;
    msg.signals.push_back(counter);
    SignalDef checksum;
    checksum.name = "checksum"; checksum.start_bit = 24; checksum.length = 8; checksum.scale = 1;
    msg.signals.push_back(checksum);

    DiagnosticsEngine diag(db);

    RawFrame f1;
    f1.timestamp_ms = 0; f1.can_id = 0x220; f1.dlc = 8;
    f1.data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // counter nibble=0
    f1.data[3] = 0; // checksum byte (will mismatch, that's fine, not under test here)

    RawFrame f2 = f1;
    f2.timestamp_ms = 20;
    f2.data[2] = 0x20; // counter nibble bits20-23 -> byte2 high nibble = 2 (expected would be 1)

    auto a1 = diag.onKnownFrame(msg, f1);
    auto a2 = diag.onKnownFrame(msg, f2);

    bool foundGap = false;
    for (const auto& a : a2) {
        if (a.kind == AnomalyKind::CounterGap) foundGap = true;
    }
    CHECK(foundGap);
}

void test_diagnostics_unknown_id_does_not_crash() {
    DbcDatabase db;
    DiagnosticsEngine diag(db);
    RawFrame f;
    f.timestamp_ms = 5; f.can_id = 0x999; f.dlc = 8;
    Anomaly a = diag.onUnknownFrame(f);
    CHECK(a.kind == AnomalyKind::UnknownId);
    CHECK(diag.unknownFrameCount() == 1);
}

} // namespace

int main() {
    test_intel_unsigned_decode();
    test_intel_counter_and_checksum_fields();
    test_motorola_signed_decode();
    test_motorola_counter_nibble();
    test_checksum_xor_matches_sample_frame();
    test_signed_sign_extension_negative_small_field();
    test_diagnostics_counter_gap_detection();
    test_diagnostics_unknown_id_does_not_crash();

    std::printf("\n%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
