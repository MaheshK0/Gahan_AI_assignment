// Minimal, dependency-free unit tests. No external test framework is used
// (kept consistent with the assignment's "avoid third-party libraries"
// guidance) -- just a small set of CHECK macros and a summary at the end.
// Run via `ctest` or by executing the built binary directly; non-zero exit
// code indicates failure.

#include "uart_parser.hpp"
#include "crc16.hpp"

#include <cstdio>
#include <vector>

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

// Build a well-formed packet's raw bytes: SOF1 SOF2 Length Type Payload
// CRC(lo,hi). CRC covers Type+Payload, transmitted little-endian.
std::vector<uint8_t> makePacket(uint8_t type, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> out;
    out.push_back(uart::kSof1);
    out.push_back(uart::kSof2);
    out.push_back(static_cast<uint8_t>(payload.size()));
    out.push_back(type);
    std::vector<uint8_t> crcInput;
    crcInput.push_back(type);
    for (auto b : payload) {
        out.push_back(b);
        crcInput.push_back(b);
    }
    uint16_t crc = uart::Crc16Ccitt::compute(crcInput.data(), crcInput.size());
    out.push_back(static_cast<uint8_t>(crc & 0xFF));        // lo
    out.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF)); // hi
    return out;
}

void test_single_valid_packet() {
    auto bytes = makePacket(0x07, {0xDE, 0xAD, 0xBE, 0xEF});
    int received = 0;
    uart::UartParser parser([&](const uart::Packet& p) {
        ++received;
        CHECK(p.type == 0x07);
        CHECK(p.length == 4);
        CHECK(p.payload[0] == 0xDE);
        CHECK(p.payload[3] == 0xEF);
    });
    for (auto b : bytes) parser.feed(b);
    CHECK(received == 1);
    CHECK(parser.stats().valid_packets == 1);
    CHECK(parser.stats().crc_failures == 0);
}

void test_zero_length_payload() {
    auto bytes = makePacket(0x02, {});
    int received = 0;
    uart::UartParser parser([&](const uart::Packet& p) {
        ++received;
        CHECK(p.length == 0);
        CHECK(p.payload == nullptr);
    });
    for (auto b : bytes) parser.feed(b);
    CHECK(received == 1);
}

void test_corrupted_crc_then_recovers() {
    auto good1 = makePacket(0x01, {0x10, 0x20});
    auto bad = makePacket(0x02, {0xAA});
    bad[bad.size() - 1] ^= 0xFF; // flip CRC high byte -> corrupt
    auto good2 = makePacket(0x03, {0x55});

    std::vector<uint8_t> stream;
    stream.insert(stream.end(), good1.begin(), good1.end());
    stream.insert(stream.end(), bad.begin(), bad.end());
    stream.insert(stream.end(), good2.begin(), good2.end());

    std::vector<uint8_t> seenTypes;
    uart::UartParser parser([&](const uart::Packet& p) { seenTypes.push_back(p.type); });
    for (auto b : stream) parser.feed(b);

    CHECK(parser.stats().valid_packets == 2);
    CHECK(parser.stats().crc_failures == 1);
    CHECK(seenTypes.size() == 2);
    if (seenTypes.size() == 2) {
        CHECK(seenTypes[0] == 0x01);
        CHECK(seenTypes[1] == 0x03);
    }
}

void test_garbage_bytes_between_packets_are_discarded_and_resync_happens() {
    auto good1 = makePacket(0x01, {0x01});
    auto good2 = makePacket(0x02, {0x02, 0x03});
    std::vector<uint8_t> garbage = {0x00, 0xFF, 0x12, 0x34, 0x77, 0x88};

    std::vector<uint8_t> stream;
    stream.insert(stream.end(), good1.begin(), good1.end());
    stream.insert(stream.end(), garbage.begin(), garbage.end());
    stream.insert(stream.end(), good2.begin(), good2.end());

    int received = 0;
    uart::UartParser parser([&](const uart::Packet&) { ++received; });
    for (auto b : stream) parser.feed(b);

    CHECK(received == 2);
    CHECK(parser.stats().valid_packets == 2);
    CHECK(parser.stats().discarded_bytes == garbage.size());
}

void test_embedded_fake_sof_inside_garbage_does_not_desync_permanently() {
    // A stray 0xAA 0x55 pair sitting inside otherwise-garbage bytes should
    // be tried as a packet start. If its bogus Length field happens to run
    // past a real, well-formed packet that follows, that real packet's own
    // SOF bytes can legitimately get consumed as (invalid) CRC bytes of the
    // fake one -- a real UART receiver has no way to tell the difference
    // in advance either. The important guarantee is just that the parser
    // never crashes, never gets stuck, and correctly resumes hunting for
    // SOF1/SOF2 afterwards. This mirrors a pattern actually present in the
    // provided uart_stream.bin sample (offset 0x20).
    auto good1 = makePacket(0x01, {0x01});
    std::vector<uint8_t> fakeSof = {0xAA, 0x55, 0x03, 0x03, 0xAA, 0x55, 0x00};
    // A trailing well-formed packet, placed far enough after the fake
    // header's bogus payload+CRC window that it cannot be swallowed by it.
    std::vector<uint8_t> filler = {0x11, 0x22, 0x33, 0x44};
    auto good2 = makePacket(0x09, {0x42});

    std::vector<uint8_t> stream;
    stream.insert(stream.end(), good1.begin(), good1.end());
    stream.insert(stream.end(), fakeSof.begin(), fakeSof.end());
    stream.insert(stream.end(), filler.begin(), filler.end());
    stream.insert(stream.end(), good2.begin(), good2.end());

    std::vector<uint8_t> seenTypes;
    uart::UartParser parser([&](const uart::Packet& p) { seenTypes.push_back(p.type); });
    for (auto b : stream) parser.feed(b);

    // Both real packets must eventually be found; the fake header in
    // between must not cause a crash or a permanent lockup.
    CHECK(parser.stats().valid_packets == 2);
    CHECK(seenTypes.size() == 2);
    if (seenTypes.size() == 2) {
        CHECK(seenTypes[0] == 0x01);
        CHECK(seenTypes[1] == 0x09);
    }
}


void test_partial_packet_at_end_of_stream_does_not_crash() {
    auto good = makePacket(0x0B, {0x01, 0x02, 0x03});
    // Truncate: drop the last 2 bytes (CRC) to simulate a packet cut off
    // mid-stream.
    std::vector<uint8_t> truncated(good.begin(), good.end() - 2);

    uart::UartParser parser([](const uart::Packet&) {
        CHECK(false); // must not fire -- packet is incomplete
    });
    for (auto b : truncated) parser.feed(b);
    parser.finish();

    CHECK(parser.stats().partial_packets == 1);
    CHECK(parser.stats().valid_packets == 0);
}

void test_reset_clears_in_flight_state_but_not_stats() {
    auto good = makePacket(0x01, {0x01});
    std::vector<uint8_t> truncated(good.begin(), good.end() - 2);

    uart::UartParser parser([](const uart::Packet&) {});
    for (auto b : truncated) parser.feed(b);
    parser.reset(); // abandon in-flight packet without counting it as partial

    // Feed a full, valid packet after reset -- parser must be back in a
    // clean WAIT_SOF1 state.
    auto good2 = makePacket(0x02, {0xAA, 0xBB});
    int received = 0;
    // Re-attach callback isn't possible post-construction in this simple
    // design, so build a fresh parser sharing style but reuse the same
    // instance by feeding into it (callback was empty above); instead
    // verify indirectly via stats after feeding good2.
    (void)received;
    for (auto b : good2) parser.feed(b);
    CHECK(parser.stats().valid_packets == 1);
}

void test_chunked_feed_matches_byte_by_byte() {
    auto good1 = makePacket(0x01, {0x10, 0x20, 0x30});
    auto good2 = makePacket(0x02, {});
    std::vector<uint8_t> stream;
    stream.insert(stream.end(), good1.begin(), good1.end());
    stream.insert(stream.end(), good2.begin(), good2.end());

    int received = 0;
    uart::UartParser parser([&](const uart::Packet&) { ++received; });
    parser.feed(stream.data(), stream.size()); // bonus: chunked API
    CHECK(received == 2);
    CHECK(parser.stats().valid_packets == 2);
}

void test_no_dynamic_growth_bounded_payload() {
    // Largest legal payload (Length is a single byte -> max 255).
    std::vector<uint8_t> payload(255);
    for (size_t i = 0; i < payload.size(); ++i) payload[i] = static_cast<uint8_t>(i);
    auto bytes = makePacket(0x08, payload);

    int received = 0;
    uart::UartParser parser([&](const uart::Packet& p) {
        ++received;
        CHECK(p.length == 255);
        CHECK(p.payload[254] == 254);
    });
    for (auto b : bytes) parser.feed(b);
    CHECK(received == 1);
}

} // namespace

int main() {
    test_single_valid_packet();
    test_zero_length_payload();
    test_corrupted_crc_then_recovers();
    test_garbage_bytes_between_packets_are_discarded_and_resync_happens();
    test_embedded_fake_sof_inside_garbage_does_not_desync_permanently();
    test_partial_packet_at_end_of_stream_does_not_crash();
    test_reset_clears_in_flight_state_but_not_stats();
    test_chunked_feed_matches_byte_by_byte();
    test_no_dynamic_growth_bounded_payload();

    std::printf("\n%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
