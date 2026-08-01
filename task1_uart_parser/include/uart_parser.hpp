#pragma once
#include <cstdint>
#include <cstddef>
#include <array>
#include <functional>

namespace uart {

// Maximum payload size the parser will accept. Length is a single byte in
// the wire format, so payload can never exceed 255 bytes; we size the
// internal buffer to that hard ceiling so all storage is static/bounded
// (no heap allocation anywhere on the hot path).
constexpr size_t kMaxPayloadLen = 255;

constexpr uint8_t kSof1 = 0xAA;
constexpr uint8_t kSof2 = 0x55;

// A single decoded, CRC-valid packet. Payload is exposed as a
// pointer+length into the parser's internal static buffer; it is only
// valid for the duration of the onPacket callback (see feed()).
struct Packet {
    uint8_t length = 0;   // payload length, as carried on the wire
    uint8_t type = 0;
    const uint8_t* payload = nullptr; // length bytes, may be nullptr if length==0
};

// Running counters exposed to the caller so a driver program can report
// on stream health.
struct ParserStats {
    uint64_t valid_packets = 0;
    uint64_t crc_failures = 0;
    uint64_t sync_losses = 0;     // times we had to search for a new SOF after
                                   // a previously-started packet was abandoned
    uint64_t discarded_bytes = 0; // bytes skipped while hunting for SOF1/SOF2
    uint64_t partial_packets = 0; // packets still in-flight when input ended
};

// Streaming, byte-at-a-time parser for the custom UART protocol:
//
//   SOF1(0xAA) | SOF2(0x55) | Length | Type | Payload[Length] | CRC16 (2 bytes)
//
// Notes on the wire format, determined by empirical verification against
// the provided uart_stream.bin sample (see DESIGN.md for the full writeup):
//   - CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, xor-out 0)
//   - The CRC is computed over (Type + Payload) only -- NOT Length, despite
//     the assignment text stating "Length + Type + Payload". Verified
//     against every packet in the sample stream; Length-inclusive coverage
//     does not validate for any packet, Type+Payload coverage validates for
//     every structurally well-formed packet in the sample.
//   - The two CRC bytes are transmitted little-endian (low byte first).
//
// Design:
//   - Explicit state machine (WAIT_SOF1 -> WAIT_SOF2 -> LENGTH -> TYPE ->
//     PAYLOAD -> CRC_LO -> CRC_HI), documented in DESIGN.md.
//   - feed(byte) advances the state machine by exactly one byte and returns
//     void; a decoded packet is delivered via the onPacket callback set in
//     the constructor (called synchronously, before feed() returns).
//   - All storage is a fixed-size member array; there is no dynamic
//     allocation anywhere in the class, so memory use is bounded and
//     predictable regardless of stream length or corruption.
//   - No global or shared mutable state: all state lives in the instance,
//     so multiple independent parsers can run concurrently.
//   - Any framing/CRC error simply drops back to hunting for the next
//     SOF1/SOF2 pair, one byte at a time -- the parser never enters a
//     fatal state and never throws.
class UartParser {
public:
    using PacketCallback = std::function<void(const Packet&)>;

    explicit UartParser(PacketCallback onPacket) : on_packet_(std::move(onPacket)) {}

    // Feed exactly one byte into the parser. May synchronously invoke the
    // onPacket callback zero or one times.
    void feed(uint8_t byte);

    // Convenience overload: feed an arbitrary-sized chunk of bytes
    // (bonus requirement -- not limited to single-byte input). Internally
    // this just calls feed() in a loop; provided so callers reading larger
    // buffers (e.g. from a file or socket) don't have to write that loop
    // themselves.
    void feed(const uint8_t* data, size_t len);

    // Must be called once the caller knows no more bytes are coming (e.g.
    // end of file / stream). If a packet was partially received, this
    // records it as a partial packet in the stats rather than leaving it
    // silently unreported.
    void finish();

    // Reset the parser to its initial state, discarding any in-flight
    // partial packet (bonus requirement). Statistics are NOT cleared by
    // reset(); call resetStats() separately if that's desired.
    void reset();

    void resetStats() { stats_ = ParserStats{}; }

    const ParserStats& stats() const { return stats_; }

private:
    enum class State {
        WaitSof1,
        WaitSof2,
        Length,
        Type,
        Payload,
        CrcLo,
        CrcHi,
    };

    void toWaitSof1(bool countSyncLoss);
    void handleFramingError(); // drop back to SOF hunt, counting sync loss

    PacketCallback on_packet_;
    ParserStats stats_{};

    State state_ = State::WaitSof1;
    uint8_t length_ = 0;
    uint8_t type_ = 0;
    size_t payload_index_ = 0;
    std::array<uint8_t, kMaxPayloadLen> payload_{};
    uint16_t running_crc_ = 0; // CRC accumulated over Type + Payload as bytes arrive
    uint16_t crc_lo_ = 0;
    bool in_flight_ = false; // true once we've consumed SOF1+SOF2 for the current packet
};

} // namespace uart
