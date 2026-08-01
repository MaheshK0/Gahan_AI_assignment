#include "uart_parser.hpp"
#include "crc16.hpp"

namespace uart {

void UartParser::feed(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        feed(data[i]);
    }
}

void UartParser::toWaitSof1(bool countSyncLoss) {
    if (countSyncLoss && in_flight_) {
        // We had already committed to a packet (past SOF2) and are
        // abandoning it due to a framing/CRC error -- that's a genuine
        // loss of synchronization, distinct from just skipping raw
        // garbage bytes that were never part of a packet attempt.
        ++stats_.sync_losses;
    }
    state_ = State::WaitSof1;
    in_flight_ = false;
    payload_index_ = 0;
}

void UartParser::handleFramingError() {
    toWaitSof1(/*countSyncLoss=*/true);
}

void UartParser::feed(uint8_t byte) {
    switch (state_) {
        case State::WaitSof1:
            if (byte == kSof1) {
                state_ = State::WaitSof2;
            } else {
                ++stats_.discarded_bytes;
            }
            break;

        case State::WaitSof2:
            if (byte == kSof2) {
                state_ = State::Length;
                in_flight_ = true;
            } else if (byte == kSof1) {
                // Stay hunting: this byte could itself be the start of the
                // real SOF1/SOF2 pair. The previous SOF1 was a false start.
                ++stats_.discarded_bytes;
                state_ = State::WaitSof2; // remain, byte acts as new SOF1
            } else {
                ++stats_.discarded_bytes;
                state_ = State::WaitSof1;
            }
            break;

        case State::Length:
            length_ = byte;
            payload_index_ = 0;
            running_crc_ = Crc16Ccitt::kInit;
            state_ = State::Type;
            break;

        case State::Type:
            type_ = byte;
            running_crc_ = Crc16Ccitt::update(running_crc_, byte);
            state_ = (length_ == 0) ? State::CrcLo : State::Payload;
            break;

        case State::Payload:
            payload_[payload_index_++] = byte;
            running_crc_ = Crc16Ccitt::update(running_crc_, byte);
            if (payload_index_ >= length_) {
                state_ = State::CrcLo;
            }
            break;

        case State::CrcLo:
            crc_lo_ = byte;
            state_ = State::CrcHi;
            break;

        case State::CrcHi: {
            // CRC is transmitted little-endian: low byte first, high byte
            // second (see header comment / DESIGN.md for how this was
            // determined).
            uint16_t received_crc = static_cast<uint16_t>((byte << 8) | crc_lo_);
            if (received_crc == running_crc_) {
                ++stats_.valid_packets;
                Packet pkt;
                pkt.length = length_;
                pkt.type = type_;
                pkt.payload = (length_ > 0) ? payload_.data() : nullptr;
                if (on_packet_) {
                    on_packet_(pkt);
                }
                toWaitSof1(/*countSyncLoss=*/false);
            } else {
                ++stats_.crc_failures;
                handleFramingError();
            }
            break;
        }
    }
}

void UartParser::finish() {
    if (state_ != State::WaitSof1) {
        // Anything other than idly waiting for a new SOF1 means we were in
        // the middle of receiving a packet when input ended.
        ++stats_.partial_packets;
    }
    reset();
}

void UartParser::reset() {
    state_ = State::WaitSof1;
    length_ = 0;
    type_ = 0;
    payload_index_ = 0;
    running_crc_ = 0;
    crc_lo_ = 0;
    in_flight_ = false;
}

} // namespace uart
