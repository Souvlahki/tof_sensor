#include "tof_hardware/tof_serial.hpp"

#include<cstring>
#include <stdexcept>
#include <cstdint>
#include "tof_hardware/cobsr.h"

TofSerial::~TofSerial()
{
    Close();
}

LibSerial::BaudRate TofSerial::BaudRateFromInt(int baud)
{
    switch (baud)
    {
    case 9600:
        return LibSerial::BaudRate::BAUD_9600;
    case 19200:
        return LibSerial::BaudRate::BAUD_19200;
    case 38400:
        return LibSerial::BaudRate::BAUD_38400;
    case 57600:
        return LibSerial::BaudRate::BAUD_57600;
    case 115200:
        return LibSerial::BaudRate::BAUD_115200;
    case 230400:
        return LibSerial::BaudRate::BAUD_230400;
    default:
        throw std::invalid_argument("Unsupported baud rate: " + std::to_string(baud));
    }
}

uint32_t TofSerial::CalculateChecksum(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    size_t word_count = len / 4; // len must be a multiple of 4 — see note below

    for (size_t i = 0; i < word_count; i++)
    {
        uint32_t word = (uint32_t)data[i * 4] | ((uint32_t)data[i * 4 + 1] << 8) | ((uint32_t)data[i * 4 + 2] << 16) | ((uint32_t)data[i * 4 + 3] << 24);

        crc ^= word;
        for (int bit = 0; bit < 32; bit++)
        {
            crc = (crc & 0x80000000u) ? (crc << 1) ^ 0x04C11DB7u : (crc << 1);
        }
    }

    return crc;
}

void TofSerial::Init(const TofSerialConfig &config)
{
    // Close any previously-open port first so Init() can double as "reopen
    // with new settings" without the caller needing to know that.
    Close();

    port_.Open(config.port);
    port_.SetBaudRate(BaudRateFromInt(config.baud_rate));
    port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    port_.SetParity(LibSerial::Parity::PARITY_NONE);
    port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

    config_ = config;
    rx_buffer_.clear();
}

void TofSerial::Close()
{
    if (port_.IsOpen())
    {
        port_.Close();
    }
    rx_buffer_.clear();
}

bool TofSerial::IsOpen() const
{
    return port_.IsOpen();
}

TofReadStatus TofSerial::ProcessBuffer(TxFrame &out_frame) const
{
    uint8_t decoded[sizeof(TxFrame)];

    auto result = cobsr_decode(
        decoded,
        sizeof(decoded),
        rx_buffer_.data(),
        rx_buffer_.size());

    if (result.status != COBSR_DECODE_OK)
    {
        return TofReadStatus::DecodeError;
    }

    if (result.out_len != sizeof(TxFrame))
    {
        return TofReadStatus::DecodeError;
    }

    TxFrame frame;
    std::memcpy(&frame, decoded, sizeof(frame));

    if (frame.checksum != CalculateChecksum(
                              reinterpret_cast<const uint8_t *>(&frame),
                              offsetof(TxFrame, checksum)))
    {
        return TofReadStatus::ChecksumError;
    }

    out_frame = frame;
    return TofReadStatus::Packet;
}

TofReadStatus TofSerial::ReadPacket(TxFrame &out_frame)
{
    while (port_.IsDataAvailable())
    {
        char byte;

        try
        {
            port_.ReadByte(byte, 0);
        }
        catch (const LibSerial::ReadTimeout &)
        {
            return TofReadStatus::NoPacket;
        }
        catch (const std::exception &)
        {
            return TofReadStatus::SerialError;
        }

        const uint8_t b = static_cast<uint8_t>(byte);

        if (b == 0)
        {
            // Ignore empty frames caused by consecutive delimiters.
            if (rx_buffer_.empty())
            {
                continue;
            }

            const TofReadStatus status = ProcessBuffer(out_frame);
            rx_buffer_.clear();

            if (status == TofReadStatus::Packet)
            {
                return status;
            }

            // CRC error or decode failure. Keep looking for the next packet.
            continue;
        }

        rx_buffer_.push_back(b);

        // Protect against losing synchronization.
        if (rx_buffer_.size() > sizeof(TxFrame) + 8)
        {
            rx_buffer_.clear();
        }
    }
    return TofReadStatus::NoPacket;
}

bool TofSerial::SendData(const RxFrame &frame_in)
{
    if (!port_.IsOpen())
    {
        return false;
    }

    RxFrame frame = frame_in;
    frame.checksum = CalculateChecksum(
        reinterpret_cast<const uint8_t *>(&frame),
        offsetof(RxFrame, checksum));

    cobsr_encode_result res = cobsr_encode(
        tx_buf_, sizeof(tx_buf_),
        reinterpret_cast<const uint8_t *>(&frame), sizeof(frame));

    if (res.status != COBSR_ENCODE_OK)
    {
        return false;
    }

    tx_buf_[res.out_len] = 0x00; // COBS-R frame delimiter

    LibSerial::DataBuffer out(tx_buf_, tx_buf_ + res.out_len + 1);

    try
    {
        for (size_t i = 0; i < res.out_len + 1; ++i)
        {
            port_.WriteByte(static_cast<char>(tx_buf_[i]));
        }
    }
    catch (const std::exception &)
    {
        return false;
    }

    return true;
}