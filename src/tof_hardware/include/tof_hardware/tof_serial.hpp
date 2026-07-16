#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <libserial/SerialPort.h>

// Configuration needed to open and talk to the ToF serial bridge.
// Deliberately has no ROS types in it so this class can be unit tested /
// reused outside of an rclcpp::Node.
struct TofSerialConfig
{
    std::string port = "/dev/ttyACM0";
    int baud_rate = 115200;
};

#pragma pack(push, 1)
struct TofPacket
{
    uint8_t sensor_id;
    uint16_t range; // mm
    uint16_t checksum;
};
#pragma pack(pop)

enum class TofReadStatus
{
    NoPacket,      // read timed out or packet isn't complete yet -- just call again
    Packet,        // out_packet was filled with a valid, checksummed packet
    DecodeError,   // COBS decode failed, or decoded size didn't match TofPacket
    ChecksumError, // decoded fine, but checksum didn't match payload
    SerialError    // fatal I/O error on the port -- treat as disconnect
};

// Owns the LibSerial port and the COBS/checksum framing logic for the ToF
// bridge protocol. Knows nothing about ROS, publishers, or topics.
class TofSerial
{
public:
    TofSerial() = default;
    ~TofSerial();

    TofSerial(const TofSerial &) = delete;
    TofSerial &operator=(const TofSerial &) = delete;

    // Opens and configures the serial port per `config`. Throws
    // std::exception (or a LibSerial exception) on failure. Safe to call
    // again after Close() to reopen with new settings.
    void Init(const TofSerialConfig &config);

    void Close();
    bool IsOpen() const;

    const TofSerialConfig &GetConfig() const { return config_; }

    TofReadStatus ReadPacket(TofPacket &out_packet);

    static uint16_t CalculateChecksum(uint8_t sensor_id, uint16_t range);

private:
    static LibSerial::BaudRate BaudRateFromInt(int baud);

    // Attempts to COBS-decode + validate whatever is currently in
    // rx_buffer_ into out_packet. Caller is responsible for clearing
    // rx_buffer_ afterward.
    TofReadStatus ProcessBuffer(TofPacket &out_packet) const;

    LibSerial::SerialPort port_;
    TofSerialConfig config_;
    std::vector<uint8_t> rx_buffer_;
};