#ifndef LUNA_PROTOCOL_H
#define LUNA_PROTOCOL_H

#include <HardwareSerial.h>

/*
 * Protocol Format for MCU - Raspberry Pi Camera Communication
 *
 * Format (4-byte payload)
 *
 * <START_BYTE>
 * <CMD>
 * <STAT>
 * <STOP_BYTE>
 */

namespace luna::proto {
    static constexpr uint8_t START_BYTE     = 0xAA;
    static constexpr uint8_t STOP_BYTE      = 0x55;

    static constexpr uint8_t CMD_PING       = 0x00;
    static constexpr uint8_t CMD_START      = 0x01;
    static constexpr uint8_t CMD_STOP       = 0x02;
    static constexpr uint8_t CMD_GETSTAT    = 0x03;
    static constexpr uint8_t CMD_RESPONSE   = 0x04;

    static constexpr uint8_t STAT_RECORDING = 0x01;
    static constexpr uint8_t STAT_STANDBY   = 0x02;

    static constexpr uint8_t INVALID        = 0xFF;

    inline const char *camera_status_string(const uint8_t status) {
        switch (status) {
            case 0b00:
                return "X";  // Unknown
            case STAT_RECORDING:
                return "R";  // Recording
            case STAT_STANDBY:
                return "S";  // Standby
            case 0b11:
                return "Z";  // Unknown
            default:
                __builtin_unreachable();
        }
    }

    class raspi_cam {
    private:
        HardwareSerial &m_hw_serial;
        uint8_t m_status   = INVALID;
        uint32_t m_counter = 0;

    public:
        explicit raspi_cam(HardwareSerial &hw_serial)
            : m_hw_serial{hw_serial} {}

        void ping() const { tx_cmd(CMD_PING); }

        void start_video() const { tx_cmd(CMD_START); }

        void stop_video() const { tx_cmd(CMD_STOP); }

        void get_status() const { tx_cmd(CMD_GETSTAT); }

        [[nodiscard]] uint8_t wait(const uint32_t timeout_ms) const {
            const uint32_t t0 = millis();

            while (millis() - t0 < timeout_ms) {
                if (m_hw_serial.available() >= 4) {
                    if (m_hw_serial.read() != START_BYTE) return INVALID;
                    if (m_hw_serial.read() != CMD_RESPONSE) return INVALID;
                    const uint8_t code = m_hw_serial.read();
                    if (m_hw_serial.read() != STOP_BYTE)
                        return INVALID;
                    return code;
                }
            }

            return INVALID;
        }

        void set_status(const uint8_t status) {
            m_status = status;
            ++m_counter;
        }

        [[nodiscard]] constexpr const uint8_t &status() const { return m_status; }

        [[nodiscard]] constexpr const uint32_t &counter() const { return m_counter; }

    private:
        void tx_cmd(const uint8_t byte, const uint8_t data = INVALID) const {
            m_hw_serial.write(START_BYTE);
            m_hw_serial.write(byte);
            m_hw_serial.write(data);
            m_hw_serial.write(STOP_BYTE);
        }
    };
}  // namespace luna::proto

#endif  //LUNA_PROTOCOL_H
