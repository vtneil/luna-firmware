#ifndef LUNA_FIRMWARE_MESSAGE_H
#define LUNA_FIRMWARE_MESSAGE_H

#include <concepts>

namespace luna {
    struct sensor_data_t {
        // Message Time
        uint32_t timestamp;
        uint32_t pc;

        // State
        state_t ps;
        uint8_t pyro_a;
        uint8_t pyro_b;
        uint8_t pyro_c;

        // Internal
        int32_t temp_cpu;

        // GNSS
        double gps_lat;
        double gps_lon;
        float gps_alt;

        // MS1
        float ms1_pres;
        float ms1_temp;

        // MS2
        float ms2_pres;
        float ms2_temp;
    };
}  // namespace luna

namespace luna {
    enum class state_t : uint8_t;

    enum class payload_type : uint8_t {
        DATA = 0,
        COMMAND,
    };

    enum class transmit_direction : uint8_t {
        DOWNLINK = 0,
        UPLINK,
    };

    enum class command_type : uint16_t {
        PING = 0,
        PONG,

        SET_STATE = 16,
    };

    // Fixed-length 16 byte (128 bit) 4-aligned command
    struct header_t {
        // 32 bit team identifier
        char team_id[4];

        // 32 bit header information
        uint8_t hwid;
        payload_type type;
        transmit_direction direction;
        uint8_t aux;

        // 32 bit payload size information
        uint32_t data_size;

        // 32 bit checksum
        uint32_t data_checksum;
    };

    struct data_t {
        uint32_t timestamp;
        uint32_t pc;
        float gps_lat;
        float gps_lon;
        state_t ps;
        int32_t temp_cpu;
    };

    // Fixed-length 64 byte (512 bit) 8-aligned command
    struct command_t {
        // 64 bit aligned 16-bit command
        command_type command;
        uint16_t aux[3];

        // 64 bit command registers
        union {
            uint64_t u64;
            int64_t i64;
            uint32_t u32[2];
            int32_t i32[2];
            uint16_t u16[4];
            int16_t i16[4];
            uint8_t u8[8];
            int8_t i8[8];
            double fp64;
            float fp32[2];
        } r[7];
    };

    namespace detail {
        template<typename T>
        concept payload_type = std::same_as<T, data_t> || std::same_as<T, command_t>;
    }

    struct message_t {
        header_t header;
        void *data;

        template<detail::payload_type PayloadType>
        PayloadType &get() {
            return *static_cast<PayloadType *>(data);
        }
    };

    template<typename InputType, std::integral OutputType>
    class crypto_checksum {
    public:
        OutputType operator()(const InputType &input) const {
            OutputType output;
            calc_checksum_impl(input, &output);
            return output;
        }

    protected:
        void calc_checksum_impl(const InputType &input_data, OutputType *const result) const {
            static_assert(sizeof(InputType) >= sizeof(OutputType), "Can\'t calculate checksum of this data type.");

            using byte_t                = uint8_t;
            constexpr OutputType ZERO   = 0;
            constexpr OutputType ONE    = 1;
            constexpr OutputType MASK   = ZERO - ONE;
            constexpr size_t Q          = sizeof(InputType) / sizeof(OutputType);
            constexpr size_t R          = sizeof(InputType) % sizeof(OutputType);
            constexpr size_t BITS_SHIFT = 4 * sizeof(OutputType);
            constexpr OutputType MASK_H = MASK << BITS_SHIFT;
            constexpr OutputType MASK_L = ~MASK_H;

            // Initialization
            byte_t input_as_byte_t[sizeof(InputType)] = {};
            memcpy(input_as_byte_t, &input_data, sizeof(InputType));

            byte_t result_as_byte_t[sizeof(OutputType)] = {};
            OutputType result_tmp;

            // Full XOR
            for (size_t i = 0; i < Q; ++i)
                *reinterpret_cast<OutputType *const>(result_as_byte_t) ^= reinterpret_cast<const OutputType *const>(input_as_byte_t)[i];

            // Partial XOR for remainder
            for (size_t i = 0; i < R; ++i)
                result_as_byte_t[i] ^= input_as_byte_t[Q * sizeof(OutputType) + i];
            result_tmp = *reinterpret_cast<const OutputType *const>(result_as_byte_t);

            // XOR Shifting Left to Right
            for (size_t i = 8 * sizeof(OutputType) - 1; i > 0; --i)
                result_tmp ^= result_tmp << i;
            for (size_t i = 1; i < 8 * sizeof(OutputType); ++i)
                result_tmp ^= result_tmp >> i;

            // XOR with its reverse bits
            OutputType result_rev_tmp = {};
            for (size_t i = 0; i < 8 * sizeof(OutputType); ++i)
                if (result_tmp & (ONE << i))
                    result_rev_tmp |= 1 << ((8 * sizeof(OutputType) - 1) - i);
            result_tmp ^= result_rev_tmp;

            // Swap first half bytes/nibbles with not second half bytes/nibbles
            result_tmp = ((result_tmp & MASK_L) << BITS_SHIFT) |
                         ((result_tmp & MASK_H) >> BITS_SHIFT);

            // Invert bits
            result_tmp = ~result_tmp;

            // Return value
            *result = result_tmp;
        }
    };
}  // namespace luna

#endif  //LUNA_FIRMWARE_MESSAGE_H
