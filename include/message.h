#ifndef LUNA_FIRMWARE_MESSAGE_H
#define LUNA_FIRMWARE_MESSAGE_H

#include <concepts>

namespace luna {
    enum class payload_type : uint8_t {
        DATA = 0,
        COMMAND,
    };

    enum class transmit_direction : uint8_t {
        DOWNLINK = 0,
        UPLINK,
    };

    enum class command_type : uint8_t {
        PING = 0,
        PONG,

        SET_STATE = 16,
    };

    struct header_t {
        // 32 bit team identifier
        char team_id[4];

        // 32 bit header information
        uint8_t hwid;
        payload_type type;
        transmit_direction direction;
        uint8_t _padding;

        // 32 bit payload size information
        uint32_t data_size;

        // 32 bit checksum
        uint32_t data_checksum;
    };

    struct data_t {
    };

    struct command_t {
    };

    struct message_t {
        header_t header;
        void *data;
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
