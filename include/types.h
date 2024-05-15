#ifndef LUNA_FIRMWARE_TYPES_H
#define LUNA_FIRMWARE_TYPES_H

namespace vt::types {
    template<typename Tp, size_t Size>
    using static_array = Tp[Size];
}  // namespace vt::types

#endif  //LUNA_FIRMWARE_TYPES_H
