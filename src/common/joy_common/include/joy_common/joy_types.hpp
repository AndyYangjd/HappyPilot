#ifndef COMMON__JOY_COMMON__JOY_TYPES_HPP_
#define COMMON__JOY_COMMON__JOY_TYPES_HPP_

#include <cstdint>
#include <vector>


namespace joypilot{
namespace common{
namespace types{
    using bool8_t = bool;
    using char8_t = char;
    using uchar8_t = unsigned char;
    using float32_t = float;
    using float64_t = double;

    // pi = tau / 2
    constexpr float32_t PI = 3.14159265359F;
    // pi/2
    constexpr float32_t PI_2 = 1.5707963267948966F;
    // tau = 2 pi
    constexpr float32_t TAU = 6.283185307179586476925286766559F;
    // arbitrary small constant: 1.0E-6F
    constexpr float32_t FEPS = 0.000001F;

    struct PointXYZIF{
        float32_t x{0};
        float32_t y{0};
        float32_t z{0};
        float32_t intensity{0};
        uint16_t id{0};
        static constexpr uint16_t END_OF_SCAN_ID = 65535u;
    };

    struct PointXYZF{
        float32_t x{0};
        float32_t y{0};
        float32_t z{0};
        uint16_t id{0};
        static constexpr uint16_t END_OF_SCAN_ID = 65535u;
    };
    using PointBlock = std::vector<PointXYZIF>;
    using PointPtrBlock = std::vector<const PointXYZIF *>;
    // \brief Stores basic configuration information, does some simple validity checking
    static constexpr uint16_t POINT_BLOCK_CAPACITY = 512U;

}  // namespace types
}  // namespace common
}  // namespace joypilot

#endif //COMMON__JOY_COMMON__JOY_TYPES_HPP_
