#ifndef JMS_UTILS_VIZ_COLOR_H
#define JMS_UTILS_VIZ_COLOR_H


#include <bit>
#include <type_traits>


namespace jms {
namespace utils {
namespace viz {


struct alignas(uint32_t) Color_big_endian {
  uint8_t alpha {0xff};
  uint8_t blue  {0x00};
  uint8_t green {0x00};
  uint8_t red   {0x00};
  constexpr Color_big_endian(void) noexcept = default;
  constexpr Color_big_endian(uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha=0xff) noexcept : alpha(alpha), blue(blue), green(green), red(red) { return; }
  constexpr uint32_t Value(void) const noexcept { return *reinterpret_cast<const uint32_t*>(this); }
};


struct alignas(uint32_t) Color_little_endian {
  uint8_t red   {0x00};
  uint8_t green {0x00};
  uint8_t blue  {0x00};
  uint8_t alpha {0xff};
  constexpr Color_little_endian(void) noexcept = default;
  constexpr Color_little_endian(uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha=0xff) noexcept : red(red), green(green), blue(blue), alpha(alpha) { return; }
  constexpr uint32_t Value(void) const noexcept { return *reinterpret_cast<const uint32_t*>(this); }
};


using Color = std::conditional<std::endian::native == std::endian::little, Color_little_endian, Color_big_endian>::type;


constexpr Color COLOR_BLACK {0x00, 0x00, 0x00, 0xff};
constexpr Color COLOR_BLUE  {0x00, 0x00, 0xff, 0xff};
constexpr Color COLOR_GREEN {0x00, 0xff, 0x00, 0xff};
constexpr Color COLOR_RED   {0xff, 0x00, 0x00, 0xff};
constexpr Color COLOR_WHITE {0xff, 0xff, 0xff, 0xff};


} // namespace viz
} // namespace utils
} // namespace jms


#endif // JMS_UTILS_VIZ_COLOR_H
