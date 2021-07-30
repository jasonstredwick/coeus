#ifndef JMS_MATH_ANGLES_H
#define JMS_MATH_ANGLES_H


#include <cmath>
#include <ranges>
#include <type_traits>


namespace jms {
namespace math {
namespace angles {


// NOTE: Currently standard function are not constexpr
template <typename T> struct Sin   { T sin;   constexpr Sin  (T theta_rads) noexcept : sin  (std::sin  (theta_rads)) {} };
template <typename T> struct Cos   { T cos;   constexpr Cos  (T theta_rads) noexcept : cos  (std::cos  (theta_rads)) {} };
template <typename T> struct Tan   { T tan;   constexpr Tan  (T theta_rads) noexcept : tan  (std::tan  (theta_rads)) {} };
template <typename T> struct Asin  { T asin;  constexpr Asin (T theta_rads) noexcept : asin (std::asin (theta_rads)) {} };
template <typename T> struct Acos  { T acos;  constexpr Acos (T theta_rads) noexcept : acos (std::acos (theta_rads)) {} };
template <typename T> struct Atan  { T atan;  constexpr Atan (T theta_rads) noexcept : atan (std::atan (theta_rads)) {} };
template <typename T> struct Atan2 { T atan2; constexpr Atan2(T theta_rads) noexcept : atan2(std::atan2(theta_rads)) {} };
template <typename T> struct Sinh  { T sinh;  constexpr Sinh (T theta_rads) noexcept : sinh (std::sinh (theta_rads)) {} };
template <typename T> struct Cosh  { T cosh;  constexpr Cosh (T theta_rads) noexcept : cosh (std::cosh (theta_rads)) {} };
template <typename T> struct Tanh  { T tanh;  constexpr Tanh (T theta_rads) noexcept : tanh (std::tanh (theta_rads)) {} };
template <typename T> struct Asinh { T asinh; constexpr Asinh(T theta_rads) noexcept : asinh(std::asinh(theta_rads)) {} };
template <typename T> struct Acosh { T acosh; constexpr Acosh(T theta_rads) noexcept : acosh(std::acosh(theta_rads)) {} };
template <typename T> struct Atanh { T atanh; constexpr Atanh(T theta_rads) noexcept : atanh(std::atanh(theta_rads)) {} };
template <typename T, template <typename> class...ValueClasses>
struct Values : public ValueClasses<T>... {
  constexpr Values(void) noexcept : ValueClasses<T>(static_cast<T>(0))... { return; }
  constexpr Values(T theta_rads) noexcept : ValueClasses<T>(theta_rads)... { return; }
};


// Based on counterclockwise rotation sweeping from left to right.
template <typename T, template <typename> class...ValueClasses>
constexpr std::vector<Values<T, ValueClasses...>> Precompute(size_t num_angles, T start, T delta) noexcept {
  static_assert(!std::is_empty_v<Values<T, ValueClasses...>>, "Precompute requires at least one function; empty.");
  alignas(64) std::vector<Values<T, ValueClasses...>> values(num_angles); // must use parans to set num elements.
  for (size_t index : std::ranges::iota_view{static_cast<size_t>(0), num_angles}) {
    values[index] = std::move(Values<T, ValueClasses...> {start + (static_cast<T>(index) * delta)});
  }
  return values;
}


// TODO: Should be able to add complexity to precompute number of angles, direction, and unrolling angles.  Either merge
// with other Precompute function and/or add support function for computing angles.
/*
template <typename T, template <typename> class...ValueClasses>
requires std::same_as<T, T>
constexpr std::vector<Values<T, ValueClasses...>> Precompute(T start, T end, T delta) noexcept {
  static_assert(!std::is_empty_v(Values<T, ValueClasses...>), "Precompute requires at least one function; empty.");
  // Based on counterclockwise rotation sweeping from left to right.
  alignas(64) std::vector<Values<T, ValueClasses...>> values {};
  for (; start <= end; start+=delta) { values.push_back(Values<T, ValueClasses...> {start}); }
  return values;
}
*/


/*
template <typename T>
struct Foo {
  // Based on counterclockwise rotation sweeping from left to right.
  alignas(64) static const std::array<T, VIEW_RAYS> COS_ANGLES;
  alignas(64) static const std::array<T, VIEW_RAYS> SIN_ANGLES;
};


template <jms::math::SimType T>
const std::array<T, VIEW_RAYS> Sim<T>::COS_ANGLES{
  []() constexpr {
    std::array<T, VIEW_RAYS> x{};
    for (size_t index=0; index<VIEW_RAYS; ++index) {
      x[index] = std::cos(VIEW_RAYS_ANGLE_START + (static_cast<T>(index) * VIEW_RAYS_ANGLE_DELTA));
    }
    return x;
  }()};


template <jms::math::SimType T>
const std::array<T, VIEW_RAYS> Sim<T>::SIN_ANGLES{
  []() constexpr {
    std::array<T, VIEW_RAYS> x{};
    for (size_t index=0; index<VIEW_RAYS; ++index) {
      x[index] = std::sin(VIEW_RAYS_ANGLE_START + (static_cast<T>(index) * VIEW_RAYS_ANGLE_DELTA));
    }
    return x;
  }()};
*/


} // namespace angles
} // namespace math
} // namespace jms


#endif // JMS_MATH_ANGLES_H
