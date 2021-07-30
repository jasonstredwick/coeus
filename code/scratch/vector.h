#ifndef JMS_MATH_VEC2D_H
#define JMS_MATH_VEC2D_H


#include <array>
#include <cmath>


namespace jms {
namespace math {
namespace vector {


template <typename T, size_t DIM>
class Vector {
public:
  static constexpr size_t dim = DIM;
  using value_type = T;

public:
  std::array<T, DIM> data {0};
};


// Negate
template <typename T, size_t DIM>
constexpr auto operator-(const Vector<T, DIM>& a) noexcept {
  std::array<T, DIM> out;
  for (size_t i=0; i<DIM; ++i) { out[i] = -a[i]; }
  return out;
}


// Addition
template <typename T, size_t DIM>
constexpr auto operator+(const Vector<T, DIM>& a, T b) noexcept {
  std::array<T, DIM> out;
  for (size_t i=0; i<DIM; ++i) { out[i] = a[i] + b; }
  return out;
}


template <typename T, size_t DIM>
constexpr auto operator+(const Vector<T, DIM>& a, const Vector<T, DIM>& b) noexcept {
  std::array<T, DIM> out;
  for (size_t i=0; i<DIM; ++i) { out[i] = a[i] + b[i]; }
  return out;
}


// Subtraction
template <typename T, size_t DIM>
constexpr auto operator-(const Vector<T, DIM>& a, T b) noexcept {
  std::array<T, DIM> out;
  for (size_t i=0; i<DIM; ++i) { out[i] = a[i] - b; }
  return out;
}


template <typename T, size_t DIM>
constexpr auto operator-(const Vector<T, DIM>& a, const Vector<T, DIM>& b) noexcept {
  std::array<T, DIM> out;
  for (size_t i=0; i<DIM; ++i) { out[i] = a[i] - b[i]; }
  return out;
}


// Multiplication
template <typename T, size_t DIM>
constexpr auto operator*(const Vector<T, DIM>& a, T b) noexcept {
  std::array<T, DIM> out;
  for (size_t i=0; i<DIM; ++i) { out[i] = a[i] * b; }
  return out;
}


// Division
template <typename T, size_t DIM>
constexpr auto operator/(const Vector<T, DIM>& a, T b) noexcept {
  std::array<T, DIM> out;
  for (size_t i=0; i<DIM; ++i) { out[i] = a[i] / b; }
  return out;
}


// Operations
template <typename T, size_t DIM>
constexpr auto aXplusY(T a, const Vector<T, DIM>& x, const Vector<T, DIM>& y) noexcept {
  std::array<T, DIM> out;
  for (size_t i=0; i<DIM; ++i) { out[i] = fma(a, x[i], y[i]); }
  return out;
}


template <typename T, size_t DIM>
constexpr auto aXplusbY(T a, const Vector<T, DIM>& x, T b, const Vector<T, DIM>& y) noexcept {
  std::array<T, DIM> out;
  for (size_t i=0; i<DIM; ++i) { out[i] = fma(a, x[i], b * y[i]); }
  return out;
}


template <typename T>
constexpr auto Cross(const Vector<T, 2>& a, const Vector<T, 2>& b) noexcept {
  //std::array<T, 3> out;
  // a[1] * b[2] - a[2] * b[1]
  // a[1] * 0    - 0    * b[1]
  //out[0] = 0;
  // a[2] * b[0] - a[0] * b[2]
  // 0    * b[0] - a[0] * 0;
  //out[1] = 0;
  //out[2] = a[0] * b[1] - a[1] * b[0];
  return std::fma(a[0], b[1], a[1] * b[0];
}


template <typename T>
constexpr auto Cross(const Vector<T, 3>& a, const Vector<T, 3>& b) noexcept {
  std::array<T, 3> out;
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
  return out;
}


template <typename T, size_t DIM>
constexpr auto Dot(const Vector<T, DIM>& a, const Vector<T, DIM>& b) noexcept {
  T out = 0;
  for (size_t i=0; i<DIM; ++i) { out += a[i] * b[i]; }
  return out;
}


template <typename T, size_t DIM>
constexpr auto Hadamard(const Vector<T, DIM>& a, const Vector<T, DIM>& b) noexcept {
  std::array<T, DIM> out;
  for (size_t i=0; i<DIM; ++i) { out[i] = a[i] * b[i]; }
  return out;
}


template <typename T, size_t DIM>
constexpr auto Magnitude(const Vector<T, DIM>& a) noexcept {
  T out = 0;
  for (size_t i=0; i<DIM; ++i) { out += a[i] * a[i]; }
  return std::sqrt(out);
}


template <typename T, size_t DIM>
constexpr auto MagnitudeSquared(const Vector<T, DIM>& a) noexcept {
  T out = 0;
  for (size_t i=0; i<DIM; ++i) { out += a[i] * a[i]; }
  return std::sqrt(out);
}


template <typename T, size_t DIM>
constexpr auto Normalize(const Vector<T, DIM>& a) noexcept {
  // This function suffers from many issues and may not be performant or accurate.
  // For T=float, the precision difference between components and magnitude can cause accuracy loss.
  std::array<T, DIM> out;
  T mag = Magnitude(a);
  for (size_t i=0; i<DIM; ++i) { out[i] = a[i] / mag; }
  return out;
}


template <typename T>
constexpr auto PerpendicularCCW(const Vector<T, 2>& a) {
  std::array<T, 2> out;
  out[0] = -a[1];
  out[1] = a[0];
  return out;
}


template <typename T>
constexpr auto PerpendicularCW(const Vector<T, 2>& a) {
  std::array<T, 2> out;
  out[0] = a[1];
  out[1] = -a[0];
  return out;
}


template <typename T>
constexpr auto RotateCCW(const Vector<T, 2>& a, T theta) noexcept {
  std::array<T, 2> out;
  T x = a[0];
  T y = a[1];
  T cos_theta = std::cos(theta);
  T sin_theta = std::sin(theta);
  out[0] = std::fma(cos_theta, x, -sin_theta * y);
  out[1] = std::fma(sin_theta, x, cos_theta * y);
  return out;
}


} // namespace vector
} // namespace math
} // namespace jms


#endif // JMS_MATH_VEC2D_H
