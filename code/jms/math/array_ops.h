#ifndef JMS_MATH_ARRAY_OPS_H
#define JMS_MATH_ARRAY_OPS_H


#include <cmath>
#include <numeric>


namespace jms {
namespace math {


/***
  * NOTE: All the following functions assume that all dimensions are validated prior to usage.
  * NOTE: Best performance assumes vectors have sizes that are 64 byte aligned.
  * TODO: Make this work for static structures like c-arrays and std::array (probably need some kind of wrapper or multiple classes)
  */
template <typename T, template <typename> typename Array_t>
class ArrayOps {
  Array_t<T> data;

public:
  ArrayOps(size_t N) noexcept : data(N) { return; } // warnings/error for std::array and c-arrays; must be initialized with () rather than {} for things like std::vector
  ArrayOps(const Array_t<T>& data) noexcept : data{data} { return; }
  ArrayOps(Array_t<T>&& data) noexcept : data{data} { return; }
  ArrayOps(void) noexcept = default;
  ArrayOps(const ArrayOps&) noexcept = default;
  ArrayOps(ArrayOps&&) noexcept = default;
  ~ArrayOps(void) noexcept = default;
  ArrayOps& operator=(Array_t<T>&& data_) noexcept { data = data_; return *this; };
  ArrayOps& operator=(const ArrayOps&) noexcept = default;
  ArrayOps& operator=(ArrayOps&&) noexcept = default;

  Array_t<T>&& Extract(void) noexcept { return std::move(data); }
  Array_t<T>& Data(void) noexcept { return data; }

  ArrayOps& operator-(void) noexcept { for (size_t i=0; i<data.size(); ++i) { data[i] = -data[i]; } return *this; }
  ArrayOps& operator+(const T rhs) noexcept { for (size_t i=0; i<data.size(); ++i) { data[i] += rhs; } return *this; }
  ArrayOps& operator-(const T rhs) noexcept { for (size_t i=0; i<data.size(); ++i) { data[i] -= rhs; } return *this; }
  ArrayOps& operator*(const T rhs) noexcept { for (size_t i=0; i<data.size(); ++i) { data[i] *= rhs; } return *this; }
  ArrayOps& operator/(const T rhs) noexcept { for (size_t i=0; i<data.size(); ++i) { data[i] /= rhs; } return *this; }
  ArrayOps& operator=(const T rhs) noexcept { for (size_t i=0; i<data.size(); ++i) { data[i] = rhs; } return *this; }
  ArrayOps& operator+(const Array_t<T>& rhs) noexcept { for (size_t i=0; i<data.size(); ++i) { data[i] += rhs[i]; } return *this; }
  ArrayOps& operator-(const Array_t<T>& rhs) noexcept { for (size_t i=0; i<data.size(); ++i) { data[i] -= rhs[i]; } return *this; }
  ArrayOps& operator*(const Array_t<T>& rhs) noexcept { for (size_t i=0; i<data.size(); ++i) { data[i] *= rhs[i]; } return *this; }
  ArrayOps& operator/(const Array_t<T>& rhs) noexcept { for (size_t i=0; i<data.size(); ++i) { data[i] /= rhs[i]; } return *this; }
  ArrayOps& operator=(const Array_t<T>& rhs) noexcept { for (size_t i=0; i<data.size(); ++i) { data[i] = rhs[i]; } return *this; }
  ArrayOps& Fma(const T slope, const T intercept) noexcept {
    for (size_t i=0; i<data.size(); ++i) { data[i] = std::fma(data[i], slope, intercept); }
    return *this;
  }
  ArrayOps& Fma(const T a, const T b, const T c) noexcept {
    const T val = std::fma(a, b, c);
    for (size_t i=0; i<data.size(); ++i) { data[i] = val; }
    return *this;
  }
  ArrayOps& Fma(const Array_t<T>& slope, const Array_t<T>& intercept) noexcept {
    for (size_t i=0; i<data.size(); ++i) { data[i] = std::fma(data[i], slope[i], intercept[i]); }
    return *this;
  }
  ArrayOps& Fma(const Array_t<T>& a, const Array_t<T>& b, const Array_t<T>& c) noexcept {
    for (size_t i=0; i<data.size(); ++i) { data[i] = std::fma(a[i], b[i], c[i]); }
    return *this;
  }
  ArrayOps& MatMul(const Array_t<T>& lhs, const Array_t<Array_t<T>>& rhs) noexcept {
    // 1xM * MxN -> 1xN ; ArrayOps expected to be 1xN
    for (size_t i=0; i<data.size(); ++i) { data[i] = std::inner_product(lhs.cbegin(), lhs.cend(), rhs[i].cbegin(), static_cast<T>(0)); }
    return *this;
  }
};


template <typename T, template <typename> typename Array_t>
inline ArrayOps<T, Array_t> Clone(const Array_t<T>& a) noexcept {
  return ArrayOps<T, Array_t>(a);
}


template <typename T, template <typename> typename Array_t>
ArrayOps<T, Array_t> Fma(const Array_t<T>& a, const Array_t<T>& b, const Array_t<T>& c) noexcept {
  Array_t<T> data{a.size()};
  for (size_t i=0; i<data.size(); ++i) { data[i] = std::fma(a[i], b[i], c[i]); }
  return ArrayOps<T, Array_t>{std::move(data)};
}


template <typename T, template <typename> typename Array_t>
ArrayOps<T, Array_t> MatMul(const Array_t<T>& lhs, const Array_t<Array_t<T>>& rhs) noexcept {
  // 1xM * MxN -> 1xN   (MxN is stored similar to array<array<T, M>, N> ; i.e. (N, M)
  Array_t<T> data{rhs.size()};
  for (size_t i=0; i<data.size(); ++i) { data[i] = std::inner_product(lhs.cbegin(), lhs.cend(), rhs[i].cbegin(), static_cast<T>(0)); }
  return ArrayOps<T, Array_t>{std::move(data)};
}


} // namespace math
} // namespace jms


#endif // JMS_MATH_ARRAY_OPS_H
