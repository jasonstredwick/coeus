#ifndef JMS_SHAPE_ENGINE_CIRCLE_H
#define JMS_SHAPE_ENGINE_CIRCLE_H


#include <array>
#include <numeric>

#include "jms/math/vector.h"


namespace jms {
namespace shape_engine {
namespace regular {


template <typename T, size_t SIDES>
class Shape {
public:
  static constexpr size_t DIM = 2;

public:
  jms::math::vector::Vector<T, 2> center;
  jms::math::vector::Vector<T, 2> orientation = {{0, 1}}; // expected to be a unit vector; or approximate
  T radius {0};
};


template <typename T> using Circle = Shape<T, 0>;
template <typename T> using Point = Shape<T, 1>;
template <typename T> using Line = Shape<T, 2>;
template <typename T> using Triangle = Shape<T, 3>;
template <typename T> using Square = Shape<T, 4>;
template <typename T> using Pentagon = Shape<T, 5>;
template <typename T> using Hexagon = Shape<T, 6>;


} // namespace regular
} // namespace shape_engine
} // namespace jms


#endif // JMS_SHAPE_ENGINE_CIRCLE_H
