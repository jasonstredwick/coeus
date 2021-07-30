#ifndef JMS_MATH_SPARCE_MATRIX_H
#define JMS_MATH_SPARCE_MATRIX_H


#include <type_traits>


namespace jms {
namespace math {


template <typename T> concept Arithmetic_t = std::is_arithmetic_v<T>;


template <Arithmetic_t T, typename Array_t>
class SparseMatrix {
private:
  Array_t<size_t> row;
  Array_t<size_t> col;
  Array_t<T> values;

public:
};


} // namespace math
} // namespace jms


#endif // JMS_MATH_SPARCE_MATRIX_H
