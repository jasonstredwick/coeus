#include "fmt/core.h"

#include "jms/shape_engine/shape.h"


int main(void) {
  fmt::print("start\n");
  jms::shape_engine::RegularShape<float, 0> circle1;
  fmt::print("Size: {}\n", circle1.center.size());
  fmt::print("Radius: {}\n", circle1.radius);
  fmt::print("end\n");
  return 0;
}
