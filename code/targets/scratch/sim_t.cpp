#include "fmt/core.h"

#include "jms/sim/sim_t.h"


int main(void) {
  fmt::print("start\n");

  fmt::print("MIN_DELTA: {}\n", jms::sim::MIN_DELTA);
  fmt::print("LARGEST_AFTER_ONE: {}\n", jms::sim::LARGEST_AFTER_ONE);
  fmt::print("LARGEST_AFTER_ONE_DIST: {}\n", jms::sim::LARGEST_AFTER_ONE_DIST);

  fmt::print("end\n");
  return 0;
}
