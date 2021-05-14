#include <iostream>
#include <limits>
#include <random>
#include "fmt/core.h"

#include "jms/sim/sim_t.h"


int main(void) {
  fmt::print("start\n");

  std::cout.precision(8);
  fmt::print("MIN_DELTA: {}\n", jms::sim::MIN_DELTA);
  fmt::print("LARGEST_AFTER_ONE: {}\n", jms::sim::LARGEST_AFTER_ONE);
  fmt::print("LARGEST_AFTER_ONE_DIST: {}\n", jms::sim::LARGEST_AFTER_ONE_DIST);
  fmt::print("{}\n", 1.000'000'00f);
  fmt::print("{}\n", 1.000'000'01f);
  fmt::print("{}\n", 1.000'000'02f);
  fmt::print("{}\n", 1.000'000'03f);
  fmt::print("{}\n", 1.000'000'04f);
  fmt::print("{}\n", 1.000'000'05f);
  fmt::print("{}\n", 1.000'000'06f);
  fmt::print("{}\n", 1.000'000'07f);
  fmt::print("{}\n", 1.000'000'08f);
  fmt::print("{}\n", 1.000'000'09f);
  fmt::print("---\n");
  fmt::print("{}\n", 1.000'000'1f);
  fmt::print("{}\n", 1.000'000'2f);
  fmt::print("{}\n", 1.000'000'3f);
  fmt::print("{}\n", 1.000'000'4f);
  fmt::print("{}\n", 1.000'000'5f);
  fmt::print("{}\n", 1.000'000'6f);
  fmt::print("{}\n", 1.000'000'7f);
  fmt::print("{}\n", 1.000'000'8f);
  fmt::print("{}\n", 1.000'000'9f);
  fmt::print("\n");
  fmt::print("---\n");
  std::cout << 1.000'000'1f << std::endl;
  std::cout << 1.000'000'2f << std::endl;
  std::cout << 1.000'000'3f << std::endl;
  std::cout << 1.000'000'4f << std::endl;
  std::cout << 1.000'000'5f << std::endl;
  std::cout << 1.000'000'6f << std::endl;
  std::cout << 1.000'000'7f << std::endl;
  std::cout << 1.000'000'8f << std::endl;
  std::cout << 1.000'000'9f << std::endl;
  std::cout << 1.000'001'0f << std::endl;
  std::cout << 1.000'008'0f << std::endl;
  std::cout << 1.000'009'0f << std::endl;
  fmt::print("---\n");
  std::cout << std::endl;
  std::cout << std::fixed << 0.000'000'01f << std::endl;
  std::cout << std::fixed << 0.000'000'02f << std::endl;
  std::cout << std::fixed << 0.000'000'03f << std::endl;
  std::cout << std::fixed << 0.000'000'04f << std::endl;
  std::cout << std::fixed << 0.000'000'05f << std::endl;
  std::cout << std::fixed << 0.000'000'06f << std::endl;
  std::cout << std::fixed << 0.000'000'07f << std::endl;
  std::cout << std::fixed << 0.000'000'08f << std::endl;
  std::cout << std::fixed << 0.000'000'09f << std::endl;
  std::cout << std::fixed << 0.000'001'00f << std::endl;
  std::cout << std::fixed << 0.000'008'0f << std::endl;
  std::cout << std::fixed << 0.000'009'0f << std::endl;
  std::cout << std::endl;
  std::cout << FLT_MIN << std::endl;
  std::cout << 1.0f + FLT_MIN << std::endl;
  std::cout << std::nextafter(1.0f, std::numeric_limits<float>::max()) << std::endl;

  fmt::print("end\n");
  return 0;
}
