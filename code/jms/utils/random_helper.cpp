#include "jms/utils/random_helper.h"

#include <optional>
#include <random>
#include <vector>


namespace jms {
namespace utils {
namespace random_helper {


void SetSeed_mt19937_64(std::mt19937_64& rng, optional_seed_input_t seed) {
  if (seed) {
    std::seed_seq rng_seed(seed.value().begin(), seed.value().end());
    rng.seed(rng_seed);
  } else {
    std::random_device random_device;
    std::seed_seq default_seed{random_device(),
                               random_device(),
                               random_device(),
                               random_device(),
                               random_device(),
                               random_device(),
                               random_device(),
                               random_device()};
    rng.seed(default_seed);
  }
  return;
}


} // namespace random_helper
} // namespace utils
} // namespace jms
