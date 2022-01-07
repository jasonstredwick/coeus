// Review: https://www.pcg-random.org/posts/cpp-seeding-surprises.html

#ifndef JMS_UTILS_RANDOM_HELPER_H
#define JMS_UTILS_RANDOM_HELPER_H


#include <algorithm>
#include <optional>
#include <random>
#include <ranges>
#include <vector>


namespace jms {
namespace utils {
namespace random_helper {


using seed_seq_input_t = std::vector<std::seed_seq::result_type>;
using optional_seed_seq_input_t = std::optional<seed_seq_input_t>;


constexpr uint32_t DEFAULT_NUM_DISCARD = 1024;


void Initialize(std::mt19937_64& rng,
                std::optional<int32_t> opt_seed=std::nullopt,
                optional_seed_seq_input_t opt_seed_seq_vec=std::nullopt,
                std::optional<uint32_t> opt_discard=std::nullopt) {
  uint32_t num_discard = opt_discard.has_value() ? opt_discard.value() : DEFAULT_NUM_DISCARD;
  seed_seq_input_t seed_seq_vec;
  if (opt_seed_seq_vec) {
    seed_seq_vec = std::move(opt_seed_seq_vec.value());
  } else if (opt_seed) {
    std::minstd_rand lcg(opt_seed.value());
    if (num_discard) { lcg.discard(num_discard); }
    for (size_t i : std::ranges::iota_view{static_cast<size_t>(0), std::mt19937_64::state_size}) { seed_seq_vec.emplace_back(lcg()); }
  } else {
    std::random_device random_device;
    if (num_discard) { for (uint32_t i : std::ranges::iota_view{static_cast<uint32_t>(0), num_discard}) { random_device(); } }
    for (size_t i : std::ranges::iota_view{static_cast<size_t>(0), std::mt19937_64::state_size}) { seed_seq_vec.emplace_back(random_device()); }
  }
  std::seed_seq rng_seed(seed_seq_vec.begin(), seed_seq_vec.end());
  rng.seed(rng_seed);
  if (num_discard) { rng.discard(num_discard); }
  return;
}


} // namespace random_helper
} // namespace utils
} // namespace jms


#endif // JMS_UTILS_RANDOM_HELPER_H
