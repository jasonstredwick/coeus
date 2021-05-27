#ifndef JMS_UTILS_RANDOM_HELPER_H
#define JMS_UTILS_RANDOM_HELPER_H


#include <optional>
#include <random>
#include <vector>


namespace jms {
namespace utils {
namespace random_helper {


using seed_input_t = std::vector<std::seed_seq::result_type>;
using optional_seed_input_t = std::optional<seed_input_t>;


void SetSeed_mt19937_64(std::mt19937_64& rng, optional_seed_input_t seed=std::nullopt);


} // namespace random_helper
} // namespace utils
} // namespace jms


#endif // JMS_UTILS_RANDOM_HELPER_H
