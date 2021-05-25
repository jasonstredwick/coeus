#ifndef SIM_SIM_T_H
#define SIM_SIM_T_H


#include <cfloat>
#include <limits>


namespace jms {
namespace sim {


#if 1
// 7 significant digits
using sim_t = float;
constexpr sim_t MAX_VALUE = std::numeric_limits<sim_t>::max();
constexpr sim_t NEG_ONE = -1.0f;
constexpr sim_t ONE = 1.0f;
constexpr sim_t TWO = 2.0f;
constexpr sim_t ZERO = 0.0f;
#else
// 16 significant digits
using sim_t = double;
constexpr sim_t MAX_VALUE = std::numeric_limits<sim_t>::max();
constexpr sim_t NEG_ONE = -1.0;
constexpr sim_t ONE = 1.0;
constexpr sim_t TWO = 2.0;
constexpr sim_t ZERO = 0.0;
#endif


} // namespace sim
} // namespace jms


#endif // SIM_SIM_T_H
