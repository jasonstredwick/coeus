#ifndef SIM_SIM_T_H
#define SIM_SIM_T_H


#include <cfloat>


namespace jms {
namespace sim {


#if 1
// 7 significant digits
using sim_t = float;
inline constexpr sim_t MAX_VALUE = FLT_MAX;
inline constexpr sim_t NEG_ONE = -1.0f;
inline constexpr sim_t ONE = 1.0f;
inline constexpr sim_t ZERO = 0.0f;
#else
// 16 significant digits
using sim_t = double;
inline constexpr sim_t MAX_VALUE = DBL_MAX;
inline constexpr sim_t NEG_ONE = -1.0;
inline constexpr sim_t ONE = 1.0;
inline constexpr sim_t ZERO = 0.0;
#endif


} // namespace sim
} // namespace jms


#endif // SIM_SIM_T_H
