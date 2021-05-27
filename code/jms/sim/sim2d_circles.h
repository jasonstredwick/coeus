#ifndef SIM_SIM2D_CIRCLES_H
#define SIM_SIM2D_CIRCLES_H


#include <memory>

#include "jms/sim/interface.h"
#include "jms/utils/random_helper.h"


namespace jms {
namespace sim {


std::unique_ptr<Interface> CreateSim2DCircles(jms::utils::random_helper::optional_seed_input_t seed=std::nullopt);
std::unique_ptr<Interface> CreateSim2DCircles_f(jms::utils::random_helper::optional_seed_input_t seed=std::nullopt);


} // namespace sim
} // namespace jms


#endif // SIM_SIM2D_CIRCLES_H
