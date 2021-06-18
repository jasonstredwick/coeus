#include "jms/sim/sim2d_circles.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <numbers>
#include <optional>
#include <random>
#include <ranges>

#include "jms/shape_engine/regular.h"
#include "jms/sim/interface.h"
#include "jms/sim/sim_t.h"
#include "jms/utils/random_helper.h"


namespace jms {
namespace sim {


class alignas(64) Actor {
public:
  jms::shape_engine::regular::Circle<sim_t> body;
  sim_t speed;
  sim_t energy;

public:
  void Move(void);
};


class Agent : public Actor {
public:
  constexpr void Initialize(auto&& GenUnit, sim_t radius_, sim_t energy_) noexcept {
    auto dir {{GenUnit(), GenUnit()}};
    sim_t dlen = jms::math::vector::Magnitude(dir);
    body.center = {0, 0};
        if (dlen == 0) {
      body.orientation = {{0, 1}};
    } else {
      body.orientation = dir / dlen;
    }
    body.radius = radius_;
    energy = energy_;
    speed = 0;
    return;
  }
};


class Food : public Actor {
public:
  constexpr void Initialize(auto&& GenRadius, auto&& GenSpawnRadius, auto&& GenUnit,
                            const auto& agent_center, sim_t energy_, sim_t speed_) noexcept {
    auto pos {{GenUnit(), GenUnit()}};
    auto dir {{GenUnit(), GenUnit()}};
    size_t plen = jms::math::vector::Magnitude(pos);
    sim_t dlen = jms::math::vector::Magnitude(dir);
    sim_t spawn_distance = GenSpawnRadius();
    if (plen == 0) {
      body.center = {agent_center[0], spawn_distance + agent_center[1]};
    } else {
      body.center = (pos / plen) * spawn_distance + agent_center;
    }
    if (dlen == 0) {
      body.orientation = {{0, 1}};
    } else {
      body.orientation = dir / dlen;
    }
    body.radius = GenRadius();
    energy = energy_;
    speed = speed_;
    return;
  }
};


/*
inline constexpr size_t BUFFER_DIM = 1024;
inline constexpr size_t BUFFER_HALF_DIM = BUFFER_DIM / 2;
inline constexpr sim_t SCALE_T = SPAWN_RADIUS_MAX / static_cast<int>(BUFFER_HALF_DIM);
inline constexpr int SCALE = static_cast<int>(SCALE_T) + static_cast<int>(std::ceil(SCALE_T) - std::floor(SCALE_T));
*/
class Sim2DCirclesOO : public Interface {
public:
  static constexpr sim_t SPACE_UNIT = 0.0001; // used below
  static constexpr sim_t ENERGY_CONSUMPTION_FOOD_IDLE = 0.5;
  static constexpr sim_t ENERGY_CONSUMPTION_FOOD_MOVE = 1.0;
  static constexpr sim_t ENERGY_CONSUMPTION_AGENT_IDLE = 1.0;
  static constexpr sim_t ENERGY_CONSUMPTION_AGENT_MOVE = 1.0;
  static constexpr sim_t ENERGY_CONSUMPTION_AGENT_ROTATION = 2.0 / std::numbers::pi;
  static constexpr sim_t ENERGY_CONSUMPTION_AGENT_SPRINT = 15.0;
  static constexpr sim_t ENERGY_GAIN_FOOD = 1000.0;
  static constexpr sim_t ENERGY_START_AGENT = 10000.0;
  static constexpr sim_t ENERGY_START_FOOD = 1000.0;
  static constexpr sim_t INTENSITY_MAX = 999.0 * SPACE_UNIT;
  static constexpr size_t NUM_FOOD_PER_AGENT = 8192;
  static constexpr sim_t RADIUS_AGENT = 2.0 * SPACE_UNIT;
  static constexpr sim_t RADIUS_AGENT_SQUARED = RADIUS_AGENT * RADIUS_AGENT;
  static constexpr sim_t RADIUS_FOOD_MIN = 0.5 * SPACE_UNIT;
  static constexpr sim_t RADIUS_FOOD_MAX = 6.0 * SPACE_UNIT;
  static constexpr sim_t RADIUS_FOOD_MIN_DELTA = std::nextafter(RADIUS_FOOD_MAX, std::numeric_limits<sim_t>::max());
  static constexpr sim_t SPAWN_RADIUS_MIN = 10.0 * SPACE_UNIT;
  static constexpr sim_t SPAWN_RADIUS_MAX = 999.0 * SPACE_UNIT;
  static constexpr sim_t SPAWN_RADIUS_MIN_DELTA = std::nextafter(SPAWN_RADIUS_MAX, std::numeric_limits<sim_t>::max());
  static constexpr sim_t SPAWN_RADIUS_DELTA = SPAWN_RADIUS_MAX - SPAWN_RADIUS_MIN;
  static constexpr sim_t SPEED_AGENT_MOVE = 5.0 * SPACE_UNIT;
  static constexpr sim_t SPEED_AGENT_SPRINT = 10.0 * SPACE_UNIT;
  static constexpr sim_t SPEED_FOOD_MOVE = 1.0 * SPACE_UNIT;
  static constexpr sim_t UNIT_MIN_DELTA = std::nextafter(ONE, std::numeric_limits<sim_t>::max());
  static constexpr size_t VIEW_RAYS = 1024;
  static constexpr sim_t VIEW_RAYS_ANGLE_RANGE = std::numbers::pi / 4.0; // 45deg
  static constexpr sim_t VIEW_RAYS_ANGLE_DELTA = VIEW_RAYS_ANGLE_RANGE / static_cast<sim_t>(VIEW_RAYS);
  // Start negative side up by 1 delta to account for an even number of rays as
  // we want to ensure the forward direction is represented.
  static constexpr sim_t VIEW_RAYS_ANGLE_START = -(VIEW_RAYS_ANGLE_RANGE / 2.0) + VIEW_RAYS_ANGLE_DELTA;
  static constexpr sim_t VIEW_DISTANCE = 500.0 * SPACE_UNIT;
  // Based on counterclockwise rotation sweeping from left to right.
  alignas(64) static const std::array<sim_t, VIEW_RAYS> COS_ANGLES;
  alignas(64) static const std::array<sim_t, VIEW_RAYS> SIN_ANGLES;

protected:
  std::array<Food, NUM_FOOD_PER_AGENT> food;
  Agent agent;
  alignas(64) std::array<sim_t, VIEW_RAYS> agent_view;
private:
  // partial results used during calculations.
  alignas(64) std::array<sim_t, NUM_FOOD_PER_AGENT> collision_check;
  alignas(64) std::array<sim_t, VIEW_RAYS> rays_dx;
  alignas(64) std::array<sim_t, VIEW_RAYS> rays_dy;
  alignas(64) std::array<sim_t, VIEW_RAYS> working_agent_view;
protected:
  std::mt19937_64 rng; // multithreading/distributed compute

public:
  Sim2DCirclesOO(void) noexcept {}
  Sim2DCirclesOO(const Sim2DCirclesOO&) = delete;
  Sim2DCirclesOO(const Sim2DCirclesOO&&) = delete;
  Sim2DCirclesOO& operator=(const Sim2DCirclesOO&) = delete;
  Sim2DCirclesOO& operator=(const Sim2DCirclesOO&&) = delete;
  virtual ~Sim2DCirclesOO(void) {}

  static std::unique_ptr<Interface> Create(
      jms::utils::random_helper::optional_seed_input_t seed=std::nullopt);
  sim_t LuminescenceValue(sim_t v);
  virtual void Step(void) override;

protected:
  void CreateFood(size_t index);
  sim_t GenRadius(void);
  sim_t GenSpawnRadius(void);
  sim_t GenUnit(void);
};


const std::array<sim_t, Sim2DCirclesOO::VIEW_RAYS> Sim2DCirclesOO::COS_ANGLES{
  []() constexpr {
    std::array<sim_t, VIEW_RAYS> x{};
    for (size_t i : std::ranges::iota_view{0, VIEW_RAYS}) {
      x[i] = std::cos(VIEW_RAYS_ANGLE_START + (i * VIEW_RAYS_ANGLE_DELTA));
    }
    return x;
  }()};


const std::array<sim_t, Sim2DCirclesOO::VIEW_RAYS> Sim2DCirclesOO::SIN_ANGLES{
  []() constexpr {
    std::array<sim_t, VIEW_RAYS> x{};
    for (size_t i : std::ranges::iota_view{0, VIEW_RAYS}) {
      x[i] = std::sin(VIEW_RAYS_ANGLE_START + (i * VIEW_RAYS_ANGLE_DELTA));
    }
    return x;
  }()};


std::unique_ptr<Interface> Sim2DCirclesOO::Create(
    jms::utils::random_helper::optional_seed_input_t seed) {
  std::unique_ptr<Sim2DCirclesOO> sim{new Sim2DCirclesOO{}};
  jms::utils::random_helper::SetSeed_mt19937_64(sim->rng, seed);
  auto GenRadiusFn = std::bind(&Sim2DCirclesOO::GenRadius, this);
  auto GenSpawnRadiusFn = std::bind(&Sim2DCirclesOO::GenSpawnRadius, this);
  auto GenUnitFn = std::bind(&Sim2DCirclesOO::GenUnit, this);
  agent.Initialize(GenUnitFn, RADIUS_AGENT, ENERGY_START_AGENT);
  for (auto& food_unit : food) {
    food_unit.Initialize(GenRadiusFn, GenSpawnRadiusFn, GenUnitFn,
                  agent.body.center, ENERGY_START_FOOD, SPEED_FOOD_MOVE);
  }
  return sim;
}


void Sim2DCirclesOO::CreateFood(size_t index) {
  jms::math::vector::Vector pos{{GenUnit(), GenUnit()}};
  size_t plen = jms::math::vector::Magnitude(pos);
  sim_t spawn_distance = GenSpawnRadius();
  // place relative to the agent
  if (plen == 0) {
    food_pos_x[index] = agent_info[AGENT_POS_X];
    food_pos_y[index] = spawn_distance + agent_info[AGENT_POS_Y];
  } else {
    food_pos_x[index] = (px / plen) * spawn_distance + agent_info[AGENT_POS_X];
    food_pos_y[index] = (py / plen) * spawn_distance + agent_info[AGENT_POS_Y];
  }
  sim_t dx = GenUnit();
  sim_t dy = GenUnit();
  sim_t dlen = std::sqrt(dx * dx + dy * dy);
  if (dlen == 0) {
    food_dir_x[index] = 0;
    food_dir_y[index] = 1;
  } else {
    food_dir_x[index] = dx / dlen;
    food_dir_y[index] = dy / dlen;
  }
  food_radius[index] = GenRadius();
  food_energy[index] = ENERGY_START_FOOD;
  return;
}


sim_t Sim2DCirclesOO::GenUnit(void) {
  std::uniform_real_distribution<sim_t> dist(NEG_ONE, ONE + UNIT_MIN_DELTA);
  return dist(rng);
};


sim_t Sim2DCirclesOO::GenRadius() {
  std::uniform_real_distribution<sim_t> dist(RADIUS_FOOD_MIN,
                                             RADIUS_FOOD_MAX + RADIUS_FOOD_MIN_DELTA);
  return dist(rng);
}


sim_t Sim2DCirclesOO::GenSpawnRadius() {
  std::uniform_real_distribution<sim_t> dist(SPAWN_RADIUS_MIN,
                                             SPAWN_RADIUS_MAX + SPAWN_RADIUS_MIN_DELTA);
  return dist(rng);
}


sim_t Sim2DCirclesOO::LuminescenceValue(sim_t v) {
  return (v > INTENSITY_MAX) ? 0 : (ONE - (v / INTENSITY_MAX));
}


void Sim2DCirclesOO::Step(void) {
  /***
   * Process-
   * for each food
   *   1. Check for possible intersections
   *   2. Check for agent/food collision and food out of energy (create new)
   *   3. Update agent_view
   *   4. Move food
   *   5. Create agent view
   * Update agent
   */
  // Load agent info.
  sim_t px_a = agent_info[AGENT_POS_X];
  sim_t py_a = agent_info[AGENT_POS_Y];
  sim_t dx_a = agent_info[AGENT_DIR_X];
  sim_t dy_a = agent_info[AGENT_DIR_Y];
  sim_t r_a = agent_info[AGENT_RADIUS];
  sim_t energy_a = agent_info[AGENT_ENERGY];
  sim_t speed_a = agent_info[AGENT_SPEED];

  // High level (initial) check for food/agent collision
  sim_t H = speed_a / 2;
  sim_t I = SPEED_FOOD_MOVE / 2;
  for (size_t index=0; index<NUM_FOOD_PER_AGENT; ++index) {
    // Load food information
    sim_t px_f = food_pos_x[index];
    sim_t py_f = food_pos_y[index];
    sim_t dx_f = food_dir_x[index];
    sim_t dy_f = food_dir_y[index];
    sim_t r_f = food_radius[index];

    sim_t A = px_f - px_a;
    sim_t C = py_f - py_a;
    sim_t E = r_f + r_a;
    sim_t J = I * dx_f - H * dx_a;
    sim_t K = I * dy_f - H * dy_a;

    // Test for possible intersection
    sim_t fx = A + J;
    sim_t fy = C + K;
    sim_t f_2 = fx * fx + fy * fy;
    sim_t f_distance = E + H + I;
    collision_check[index] = f_distance - f_2;
  }

  // For possible collisions, determine if an actual collision occurred
  for (size_t index=0; index<NUM_FOOD_PER_AGENT; ++index) {
    sim_t energy_f = food_energy[index];

    if (collision_check[index] >= 0) {
      // Load food information
      sim_t px_f = food_pos_x[index];
      sim_t py_f = food_pos_y[index];
      sim_t dx_f = food_dir_x[index];
      sim_t dy_f = food_dir_y[index];
      sim_t r_f = food_radius[index];

      // Reused calculations
      sim_t A = px_f - px_a;
      sim_t B = dx_f - dx_a;
      sim_t C = py_f - py_a;
      sim_t D = dy_f - dy_a;
      sim_t E = r_f + r_a;
      sim_t F = E * E;

      // Check for intersection in motion
      sim_t a = B * B + D * D;
      sim_t b_partial = A * B + C * D;
      sim_t c = A * A + C * C - F;
      sim_t discriminant = fma(b_partial, b_partial, -a * c);
      if (discriminant > 0) {
        sim_t q = -(b_partial +
                    std::copysign(std::sqrt(discriminant), b_partial));
        sim_t t1 = q / a;
        sim_t t2 = c / q;
        if (t1 >= 0 or t2 >= 0) {
          // collision
          energy_a += ENERGY_GAIN_FOOD;
          CreateFood(index);
          continue;
        }
      }
    }
    if (energy_f - ENERGY_CONSUMPTION_FOOD_MOVE <= 0) {
      CreateFood(index);
    }
  }

  // Update agent
  if (speed_a > 0) {
    px_a = fma(speed_a, dx_a, px_a);
    py_a = fma(speed_a, dy_a, py_a);
    agent_info[AGENT_POS_X] = px_a;
    agent_info[AGENT_POS_Y] = py_a;
  }
  agent_info[AGENT_ENERGY] = energy_a;

  // Move food
  for (size_t index=0; index<NUM_FOOD_PER_AGENT; ++index) {
    // Load food information
    sim_t px_f = food_pos_x[index];
    sim_t py_f = food_pos_y[index];
    sim_t dx_f = food_dir_x[index];
    sim_t dy_f = food_dir_y[index];
    sim_t r_f = food_radius[index];
    sim_t energy_f = food_energy[index];

    // Update food position
    food_pos_x[index] = fma(SPEED_FOOD_MOVE, dx_f, px_f);
    food_pos_y[index] = fma(SPEED_FOOD_MOVE, dy_f, py_f);
    food_energy[index] = energy_f - ENERGY_CONSUMPTION_FOOD_MOVE;
  }

  // Generate agent view
  for (size_t index=0; index<VIEW_RAYS; ++index) {
    sim_t cos_t = COS_ANGLES[index];
    sim_t sin_t = SIN_ANGLES[index];
    rays_dx[index] = dx_a * cos_t - dy_a * sin_t;
    rays_dy[index] = dx_a * sin_t + dy_a * cos_t;
  }
  std::fill_n(working_agent_view.begin(), VIEW_RAYS, MAX_VALUE);

  // Slowest unit of this function by a few orders of magnitude.
  for (size_t food_index=0; food_index<NUM_FOOD_PER_AGENT; ++food_index) {
    // Load food information
    sim_t px_f = food_pos_x[food_index];
    sim_t py_f = food_pos_y[food_index];
    sim_t dx_f = food_dir_x[food_index];
    sim_t dy_f = food_dir_y[food_index];
    sim_t r_f = food_radius[food_index];

    sim_t A = px_a - px_f;
    sim_t C = py_a - py_f;
    sim_t F = r_f * r_f;
    sim_t G = A * A + C * C - F;
    sim_t H = -G;

    // test if food is touching ends or in the middle of view range.
    // left
    sim_t rays_dx_l = rays_dx[0];
    sim_t rays_dy_l = rays_dy[0];
    sim_t b_partial_l = A * rays_dx_l + C * rays_dy_l;
    sim_t discriminant_l = fma(b_partial_l, b_partial_l, H);
    // right
    sim_t rays_dx_r = rays_dx[VIEW_RAYS - 1];
    sim_t rays_dy_r = rays_dy[VIEW_RAYS - 1];
    sim_t b_partial_r = A * rays_dx_r + C * rays_dy_r;
    sim_t discriminant_r = fma(b_partial_r, b_partial_r, H);
    if (discriminant_l < 0 && discriminant_r < 0) { // no intersection
      sim_t af_x = px_f - px_a;
      sim_t af_y = py_f - py_a;
      // ax * by - ay * bx : a == ray; b == af (agent to food vector; f - a)
      sim_t dir_l = rays_dx_l * af_y - rays_dy_l * af_x;
      sim_t dir_r = rays_dx_r * af_y - rays_dy_r * af_x;
      // in between = dir_l <= 0 (to the right) && dir_r >= 0 (to the left)
      // check for not between
      if (dir_l > 0 || dir_r < 0) {
        continue;
      }
    }

    for (size_t ray_index=0; ray_index<VIEW_RAYS; ++ray_index) {
      // sim_t a = ONE;
      sim_t b_partial = A * rays_dx[ray_index] + C * rays_dy[ray_index];
      sim_t discriminant = fma(b_partial, b_partial, H);
      if (discriminant >= 0) {
        sim_t ray_distance = working_agent_view[ray_index];
        sim_t x = ray_distance;
        // x1 = q / a -> q / 1 -> q
        sim_t x1 = -(-b_partial +
                    std::copysign(std::sqrt(discriminant), -b_partial));
        if (x1 > RADIUS_AGENT && x1 < x) {
          x = x1;
        }
        if (x1 != 0) {
          sim_t x2 = G / x1;
          if (x2 > RADIUS_AGENT && x2 < x) {
            x = x2;
          }
        }
        if (x < ray_distance) {
          working_agent_view[ray_index] = x;
        }
      }
    }
  }

  for (size_t index=0; index<VIEW_RAYS; ++index) {
    agent_view[index] = LuminescenceValue(working_agent_view[index]);
  }

  return;
}


std::unique_ptr<Interface> CreateSim2DCirclesOO(
    jms::utils::random_helper::optional_seed_input_t seed) {
  return Sim2DCirclesOO::Create(seed);
}


} // namespace sim
} // namespace jms