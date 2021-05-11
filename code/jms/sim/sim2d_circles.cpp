#include "sim/sim2d_circles.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <numbers>
#include <optional>
#include <random>

#include "sim/interface.h"
#include "sim/sim_t.h"
#include "utils/random_helper.h"


namespace jms {
namespace sim {


/*
inline constexpr size_t BUFFER_DIM = 1024;
inline constexpr size_t BUFFER_HALF_DIM = BUFFER_DIM / 2;
inline constexpr sim_t SCALE_T = SPAWN_RADIUS_MAX / static_cast<int>(BUFFER_HALF_DIM);
inline constexpr int SCALE = static_cast<int>(SCALE_T) + static_cast<int>(std::ceil(SCALE_T) - std::floor(SCALE_T));
*/
class Sim2DCircles : public Interface {
public:
  static constexpr sim_t SPACE_UNIT = 0.0001; // used below
  static constexpr size_t AGENT_POS_X = 0;
  static constexpr size_t AGENT_POS_Y = 1;
  static constexpr size_t AGENT_DIR_X = 2;
  static constexpr size_t AGENT_DIR_Y = 3;
  static constexpr size_t AGENT_RADIUS = 4;
  static constexpr size_t AGENT_ENERGY = 5;
  static constexpr size_t AGENT_SPEED = 6;
  static constexpr sim_t ENERGY_CONSUMPTION_FOOD_IDLE = 0.5;
  static constexpr sim_t ENERGY_CONSUMPTION_FOOD_MOVE = 1.0;
  static constexpr sim_t ENERGY_CONSUMPTION_AGENT_IDLE = 1.0;
  static constexpr sim_t ENERGY_CONSUMPTION_AGENT_MOVE = 1.0;
  static constexpr sim_t ENERGY_CONSUMPTION_AGENT_ROTATION =
      2.0 / std::numbers::pi;
  static constexpr sim_t ENERGY_CONSUMPTION_AGENT_SPRINT = 15.0;
  static constexpr sim_t ENERGY_GAIN_FOOD = 1000.0;
  static constexpr sim_t ENERGY_START_AGENT = 10000.0;
  static constexpr sim_t ENERGY_START_FOOD = 1000.0;
  static constexpr sim_t INTENSITY_MAX = 2500.0 * SPACE_UNIT;
  static constexpr size_t NUM_FOOD_PER_AGENT = 8192;
  static constexpr sim_t RADIUS_AGENT = 2.0 * SPACE_UNIT;
  static constexpr sim_t RADIUS_AGENT_SQUARED = RADIUS_AGENT * RADIUS_AGENT;
  static constexpr sim_t RADIUS_FOOD_MIN = 0.5 * SPACE_UNIT;
  static constexpr sim_t RADIUS_FOOD_MAX = 6.0 * SPACE_UNIT;
  static constexpr sim_t SPAWN_RADIUS_MIN = 10.0 * SPACE_UNIT;
  static constexpr sim_t SPAWN_RADIUS_MAX = 2500.0 * SPACE_UNIT;
  static constexpr sim_t SPAWN_RADIUS_DELTA =
      SPAWN_RADIUS_MAX - SPAWN_RADIUS_MIN;
  static constexpr sim_t SPEED_AGENT_MOVE = 5.0 * SPACE_UNIT;
  static constexpr sim_t SPEED_AGENT_SPRINT = 10.0 * SPACE_UNIT;
  static constexpr sim_t SPEED_FOOD_MOVE = 1.0 * SPACE_UNIT;
  static constexpr size_t VIEW_RAYS = 1024;
  static constexpr sim_t VIEW_RAYS_ANGLE_RANGE =
      std::numbers::pi / 4.0; // 45deg
  static constexpr sim_t VIEW_RAYS_ANGLE_DELTA =
      VIEW_RAYS_ANGLE_RANGE / static_cast<sim_t>(VIEW_RAYS);
  // Start negative side up by 1 delta to account for an even number of rays as
  // we want to ensure the forward direction is represented.
  static constexpr sim_t VIEW_RAYS_ANGLE_START =
      -(VIEW_RAYS_ANGLE_RANGE / 2.0) + VIEW_RAYS_ANGLE_DELTA;
  static constexpr sim_t VIEW_DISTANCE = 1000.0 * SPACE_UNIT;
  // Based on counterclockwise rotation sweeping from left to right.
  alignas(64) static const std::array<sim_t, VIEW_RAYS> COS_ANGLES;
  alignas(64) static const std::array<sim_t, VIEW_RAYS> SIN_ANGLES;

protected:
  alignas(64) std::array<sim_t, NUM_FOOD_PER_AGENT> food_pos_x;
  alignas(64) std::array<sim_t, NUM_FOOD_PER_AGENT> food_pos_y;
  alignas(64) std::array<sim_t, NUM_FOOD_PER_AGENT> food_dir_x;
  alignas(64) std::array<sim_t, NUM_FOOD_PER_AGENT> food_dir_y;
  alignas(64) std::array<sim_t, NUM_FOOD_PER_AGENT> food_radius;
  alignas(64) std::array<sim_t, NUM_FOOD_PER_AGENT> food_energy;
  // extra for growth and alignment
  alignas(64) std::array<sim_t, 16> agent_info;
  alignas(64) std::array<sim_t, VIEW_RAYS> agent_view;
private:
  // partial results used during calculations.
  alignas(64) std::array<sim_t, VIEW_RAYS> rays_dx;
  alignas(64) std::array<sim_t, VIEW_RAYS> rays_dy;
  alignas(64) std::array<sim_t, NUM_FOOD_PER_AGENT> collision_check;
  alignas(64) std::array<sim_t, VIEW_RAYS> working_agent_view;
protected:
  std::mt19937_64 rng; // multithreading/distributed compute

public:
  Sim2DCircles(void) noexcept {}
  Sim2DCircles(const Sim2DCircles&) = delete;
  Sim2DCircles(const Sim2DCircles&&) = delete;
  Sim2DCircles& operator=(const Sim2DCircles&) = delete;
  Sim2DCircles& operator=(const Sim2DCircles&&) = delete;
  virtual ~Sim2DCircles(void) {}

  static std::unique_ptr<Interface> Create(
      jms::utils::random_helper::optional_seed_input_t seed=std::nullopt);
  sim_t LuminescenceValue(sim_t v);
  virtual void Step(void) override;

protected:
  void CreateFood(size_t index);
  sim_t GenUnit(void);
  sim_t GenRadius(void);
  sim_t GenSpawnRadius(void);
};


const std::array<sim_t, Sim2DCircles::VIEW_RAYS> Sim2DCircles::COS_ANGLES{
  []() constexpr {
    std::array<sim_t, VIEW_RAYS> x{};
    for (size_t i = 0; i<VIEW_RAYS; ++i) {
      x[i] = std::cos(VIEW_RAYS_ANGLE_START + (i * VIEW_RAYS_ANGLE_DELTA));
    }
    return x;
  }()};


const std::array<sim_t, Sim2DCircles::VIEW_RAYS> Sim2DCircles::SIN_ANGLES{
  []() constexpr {
    std::array<sim_t, VIEW_RAYS> x{};
    for (size_t i = 0; i<VIEW_RAYS; ++i) {
      x[i] = std::sin(VIEW_RAYS_ANGLE_START + (i * VIEW_RAYS_ANGLE_DELTA));
    }
    return x;
  }()};


std::unique_ptr<Interface> Sim2DCircles::Create(
    jms::utils::random_helper::optional_seed_input_t seed) {
  std::unique_ptr<Sim2DCircles> sim{new Sim2DCircles{}};
  jms::utils::random_helper::SetSeed_mt19937_64(sim->rng, seed);
  std::generate_n(sim->food_pos_x.begin(), NUM_FOOD_PER_AGENT,
                  [&sim]() { return sim->GenUnit(); });
  std::generate_n(sim->food_pos_y.begin(), NUM_FOOD_PER_AGENT,
                  [&sim]() { return sim->GenUnit(); });
  std::generate_n(sim->food_dir_x.begin(), NUM_FOOD_PER_AGENT,
                  [&sim]() { return sim->GenUnit(); });
  std::generate_n(sim->food_dir_y.begin(), NUM_FOOD_PER_AGENT,
                  [&sim]() { return sim->GenUnit(); });
  std::generate_n(sim->food_radius.begin(), NUM_FOOD_PER_AGENT,
                  [&sim]() { return sim->GenRadius(); });
  std::fill_n(sim->food_energy.begin(), NUM_FOOD_PER_AGENT, ENERGY_START_FOOD);
  // start as "center of the world"
  sim->agent_info[AGENT_POS_X] = ZERO;
  sim->agent_info[AGENT_POS_Y] = ZERO;
  sim->agent_info[AGENT_DIR_X] = sim->GenUnit();
  sim->agent_info[AGENT_DIR_Y] = sim->GenUnit();
  sim->agent_info[AGENT_RADIUS] = RADIUS_AGENT;
  sim->agent_info[AGENT_ENERGY] = ENERGY_START_AGENT;
  sim->agent_info[AGENT_SPEED] = ZERO;
  // Normalize agent's direction
  sim_t agent_len = std::sqrt(
      sim->agent_info[AGENT_DIR_X] * sim->agent_info[AGENT_DIR_X] +
      sim->agent_info[AGENT_DIR_Y] * sim->agent_info[AGENT_DIR_Y]);
  if (agent_len == 0) { sim->agent_info[AGENT_DIR_Y] = 1.0; }
  else {
    sim->agent_info[AGENT_DIR_X] /= agent_len;
    sim->agent_info[AGENT_DIR_Y] /= agent_len;
  }
  // Normalize food direction; normalize food position * spawn distance
  for (size_t i=0; i<NUM_FOOD_PER_AGENT; ++i) {
    sim_t dx = sim->food_dir_x[i];
    sim_t dy = sim->food_dir_y[i];
    sim_t dlen = std::sqrt(dx*dx + dy*dy);
    if (dlen == 0) { sim->food_dir_y[i] = 1.0; }
    else {
      sim->food_dir_x[i] = dx / dlen;
      sim->food_dir_y[i] = dy / dlen;
    }
    sim_t px = sim->food_pos_x[i];
    sim_t py = sim->food_pos_y[i];
    sim_t plen = std::sqrt(px*px + py*py);
    sim_t spawn_distance = sim->GenSpawnRadius();
    if (plen == 0) { sim->food_pos_y[i] = spawn_distance; }
    else {
      sim->food_pos_x[i] = (px / plen) * spawn_distance;
      sim->food_pos_y[i] = (py / plen) * spawn_distance;
    }
  }
  std::fill_n(sim->agent_view.begin(), VIEW_RAYS, ZERO);
  return sim;
}


void Sim2DCircles::CreateFood(size_t index) {
  sim_t px = GenUnit();
  sim_t py = GenUnit();
  sim_t plen = std::sqrt(px * px + py * py);
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


sim_t Sim2DCircles::GenUnit(void) {
  std::uniform_real_distribution<sim_t> dist(NEG_ONE, LARGEST_AFTER_ONE_DIST);
  return dist(rng);
};


sim_t Sim2DCircles::GenRadius() {
  std::uniform_real_distribution<sim_t> dist(RADIUS_FOOD_MIN,
                                             RADIUS_FOOD_MAX + MIN_DELTA);
  return dist(rng);
}


sim_t Sim2DCircles::GenSpawnRadius() {
  std::uniform_real_distribution<sim_t> dist(SPAWN_RADIUS_MIN,
                                             SPAWN_RADIUS_MAX + MIN_DELTA);
  return dist(rng);
}


sim_t Sim2DCircles::LuminescenceValue(sim_t v) {
  return (v > INTENSITY_MAX) ? 0 : (ONE - (v / INTENSITY_MAX));
}


void Sim2DCircles::Step(void) {
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
  for (size_t i=0; i<NUM_FOOD_PER_AGENT; ++i) {
    // Load food information
    sim_t px_f = food_pos_x[i];
    sim_t py_f = food_pos_y[i];
    sim_t dx_f = food_dir_x[i];
    sim_t dy_f = food_dir_y[i];
    sim_t r_f = food_radius[i];

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
    collision_check[i] = f_distance - f_2;
  }

  // For possible collisions, determine if an actual collision occurred
  for (size_t i=0; i<NUM_FOOD_PER_AGENT; ++i) {
    // Load food information
    sim_t px_f = food_pos_x[i];
    sim_t py_f = food_pos_y[i];
    sim_t dx_f = food_dir_x[i];
    sim_t dy_f = food_dir_y[i];
    sim_t r_f = food_radius[i];
    sim_t energy_f = food_energy[i];

    if (collision_check[i] >= 0) {
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
          CreateFood(i);
          continue;
        }
      }
    }
    if (energy_f - ENERGY_CONSUMPTION_FOOD_MOVE <= 0) {
      CreateFood(i);
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
  for (size_t i=0; i<NUM_FOOD_PER_AGENT; ++i) {
    // Load food information
    sim_t px_f = food_pos_x[i];
    sim_t py_f = food_pos_y[i];
    sim_t dx_f = food_dir_x[i];
    sim_t dy_f = food_dir_y[i];
    sim_t r_f = food_radius[i];
    sim_t energy_f = food_energy[i];

    // Update food position
    food_pos_x[i] = fma(SPEED_FOOD_MOVE, dx_f, px_f);
    food_pos_y[i] = fma(SPEED_FOOD_MOVE, dy_f, py_f);
    food_energy[i] = energy_f - ENERGY_CONSUMPTION_FOOD_MOVE;
  }

  // Generate agent view
  for (size_t i=0; i<VIEW_RAYS; ++i) {
    sim_t cos_t = COS_ANGLES[i];
    sim_t sin_t = SIN_ANGLES[i];
    rays_dx[i] = dx_a * cos_t - dy_a * sin_t;
    rays_dy[i] = dx_a * sin_t + dy_a * cos_t;
  }
  std::fill_n(working_agent_view.begin(), VIEW_RAYS, MAX_VALUE);

  for (size_t i=0; i<NUM_FOOD_PER_AGENT; ++i) {
    // Load food information
    sim_t px_f = food_pos_x[i];
    sim_t py_f = food_pos_y[i];
    sim_t dx_f = food_dir_x[i];
    sim_t dy_f = food_dir_y[i];
    sim_t r_f = food_radius[i];

    sim_t E = r_f + r_a;

    for (size_t ray_i=0; ray_i<VIEW_RAYS; ++ray_i) {
      sim_t ray_distance = working_agent_view[ray_i];

      sim_t A = px_a - px_f;
      sim_t C = py_a - py_f;
      sim_t F = r_f * r_f;

      // sim_t a = ONE;
      sim_t b_partial = A * rays_dx[ray_i] + C * rays_dy[ray_i];
      sim_t c = A * A + C * C - F;
      sim_t discriminant = fma(b_partial, b_partial, -c);
      if (discriminant >= 0) { // does intersect
        sim_t q = -(-b_partial +
                    std::copysign(std::sqrt(discriminant), -b_partial));
        sim_t t1 = q; // q / a -> q / 1 -> q
        sim_t t2 = c / q;
        sim_t t = ray_distance;
        if (t1 > RADIUS_AGENT && t1 < t) {
          t = t1; 
        }
        if (t2 > RADIUS_AGENT && t2 < t) {
          t = t2;
        }
        if (t < ray_distance) {
          working_agent_view[ray_i] = t;
        }
      }
    }
  }

  for (size_t i=0; i<VIEW_RAYS; ++i) {
    agent_view[i] = LuminescenceValue(working_agent_view[i]);
  }

  return;
}


std::unique_ptr<Interface> CreateSim2DCircles(
    jms::utils::random_helper::optional_seed_input_t seed) {
  return Sim2DCircles::Create(seed);
}


} // namespace sim
} // namespace jms
