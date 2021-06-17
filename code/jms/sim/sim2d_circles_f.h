#ifndef SIM2d_CIRCLES_F_H
#define SIM2d_CIRCLES_F_H


#include <iterator>
#include <numbers>
#include <optional>
#include <vector>

#include "jms/utils/random_helper.h"


namespace jms {
namespace sim {
namespace Circles2D_f {


constexpr float SPACE_UNIT = 0.0001f; // used below
constexpr size_t AGENT_POS_X = 0;
constexpr size_t AGENT_POS_Y = 1;
constexpr size_t AGENT_DIR_X = 2;
constexpr size_t AGENT_DIR_Y = 3;
constexpr size_t AGENT_RADIUS = 4;
constexpr size_t AGENT_ENERGY = 5;
constexpr size_t AGENT_SPEED = 6;
constexpr float ENERGY_CONSUMPTION_FOOD_IDLE = 0.5f;
constexpr float ENERGY_CONSUMPTION_FOOD_MOVE = 1.0f;
constexpr float ENERGY_CONSUMPTION_AGENT_IDLE = 1.0f;
constexpr float ENERGY_CONSUMPTION_AGENT_MOVE = 1.0f;
constexpr float ENERGY_CONSUMPTION_AGENT_ROTATION = static_cast<float>(2.0 / std::numbers::pi);
constexpr float ENERGY_CONSUMPTION_AGENT_SPRINT = 15.0f;
constexpr float ENERGY_GAIN_FOOD = 1000.0f;
constexpr float ENERGY_START_AGENT = 10000.0f;
constexpr float ENERGY_START_FOOD = 1000.0f;
constexpr float INTENSITY_MAX = 999.0f * SPACE_UNIT;
constexpr float INTENSITY_MAX_INV_NEG = -1.0f / INTENSITY_MAX;
constexpr size_t NUM_FOOD_PER_AGENT = 8192;
constexpr float RADIUS_AGENT = 2.0f * SPACE_UNIT;
constexpr float RADIUS_AGENT_SQUARED = RADIUS_AGENT * RADIUS_AGENT;
constexpr float RADIUS_FOOD_MIN = 0.5f * SPACE_UNIT;
constexpr float RADIUS_FOOD_MAX = 6.0f * SPACE_UNIT;
constexpr float RADIUS_FOOD_MIN_DELTA = std::nextafter(RADIUS_FOOD_MAX, std::numeric_limits<float>::max());
constexpr float RADIUS_FOOD_MAX_WITH_DELTA = RADIUS_FOOD_MAX + RADIUS_FOOD_MIN_DELTA;
constexpr float SPAWN_RADIUS_MIN = 10.0f * SPACE_UNIT;
constexpr float SPAWN_RADIUS_MAX = 999.0f * SPACE_UNIT;
constexpr float SPAWN_RADIUS_MIN_DELTA = std::nextafter(SPAWN_RADIUS_MAX, std::numeric_limits<float>::max());
constexpr float SPAWN_RADIUS_MAX_WITH_DELTA = SPAWN_RADIUS_MAX + SPAWN_RADIUS_MIN_DELTA;
constexpr float SPAWN_RADIUS_DELTA = SPAWN_RADIUS_MAX - SPAWN_RADIUS_MIN;
constexpr float SPEED_AGENT_MOVE = 5.0f * SPACE_UNIT;
constexpr float SPEED_AGENT_SPRINT = 10.0f * SPACE_UNIT;
constexpr float SPEED_FOOD_MOVE = 1.0f * SPACE_UNIT;
constexpr float UNIT_MIN_DELTA = std::nextafter(1.0f, std::numeric_limits<float>::max());
constexpr float UNIT_MAX_WITH_DELTA = 1.0f + UNIT_MIN_DELTA;
constexpr size_t VIEW_RAYS = 1024;
constexpr float VIEW_RAYS_ANGLE_RANGE = static_cast<float>(std::numbers::pi / 4.0); // 45deg
constexpr float VIEW_RAYS_ANGLE_DELTA = VIEW_RAYS_ANGLE_RANGE / static_cast<float>(VIEW_RAYS);
// Start negative side up by 1 delta to account for an even number of rays as
// we want to ensure the forward direction is represented.
constexpr float VIEW_RAYS_ANGLE_START = -(VIEW_RAYS_ANGLE_RANGE / 2.0f) + VIEW_RAYS_ANGLE_DELTA;
constexpr float VIEW_DISTANCE = 500.0f * SPACE_UNIT;


struct Position {
  float x;
  float y;
};


class PositionInfo {
private:
  const std::array<float, NUM_FOOD_PER_AGENT>& food_pos_x;
  const std::array<float, NUM_FOOD_PER_AGENT>& food_pos_y;
  const std::array<float, 16>& agent_info;
  size_t index;

public:
  PositionInfo(const std::array<float, NUM_FOOD_PER_AGENT>& food_pos_x,
               const std::array<float, NUM_FOOD_PER_AGENT>& food_pos_y,
               const std::array<float, 16>& agent_info)
  : food_pos_x(food_pos_x), food_pos_y(food_pos_y), agent_info(agent_info), index(0) {
    return;
  }

  Position Agent(void) const noexcept { return Position {.x=agent_info[AGENT_POS_X], .y=agent_info[AGENT_POS_Y]}; }
  std::optional<Position> NextFood(void) noexcept {
    if (index >= NUM_FOOD_PER_AGENT) { return std::optional<Position>(); }
    index++;
    return std::optional<Position>(Position {.x=food_pos_x[index-1], .y=food_pos_y[index-1]});
  }
  void ResetFood(void) noexcept { index = 0; return; }
};


class Sim {
public:
  // Based on counterclockwise rotation sweeping from left to right.
  alignas(64) static const std::array<float, VIEW_RAYS> COS_ANGLES;
  alignas(64) static const std::array<float, VIEW_RAYS> SIN_ANGLES;

protected:
  alignas(64) std::array<float, NUM_FOOD_PER_AGENT> food_pos_x;
  alignas(64) std::array<float, NUM_FOOD_PER_AGENT> food_pos_y;
  alignas(64) std::array<float, NUM_FOOD_PER_AGENT> food_dir_x;
  alignas(64) std::array<float, NUM_FOOD_PER_AGENT> food_dir_y;
  alignas(64) std::array<float, NUM_FOOD_PER_AGENT> food_radius;
  alignas(64) std::array<float, NUM_FOOD_PER_AGENT> food_energy;
  // extra for growth and alignment
  alignas(64) std::array<float, 16> agent_info;
  alignas(64) std::array<float, VIEW_RAYS> agent_view;
private:
  // partial results used during calculations.
  alignas(64) std::array<float, NUM_FOOD_PER_AGENT> collision_check;
  alignas(64) std::array<float, VIEW_RAYS> rays_dx;
  alignas(64) std::array<float, VIEW_RAYS> rays_dy;
  alignas(64) std::array<float, VIEW_RAYS> working_agent_view;
protected:
  std::mt19937_64 rng; // each simulation owns its own generator

public:
  Sim(void) noexcept = default;
  Sim(const Sim&) = delete;
  Sim(Sim&&) noexcept = default;
  ~Sim(void) noexcept = default;
  Sim& operator=(const Sim&) = delete;
  Sim& operator=(Sim&&) noexcept = default;

  static std::unique_ptr<Sim> Create(jms::utils::random_helper::optional_seed_input_t seed=std::nullopt);
  PositionInfo GetPositionInfoAccess(void) const noexcept { return PositionInfo {food_pos_x, food_pos_y, agent_info}; }
  void Step(void) noexcept;

protected:
  inline float GenUnit(void) noexcept {
    std::uniform_real_distribution<float> dist(-1.0f, UNIT_MAX_WITH_DELTA);
    return dist(rng);
  }
  inline float GenRadius() noexcept {
    std::uniform_real_distribution<float> dist(RADIUS_FOOD_MIN, RADIUS_FOOD_MAX_WITH_DELTA);
    return dist(rng);
  }
  inline float GenSpawnRadius() noexcept {
    std::uniform_real_distribution<float> dist(SPAWN_RADIUS_MIN, SPAWN_RADIUS_MAX_WITH_DELTA);
    return dist(rng);
  }
  void InitializeAgent(void) noexcept;
  void InitializeFood(size_t index) noexcept;
  inline float LuminescenceValue(float v) noexcept {
    //return (v > INTENSITY_MAX) ? 0.0f : (1.0f - (v / INTENSITY_MAX));
    // 1 - v / IM = 1 - v * (1/IM) = -(v * (1/IM)) + 1 = -v * (1/IM) + 1 = v * (-1/IM) + 1
    return (v > INTENSITY_MAX) ? 0.0f : std::fma(v, INTENSITY_MAX_INV_NEG, 1.0f);
  }
};


const std::array<float, VIEW_RAYS> Sim::COS_ANGLES{
  []() constexpr {
    std::array<float, VIEW_RAYS> x{};
    for (size_t index : std::ranges::iota_view {static_cast<size_t>(0), VIEW_RAYS}) {
      x[index] = std::cos(VIEW_RAYS_ANGLE_START + (static_cast<float>(index) * VIEW_RAYS_ANGLE_DELTA));
    }
    return x;
  }()};


const std::array<float, VIEW_RAYS> Sim::SIN_ANGLES{
  []() constexpr {
    std::array<float, VIEW_RAYS> x{};
    for (size_t index : std::ranges::iota_view {static_cast<size_t>(0), VIEW_RAYS}) {
      x[index] = std::sin(VIEW_RAYS_ANGLE_START + (static_cast<float>(index) * VIEW_RAYS_ANGLE_DELTA));
    }
    return x;
  }()};


std::unique_ptr<Sim> Sim::Create(jms::utils::random_helper::optional_seed_input_t seed) {
  std::unique_ptr<Sim> sim{new Sim{}};
  jms::utils::random_helper::SetSeed_mt19937_64(sim->rng, seed);
  sim->InitializeAgent();
  for (size_t index : std::ranges::iota_view {static_cast<size_t>(0), NUM_FOOD_PER_AGENT}) { sim->InitializeFood(index); }
  std::fill_n(sim->agent_view.begin(), VIEW_RAYS, 0.0f);
  return sim;
}


void Sim::InitializeAgent(void) noexcept {
  float dx = GenUnit();
  float dy = GenUnit();
  float dlen = std::sqrt(std::fma(dx, dx, dy * dy));
  agent_info[AGENT_POS_X] = 0.0f;
  agent_info[AGENT_POS_Y] = 0.0f;
  if (dlen == 0.0f) {
    agent_info[AGENT_DIR_X] = 0.0f;
    agent_info[AGENT_DIR_Y] = 1.0f;
  } else {
    agent_info[AGENT_DIR_X] = dx / dlen;
    agent_info[AGENT_DIR_Y] = dy / dlen;
  }
  agent_info[AGENT_RADIUS] = RADIUS_AGENT;
  agent_info[AGENT_ENERGY] = ENERGY_START_AGENT;
  agent_info[AGENT_SPEED] = 0.0f;
  return;
}


void Sim::InitializeFood(size_t index) noexcept {
  float px = GenUnit();
  float py = GenUnit();
  float plen = std::sqrt(std::fma(px, px, py * py));
  float spawn_distance = GenSpawnRadius();
  float dx = GenUnit();
  float dy = GenUnit();
  float dlen = std::sqrt(std::fma(dx , dx, dy * dy));
  // place relative to the agent
  if (plen == 0.0f) {
    food_pos_x[index] = agent_info[AGENT_POS_X];
    food_pos_y[index] = spawn_distance + agent_info[AGENT_POS_Y];
  } else {
    food_pos_x[index] = std::fma(px / plen, spawn_distance, agent_info[AGENT_POS_X]);
    food_pos_y[index] = std::fma(py / plen, spawn_distance, agent_info[AGENT_POS_Y]);
  }
  if (dlen == 0.0f) {
    food_dir_x[index] = 0.0f;
    food_dir_y[index] = 1.0f;
  } else {
    food_dir_x[index] = dx / dlen;
    food_dir_y[index] = dy / dlen;
  }
  food_radius[index] = GenRadius();
  food_energy[index] = ENERGY_START_FOOD;
  return;
}


void Sim::Step(void) noexcept {
  /***
   * Process-
   * for each food
   *   1. Check for possible intersections
   *   2. For each possible collision, check for agent/food collision and food out of energy (create new)
   *   3. Update agent_view
   *   4. Move food
   *   5. Create agent view
   * Update agent
   */
  // Load agent info.
  float px_a = agent_info[AGENT_POS_X];
  float py_a = agent_info[AGENT_POS_Y];
  float dx_a = agent_info[AGENT_DIR_X];
  float dy_a = agent_info[AGENT_DIR_Y];
  float r_a = agent_info[AGENT_RADIUS];
  float energy_a = agent_info[AGENT_ENERGY];
  float speed_a = agent_info[AGENT_SPEED];

  // High level (initial) check for food/agent collision by creating representative circles
  // for each circle that covers the distance covered in this step.
  float H = speed_a * 0.5f;
  float I = SPEED_FOOD_MOVE * 0.5f;
  for (size_t index : std::ranges::iota_view {static_cast<size_t>(0), NUM_FOOD_PER_AGENT}) {
    // Load food information
    float px_f = food_pos_x[index];
    float py_f = food_pos_y[index];
    float dx_f = food_dir_x[index];
    float dy_f = food_dir_y[index];
    float r_f = food_radius[index];

    float A = px_f - px_a;
    float C = py_f - py_a;
    float E = r_f + r_a;
    float J = std::fma(I, dx_f, -H * dx_a); // I * dx_f - H * dx_a
    float K = std::fma(I, dy_f, -H * dy_a); // I * dy_f - H * dy_a

    // Test for possible intersection
    float fx = A + J;
    float fy = C + K;
    float f_2 = std::fma(fx, fx, fy * fy);
    float f_distance = E + H + I;
    // ||M1M2||^2 <= (R_M1 + R_M2)^2  ->  (R_M1 + R_M2)^2 - ||M1M2||^2 >= 0 -> collision
    collision_check[index] = std::fma(f_distance, f_distance, -f_2);
  }

  // For possible collisions, determine if an actual collision occurred
  for (size_t index : std::ranges::iota_view {static_cast<size_t>(0), NUM_FOOD_PER_AGENT}) {
    float energy_f = food_energy[index];

    if (collision_check[index] >= 0.0f) {
      // Load food information
      float px_f = food_pos_x[index];
      float py_f = food_pos_y[index];
      float dx_f = food_dir_x[index];
      float dy_f = food_dir_y[index];
      float r_f = food_radius[index];

      // Reused calculations
      float A = px_f - px_a;
      float B = dx_f - dx_a;
      float C = py_f - py_a;
      float D = dy_f - dy_a;
      float E = r_f + r_a;
      float F = E * E;

      // Check for intersection in motion
      float a = B * B + D * D;
      float b_partial = A * B + C * D;
      float c = A * A + C * C - F;
      float discriminant = std::fma(b_partial, b_partial, -a * c);
      if (discriminant > 0.0f) {
        float q = -(b_partial +
                    std::copysign(std::sqrt(discriminant), b_partial));
        float t1 = q / a;
        float t2 = c / q;
        if (t1 >= 0.0f or t2 >= 0.0f) {
          // collision
          energy_a += ENERGY_GAIN_FOOD;
          // agent just consumed food, so "generate" a new one (reuse data structure)
          InitializeFood(index);
          continue;
        }
      }
    }
    // Food out of energy, so "generate" a new one (reuse data structure)
    if (energy_f - ENERGY_CONSUMPTION_FOOD_MOVE <= 0.0f) {
      InitializeFood(index);
    }
  }

  // Update agent
  if (speed_a > 0.0f) {
    px_a = std::fma(speed_a, dx_a, px_a);
    py_a = std::fma(speed_a, dy_a, py_a);
    agent_info[AGENT_POS_X] = px_a;
    agent_info[AGENT_POS_Y] = py_a;
  }
  agent_info[AGENT_ENERGY] = energy_a;

  // Move food
  for (size_t index : std::ranges::iota_view {static_cast<size_t>(0), NUM_FOOD_PER_AGENT}) {
    // Load food information
    float px_f = food_pos_x[index];
    float py_f = food_pos_y[index];
    float dx_f = food_dir_x[index];
    float dy_f = food_dir_y[index];
    float r_f = food_radius[index];
    float energy_f = food_energy[index];

    // Update food position
    food_pos_x[index] = std::fma(SPEED_FOOD_MOVE, dx_f, px_f);
    food_pos_y[index] = std::fma(SPEED_FOOD_MOVE, dy_f, py_f);
    // should never be less than zero; checked above and reinitialized above.
    food_energy[index] = energy_f - ENERGY_CONSUMPTION_FOOD_MOVE;
  }

  // Generate agent view
  for (size_t index : std::ranges::iota_view {static_cast<size_t>(0), VIEW_RAYS}) {
    float cos_t = COS_ANGLES[index];
    float sin_t = SIN_ANGLES[index];
    rays_dx[index] = std::fma(dx_a, cos_t, -dy_a * sin_t);
    rays_dy[index] = std::fma(dx_a, sin_t, dy_a * cos_t);
  }
  std::fill_n(working_agent_view.begin(), VIEW_RAYS, std::numeric_limits<float>::max());

  // Slowest unit of this function by a few orders of magnitude.
  for (size_t food_index : std::ranges::iota_view {static_cast<size_t>(0), NUM_FOOD_PER_AGENT}) {
    // Load food information
    float px_f = food_pos_x[food_index];
    float py_f = food_pos_y[food_index];
    float dx_f = food_dir_x[food_index];
    float dy_f = food_dir_y[food_index];
    float r_f = food_radius[food_index];

    float A = px_a - px_f;
    float C = py_a - py_f;
    float F = r_f * r_f;
    float G = std::fma(A, A, C * C) - F;
    float H = -G;

    // test if food is touching ends or in the middle of view range.
    // left
    float rays_dx_l = rays_dx[0];
    float rays_dy_l = rays_dy[0];
    float b_partial_l = std::fma(A, rays_dx_l, C * rays_dy_l);
    float discriminant_l = std::fma(b_partial_l, b_partial_l, H);
    // right
    float rays_dx_r = rays_dx[VIEW_RAYS - 1];
    float rays_dy_r = rays_dy[VIEW_RAYS - 1];
    float b_partial_r = std::fma(A, rays_dx_r, C * rays_dy_r);
    float discriminant_r = std::fma(b_partial_r, b_partial_r, H);
    if (discriminant_l < 0.0f && discriminant_r < 0.0f) { // no intersection
      float af_x = px_f - px_a;
      float af_y = py_f - py_a;
      // ax * by - ay * bx : a == ray; b == af (agent to food vector; f - a)
      float dir_l = std::fma(rays_dx_l, af_y, -rays_dy_l * af_x);
      float dir_r = std::fma(rays_dx_r, af_y, -rays_dy_r * af_x);
      // in between = dir_l <= 0 (to the right) && dir_r >= 0 (to the left)
      // check for not between
      if (dir_l > 0.0 || dir_r < 0.0) {
        continue;
      }
    }

    for (size_t ray_index : std::ranges::iota_view {static_cast<size_t>(0), VIEW_RAYS}) {
      // float a = 1.0;
      float b_partial = std::fma(A, rays_dx[ray_index], C * rays_dy[ray_index]);
      float discriminant = std::fma(b_partial, b_partial, H);
      if (discriminant >= 0.0f) {
        float ray_distance = working_agent_view[ray_index];
        float x = ray_distance;
        // x1 = q / a -> q / 1 -> q
        float x1 = -(-b_partial +
                    std::copysign(std::sqrt(discriminant), -b_partial));
        if (x1 > RADIUS_AGENT && x1 < x) {
          x = x1;
        }
        if (x1 != 0.0f) {
          float x2 = G / x1;
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

  for (size_t index : std::ranges::iota_view {static_cast<size_t>(0), VIEW_RAYS}) {
    agent_view[index] = LuminescenceValue(working_agent_view[index]);
  }

  return;
}


} // namespace Circles2D_f
} // namespace sim
} // namespace jms


#endif // SIM2d_CIRCLES_F_H
