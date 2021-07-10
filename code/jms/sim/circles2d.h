#ifndef JMS_SIM_CIRCLES2D_H
#define JMS_SIM_CIRCLES2D_H


#include <array>
#include <memory>
#include <numbers>
#include <optional>
#include <type_traits>
#include <vector>

#include "jms/utils/random_helper.h"


namespace jms {
namespace sim {
namespace Circles2D {


template <typename T>
concept SimType = std::is_floating_point_v<T>;
template <SimType T> class Sim;


constexpr size_t VIEW_RAYS = 1024;
constexpr float VIEW_RAYS_ANGLE_RANGE = std::numbers::pi / 4.0; // 45deg
constexpr float VIEW_RAYS_ANGLE_DELTA = VIEW_RAYS_ANGLE_RANGE / VIEW_RAYS;
// Start negative side up by 1 delta to account for an even number of rays as
// we want to ensure the forward direction is represented.
constexpr float VIEW_RAYS_ANGLE_START = -(VIEW_RAYS_ANGLE_RANGE / 2) + VIEW_RAYS_ANGLE_DELTA;


template <SimType T>
struct Config {
  static constexpr size_t NUM_FOOD_PER_AGENT = 8192;
  static constexpr T SPACE_UNIT = 0.0001; // used below
  static constexpr T ENERGY_CONSUMPTION_FOOD_IDLE = 0.5;
  static constexpr T ENERGY_CONSUMPTION_FOOD_MOVE = 1;
  static constexpr T ENERGY_CONSUMPTION_AGENT_IDLE = 1;
  static constexpr T ENERGY_CONSUMPTION_AGENT_MOVE = 1;
  static constexpr T ENERGY_CONSUMPTION_AGENT_ROTATION = 2.0 / std::numbers::pi;
  static constexpr T ENERGY_CONSUMPTION_AGENT_SPRINT = 15;
  static constexpr T ENERGY_GAIN_FOOD = 1000;
  static constexpr T ENERGY_START_AGENT = 100000;
  static constexpr T ENERGY_START_FOOD = 1000;
  static constexpr T RADIUS_AGENT = 2 * SPACE_UNIT;
  static constexpr T RADIUS_AGENT_SQUARED = RADIUS_AGENT * RADIUS_AGENT;
  static constexpr T RADIUS_FOOD_MIN = 0.5 * SPACE_UNIT;
  static constexpr T RADIUS_FOOD_MAX = 6 * SPACE_UNIT;
  static constexpr T RADIUS_FOOD_MIN_DELTA = std::nextafter(RADIUS_FOOD_MAX, std::numeric_limits<T>::max());
  static constexpr T RADIUS_FOOD_MAX_WITH_DELTA = RADIUS_FOOD_MAX + RADIUS_FOOD_MIN_DELTA;
  static constexpr T SPAWN_RADIUS_MIN = 10 * SPACE_UNIT;
  static constexpr T SPAWN_RADIUS_MAX = 999 * SPACE_UNIT;
  static constexpr T SPAWN_RADIUS_MIN_DELTA = std::nextafter(SPAWN_RADIUS_MAX, std::numeric_limits<T>::max());
  static constexpr T SPAWN_RADIUS_MAX_WITH_DELTA = SPAWN_RADIUS_MAX + SPAWN_RADIUS_MIN_DELTA;
  static constexpr T SPAWN_RADIUS_DELTA = SPAWN_RADIUS_MAX - SPAWN_RADIUS_MIN;
  static constexpr T SPEED_AGENT_MOVE = 5 * SPACE_UNIT;
  static constexpr T SPEED_AGENT_SPRINT = 10 * SPACE_UNIT;
  static constexpr T SPEED_FOOD_MOVE = 1 * SPACE_UNIT;
  static constexpr T UNIT_MIN_DELTA = std::nextafter(static_cast<T>(1), std::numeric_limits<T>::max());
  static constexpr T UNIT_MAX_WITH_DELTA = 1 + UNIT_MIN_DELTA;

  static constexpr T VIEW_DISTANCE = 500 * SPACE_UNIT;
  static constexpr T INTENSITY_MAX = 999 * SPACE_UNIT;
  static constexpr T INTENSITY_MAX_INV_NEG = -1 / INTENSITY_MAX;
};


template <SimType T>
struct Position {
  T x;
  T y;
};


template <SimType T>
class PositionInfo {
private:
  const Sim<T>& sim;
  size_t agent_index;
  size_t food_index;

public:
  PositionInfo(const Sim<T>& sim) noexcept : sim(sim), agent_index(0), food_index(0) { return; }
  std::optional<Position<T>> Agent(void) noexcept {
    if (agent_index >= sim.agent.size()) { return std::nullopt; }
    agent_index++;
    return Position<T>{.x=sim.agent[agent_index-1].pos_x, .y=sim.agent[agent_index-1].pos_y};
  }
  std::optional<Position<T>> Food(void) noexcept {
    if (food_index >= Config<T>::NUM_FOOD_PER_AGENT) { return std::nullopt; }
    food_index++;
    return Position<T>{.x=sim.food_pos_x[food_index-1], .y=sim.food_pos_y[food_index-1]};
  }
  void ResetFood(void) noexcept { agent_index = 0; food_index = 0; return; }
};


template <SimType T>
struct alignas(32) SimDataUnit {
  T pos_x;  // 4
  T pos_y;  // 8
  T dir_x;  // 12
  T dir_y;  // 16
  T radius; // 20
  T energy; // 24
  T speed;  // 28
  T dummy;  // 32
};


template <SimType T>
class Sim {
public:
  // Based on counterclockwise rotation sweeping from left to right.
  alignas(64) static const std::array<T, VIEW_RAYS> COS_ANGLES;
  alignas(64) static const std::array<T, VIEW_RAYS> SIN_ANGLES;
  friend class PositionInfo<T>;

protected:
  alignas(64) std::array<T, Config<T>::NUM_FOOD_PER_AGENT> food_pos_x;
  alignas(64) std::array<T, Config<T>::NUM_FOOD_PER_AGENT> food_pos_y;
  alignas(64) std::array<T, Config<T>::NUM_FOOD_PER_AGENT> food_dir_x;
  alignas(64) std::array<T, Config<T>::NUM_FOOD_PER_AGENT> food_dir_y;
  alignas(64) std::array<T, Config<T>::NUM_FOOD_PER_AGENT> food_radius;
  alignas(64) std::array<T, Config<T>::NUM_FOOD_PER_AGENT> food_energy;
  alignas(64) std::array<T, Config<T>::NUM_FOOD_PER_AGENT> food_speed;
  alignas(64) std::array<SimDataUnit<T>, 1> agent;
  alignas(64) std::array<T, VIEW_RAYS> agent_view;

private:
  // partial results used during calculations.
  alignas(64) std::array<T, Config<T>::NUM_FOOD_PER_AGENT> collision_check;
  alignas(64) std::array<T, VIEW_RAYS> rays_dx;
  alignas(64) std::array<T, VIEW_RAYS> rays_dy;
  alignas(64) std::array<T, VIEW_RAYS> working_agent_view;
protected:
  std::mt19937_64 rng; // each simulation owns its own generator

public:
  Sim(void) noexcept = default;
  Sim(const Sim&) = delete;
  Sim(Sim&&) noexcept = default;
  ~Sim(void) noexcept = default;
  Sim& operator=(const Sim&) = delete;
  Sim& operator=(Sim&&) noexcept = default;

  static std::unique_ptr<Sim<T>> Create(std::optional<int32_t> opt_seed=std::nullopt,
                                        jms::utils::random_helper::optional_seed_seq_input_t opt_seed_seq_vec=std::nullopt,
                                        std::optional<uint32_t> opt_discard=std::nullopt);
  PositionInfo<T> GetPositionInfoAccess(void) const noexcept { return PositionInfo<T>{*this}; }
  void Step(void) noexcept;

protected:
  inline T GenUnit(void) noexcept {
    std::uniform_real_distribution<T> dist(static_cast<T>(-1), Config<T>::UNIT_MAX_WITH_DELTA);
    return dist(rng);
  }
  inline T GenRadius() noexcept {
    std::uniform_real_distribution<T> dist(Config<T>::RADIUS_FOOD_MIN, Config<T>::RADIUS_FOOD_MAX_WITH_DELTA);
    return dist(rng);
  }
  inline T GenSpawnRadius() noexcept {
    std::uniform_real_distribution<T> dist(Config<T>::SPAWN_RADIUS_MIN, Config<T>::SPAWN_RADIUS_MAX_WITH_DELTA);
    return dist(rng);
  }
  void InitializeAgent(size_t index=0) noexcept;
  void InitializeFood(size_t index) noexcept;
  inline T LuminescenceValue(T v) noexcept {
    //return (v > INTENSITY_MAX) ? 0 : (1 - (v / INTENSITY_MAX));
    // 1 - v / IM = 1 - v * (1/IM) = -(v * (1/IM)) + 1 = -v * (1/IM) + 1 = v * (-1/IM) + 1
    return (v > Config<T>::INTENSITY_MAX) ? 0 : std::fma(v, Config<T>::INTENSITY_MAX_INV_NEG, 1);
  }
};


template <SimType T>
const std::array<T, VIEW_RAYS> Sim<T>::COS_ANGLES{
  []() constexpr {
    std::array<T, VIEW_RAYS> x{};
    for (size_t index=0; index<VIEW_RAYS; ++index) {
      x[index] = std::cos(VIEW_RAYS_ANGLE_START + (static_cast<T>(index) * VIEW_RAYS_ANGLE_DELTA));
    }
    return x;
  }()};


template <SimType T>
const std::array<T, VIEW_RAYS> Sim<T>::SIN_ANGLES{
  []() constexpr {
    std::array<T, VIEW_RAYS> x{};
    for (size_t index=0; index<VIEW_RAYS; ++index) {
      x[index] = std::sin(VIEW_RAYS_ANGLE_START + (static_cast<T>(index) * VIEW_RAYS_ANGLE_DELTA));
    }
    return x;
  }()};


template <SimType T>
std::unique_ptr<Sim<T>> Sim<T>::Create(std::optional<int32_t> opt_seed,
                                       jms::utils::random_helper::optional_seed_seq_input_t opt_seed_seq_vec,
                                       std::optional<uint32_t> opt_discard) {
  std::unique_ptr<Sim<T>> sim{new Sim<T>{}};
  jms::utils::random_helper::Initialize(sim->rng, opt_seed, opt_seed_seq_vec, opt_discard);
  sim->InitializeAgent();
  for (size_t index=0; index<Config<T>::NUM_FOOD_PER_AGENT; ++index) { sim->InitializeFood(index); }
  std::fill_n(sim->agent_view.begin(), VIEW_RAYS, 0);
  return sim;
}


template <SimType T>
void Sim<T>::InitializeAgent(size_t index) noexcept {
  T dx = GenUnit();
  T dy = GenUnit();
  T dlen = std::sqrt(std::fma(dx, dx, dy * dy));
  agent[index].pos_x = 0;
  agent[index].pos_y = 0;
  if (dlen == 0) {
    agent[index].dir_x = 0;
    agent[index].dir_y = 1;
  } else {
    agent[index].dir_x = dx / dlen;
    agent[index].dir_y = dy / dlen;
  }
  agent[index].radius = Config<T>::RADIUS_AGENT;
  agent[index].energy = Config<T>::ENERGY_START_AGENT;
  agent[index].speed = 0;
  return;
}


template <SimType T>
void Sim<T>::InitializeFood(size_t index) noexcept {
  T px = GenUnit();
  T py = GenUnit();
  T plen = std::sqrt(std::fma(px, px, py * py));
  T spawn_distance = GenSpawnRadius();
  T dx = GenUnit();
  T dy = GenUnit();
  T dlen = std::sqrt(std::fma(dx, dx, dy * dy));
  // place relative to the agent
  if (plen == 0) {
    food_pos_x[index] = agent[index].pos_x;
    food_pos_y[index] = spawn_distance + agent[index].pos_y;
  } else {
    food_pos_x[index] = std::fma(px / plen, spawn_distance, agent[index].pos_x);
    food_pos_y[index] = std::fma(py / plen, spawn_distance, agent[index].pos_y);
  }
  if (dlen == 0) {
    food_dir_x[index] = 0;
    food_dir_y[index] = 1;
  } else {
    food_dir_x[index] = dx / dlen;
    food_dir_y[index] = dy / dlen;
  }
  food_radius[index] = GenRadius();
  food_energy[index] = Config<T>::ENERGY_START_FOOD;
  food_speed[index] = Config<T>::SPEED_FOOD_MOVE;
  return;
}


template <SimType T>
void Sim<T>::Step(void) noexcept {
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
  size_t agent_index = 0;
  // Load agent info.
  T px_a = agent[agent_index].pos_x;
  T py_a = agent[agent_index].pos_y;
  T dx_a = agent[agent_index].dir_x;
  T dy_a = agent[agent_index].dir_y;
  T r_a = agent[agent_index].radius;
  T energy_a = agent[agent_index].energy;
  T speed_a = agent[agent_index].speed;

  // High level (initial) check for food/agent collision by creating representative circles
  // for each circle that covers the distance covered in this step.
  T H = speed_a * 0.5;
  for (size_t food_index=0; food_index<Config<T>::NUM_FOOD_PER_AGENT; ++food_index) {
    // Load food information
    T px_f = food_pos_x[food_index];
    T py_f = food_pos_y[food_index];
    T dx_f = food_dir_x[food_index];
    T dy_f = food_dir_y[food_index];
    T r_f = food_radius[food_index];
    T s_f = food_speed[food_index];

    T A = px_f - px_a;
    T C = py_f - py_a;
    T E = r_f + r_a;
    T I = s_f * 0.5;
    T J = std::fma(I, dx_f, -H * dx_a); // I * dx_f - H * dx_a
    T K = std::fma(I, dy_f, -H * dy_a); // I * dy_f - H * dy_a

    // Test for possible intersection
    T fx = A + J;
    T fy = C + K;
    T f_2 = std::fma(fx, fx, fy * fy);
    T f_distance = E + H + I;
    // ||M1M2||^2 <= (R_M1 + R_M2)^2  ->  (R_M1 + R_M2)^2 - ||M1M2||^2 >= 0 -> collision
    collision_check[food_index] = std::fma(f_distance, f_distance, -f_2);
  }

  // For possible collisions, determine if an actual collision occurred
  for (size_t food_index=0; food_index<Config<T>::NUM_FOOD_PER_AGENT; ++food_index) {
    T energy_f = food_energy[food_index];

    if (collision_check[food_index] >= 0) {
      // Load food information
      T px_f = food_pos_x[food_index];
      T py_f = food_pos_y[food_index];
      T dx_f = food_dir_x[food_index];
      T dy_f = food_dir_y[food_index];
      T r_f = food_radius[food_index];

      // Reused calculations
      T A = px_f - px_a;
      T B = dx_f - dx_a;
      T C = py_f - py_a;
      T D = dy_f - dy_a;
      T E = r_f + r_a;
      T F = E * E;

      // Check for intersection in motion
      T a = std::fma(B, B, D * D);
      T b_partial = std::fma(A, B, C * D);
      T c = std::fma(A, A, std::fma(C, C, -F));
      T discriminant = std::fma(b_partial, b_partial, -a * c);
      if (discriminant > 0) {
        T q = -(b_partial + std::copysign(std::sqrt(discriminant), b_partial));
        T t1 = q / a;
        T t2 = c / q;
        if (t1 >= 0 or t2 >= 0) {
          // collision
          energy_a += Config<T>::ENERGY_GAIN_FOOD;
          // agent just consumed food, so "generate" a new one (reuse data structure)
          InitializeFood(food_index);
          continue;
        }
      }
    }
    // Food out of energy, so "generate" a new one (reuse data structure)
    if (energy_f - Config<T>::ENERGY_CONSUMPTION_FOOD_MOVE <= 0) {
      InitializeFood(food_index);
    }
  }

  // Update agent
  if (speed_a > 0) {
    px_a = std::fma(speed_a, dx_a, px_a);
    py_a = std::fma(speed_a, dy_a, py_a);
    agent[agent_index].pos_x = px_a;
    agent[agent_index].pos_y = py_a;
  }
  agent[agent_index].energy = energy_a;

  // Move food
  for (size_t index=0; index<Config<T>::NUM_FOOD_PER_AGENT; ++index) {
    // Load food information
    T px_f = food_pos_x[index];
    T py_f = food_pos_y[index];
    T dx_f = food_dir_x[index];
    T dy_f = food_dir_y[index];
    T r_f = food_radius[index];
    T energy_f = food_energy[index];
    T s_f = food_speed[index];

    // Update food position
    food_pos_x[index] = std::fma(s_f, dx_f, px_f);
    food_pos_y[index] = std::fma(s_f, dy_f, py_f);
    // should never be less than zero; checked above and reinitialized above.
    food_energy[index] = energy_f - Config<T>::ENERGY_CONSUMPTION_FOOD_MOVE;
  }

  // Generate agent view
  for (size_t index=0; index<VIEW_RAYS; ++index) {
    T cos_t = COS_ANGLES[index];
    T sin_t = SIN_ANGLES[index];
    rays_dx[index] = std::fma(dx_a, cos_t, -dy_a * sin_t);
    rays_dy[index] = std::fma(dx_a, sin_t, dy_a * cos_t);
  }
  std::fill_n(working_agent_view.begin(), VIEW_RAYS, std::numeric_limits<T>::max());

  // Slowest unit of this function by a few orders of magnitude.
  for (size_t food_index=0; food_index<Config<T>::NUM_FOOD_PER_AGENT; ++food_index) {
    // Load food information
    T px_f = food_pos_x[food_index];
    T py_f = food_pos_y[food_index];
    T dx_f = food_dir_x[food_index];
    T dy_f = food_dir_y[food_index];
    T r_f = food_radius[food_index];

    T A = px_a - px_f;
    T C = py_a - py_f;
    T F = r_f * r_f;
    T G = std::fma(A, A, C * C) - F;
    T H = -G;

    // test if food is touching ends or in the middle of view range.
    // left
    T rays_dx_l = rays_dx[0];
    T rays_dy_l = rays_dy[0];
    T b_partial_l = std::fma(A, rays_dx_l, C * rays_dy_l);
    T discriminant_l = std::fma(b_partial_l, b_partial_l, H);
    // right
    T rays_dx_r = rays_dx[VIEW_RAYS - 1];
    T rays_dy_r = rays_dy[VIEW_RAYS - 1];
    T b_partial_r = std::fma(A, rays_dx_r, C * rays_dy_r);
    T discriminant_r = std::fma(b_partial_r, b_partial_r, H);
    if (discriminant_l < 0 && discriminant_r < 0) { // no intersection
      T af_x = px_f - px_a;
      T af_y = py_f - py_a;
      // ax * by - ay * bx : a == ray; b == af (agent to food vector; f - a)
      T dir_l = std::fma(rays_dx_l, af_y, -rays_dy_l * af_x);
      T dir_r = std::fma(rays_dx_r, af_y, -rays_dy_r * af_x);
      // in between = dir_l <= 0 (to the right) && dir_r >= 0 (to the left)
      // check for not between
      if (dir_l > 0 || dir_r < 0) {
        continue;
      }
    }

    for (size_t ray_index; ray_index<VIEW_RAYS; ++ray_index) {
      // T a = 1;
      T b_partial = std::fma(A, rays_dx[ray_index], C * rays_dy[ray_index]);
      T discriminant = std::fma(b_partial, b_partial, H);
      if (discriminant >= 0) {
        T ray_distance = working_agent_view[ray_index];
        T x = ray_distance;
        // x1 = q / a -> q / 1 -> q
        T x1 = -(-b_partial +
                    std::copysign(std::sqrt(discriminant), -b_partial));
        if (x1 > Config<T>::RADIUS_AGENT && x1 < x) {
          x = x1;
        }
        if (x1 != 0) {
          T x2 = G / x1;
          if (x2 > Config<T>::RADIUS_AGENT && x2 < x) {
            x = x2;
          }
        }
        if (x < ray_distance) {
          working_agent_view[ray_index] = x;
        }
      }
    }
  }

  for (size_t index; index<VIEW_RAYS; ++index) {
    agent_view[index] = LuminescenceValue(working_agent_view[index]);
  }

  return;
}


/*
void View(void) {


  return;
}
*/


} // namespace Circles2D_f
} // namespace sim
} // namespace jms


#endif // JMS_SIM_CIRCLES2D_H
