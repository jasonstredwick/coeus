#ifndef JMS_SIM_CIRCLES2D_H
#define JMS_SIM_CIRCLES2D_H


#include <array>
#include <functional>
#include <memory>
#include <numbers>
#include <optional>
#include <type_traits>
#include <vector>

#include "jms/math/angles.h"
#include "jms/sim/controls.h"
#include "jms/utils/random_helper.h"


namespace jms {
namespace sim {
namespace Circles2D {


template <typename T> concept SimType = std::is_floating_point_v<T>;


/*
FLOAT: 32bit - 23 bit mantissa; 2^23 = 8,388,608 +/-
DoUBLE: 64bit - 52 bit mantissa; 2^52 = 4,503,599,627,370,496 +/-

short - range: 400-500 nm; peak range: 420-440 nm
medium - range: 450-630 nm; peak range: 534-555 nm
long - range: 500-700 nm; peak range: 564-580 nm
*/
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
  static constexpr T ENERGY_START_FOOD = 100000;
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
  static constexpr T AGENT_FORWARD_SPEED_MAX = 10;
  static constexpr T AGENT_ROTATIONAL_SPEED_MAX = 0.8; // approximately std::numbers::pi / 4.0
  static constexpr T UNIT_MIN_DELTA = std::nextafter(static_cast<T>(1), std::numeric_limits<T>::max());
  static constexpr T UNIT_MAX_WITH_DELTA = 1 + UNIT_MIN_DELTA;

  static constexpr T VIEW_DISTANCE = 500 * SPACE_UNIT;
  static constexpr T INTENSITY_MAX = 999 * SPACE_UNIT;
  static constexpr T INTENSITY_MAX_INV_NEG = -1 / INTENSITY_MAX;

  static constexpr size_t VIEW_RAYS = 1024;
  static constexpr float VIEW_RAYS_ANGLE_RANGE = std::numbers::pi / 4.0; // 45deg
  static constexpr float VIEW_RAYS_ANGLE_DELTA = VIEW_RAYS_ANGLE_RANGE / VIEW_RAYS;
  // Start negative side up by 1 delta to account for an even number of rays as
  // we want to ensure the forward direction is represented.
  static constexpr float VIEW_RAYS_ANGLE_START = -(VIEW_RAYS_ANGLE_RANGE / 2) + VIEW_RAYS_ANGLE_DELTA;
};


template <SimType T> class Sim;
template <SimType T> struct Position { T x, y; };
template <SimType T>
struct alignas(64) AgentBody_t {
  // Body details
  T pos_x{0};
  T pos_y{0};
  T dir_x{0};
  T dir_y{0};
  T radius{0};
  T energy{0};
  T forward_speed{0};
  T rotational_speed{0};
  std::array<T, 8> dummy{0};

  // Senses
  alignas(64) std::array<T, Config<T>::VIEW_RAYS> agent_view{0};
  // partial results used during calculations.
  alignas(64) std::array<T, Config<T>::VIEW_RAYS> rays_dx{0};
  alignas(64) std::array<T, Config<T>::VIEW_RAYS> rays_dy{0};
  alignas(64) std::array<T, Config<T>::VIEW_RAYS> working_agent_view{0};

  // Controls
  jms::sim::controls::Linear<T, T> forward_speed_control{forward_speed, 0, Config<T>::AGENT_FORWARD_SPEED_MAX, 1, 0, 0.25};
  jms::sim::controls::Linear<T, T> rotation_speed_control{rotational_speed, -Config<T>::AGENT_ROTATIONAL_SPEED_MAX, Config<T>::AGENT_ROTATIONAL_SPEED_MAX, 1, 0, 0.1};
  std::vector<jms::sim::controls::Interface<T, T>*> controls{&forward_speed_control, &rotation_speed_control};
};


template <SimType T>
class Sim {
private:
  // Based on counterclockwise rotation sweeping from left to right.
  static const std::vector<jms::math::angles::Values<T, jms::math::angles::Sin, jms::math::angles::Cos>> trig_vals;

  // Food
  alignas(64) std::array<T, Config<T>::NUM_FOOD_PER_AGENT> food_pos_x;
  alignas(64) std::array<T, Config<T>::NUM_FOOD_PER_AGENT> food_pos_y;
  alignas(64) std::array<T, Config<T>::NUM_FOOD_PER_AGENT> food_dir_x;
  alignas(64) std::array<T, Config<T>::NUM_FOOD_PER_AGENT> food_dir_y;
  alignas(64) std::array<T, Config<T>::NUM_FOOD_PER_AGENT> food_radius;
  alignas(64) std::array<T, Config<T>::NUM_FOOD_PER_AGENT> food_energy;
  alignas(64) std::array<T, Config<T>::NUM_FOOD_PER_AGENT> food_speed;
  // partial results used during calculations.
  alignas(64) std::array<T, Config<T>::NUM_FOOD_PER_AGENT> collision_check;

  // Agent
  alignas(64) std::array<AgentBody_t<T>, 1> agents;

  // each simulation owns its own generator
  std::mt19937_64 rng;

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
  std::function<std::optional<Position<T>>(void)> AgentPos(void) const noexcept {
    return [this, i=0](void) mutable -> std::optional<Position<T>> {
        if (i >= agents.size()) { return std::nullopt; } else { i++; }
        return Position<T>{.x=agents[i-1].pos_x, .y=agents[i-1].pos_y};
    };
  }
  std::function<std::optional<Position<T>>(void)> FoodPos(void) const noexcept {
    return [this, i=0](void) mutable -> std::optional<Position<T>> {
        if (i >= food_pos_x.size()) { return std::nullopt; } else { i++; }
        return Position<T>{.x=food_pos_x[i-1], .y=food_pos_y[i-1]};
    };
  }
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
const std::vector<jms::math::angles::Values<T, jms::math::angles::Sin, jms::math::angles::Cos>> Sim<T>::trig_vals{
    jms::math::angles::Precompute<T, jms::math::angles::Sin, jms::math::angles::Cos>(Config<T>::VIEW_RAYS, Config<T>::VIEW_RAYS_ANGLE_START, Config<T>::VIEW_RAYS_ANGLE_DELTA)};


template <SimType T>
std::unique_ptr<Sim<T>> Sim<T>::Create(std::optional<int32_t> opt_seed,
                                       jms::utils::random_helper::optional_seed_seq_input_t opt_seed_seq_vec,
                                       std::optional<uint32_t> opt_discard) {
  std::unique_ptr<Sim<T>> sim{new Sim<T>{}};
  jms::utils::random_helper::Initialize(sim->rng, opt_seed, opt_seed_seq_vec, opt_discard);
  sim->InitializeAgent();
  for (size_t index=0; index<Config<T>::NUM_FOOD_PER_AGENT; ++index) { sim->InitializeFood(index); }
  std::fill_n(sim->agents[0].agent_view.begin(), Config<T>::VIEW_RAYS, 0);
  return sim;
}


template <SimType T>
void Sim<T>::InitializeAgent(size_t index) noexcept {
  T dx = GenUnit();
  T dy = GenUnit();
  T dlen = std::sqrt(std::fma(dx, dx, dy * dy));
  agents[index].pos_x = 0;
  agents[index].pos_y = 0;
  if (dlen == 0) {
    agents[index].dir_x = 0;
    agents[index].dir_y = 1;
  } else {
    agents[index].dir_x = dx / dlen;
    agents[index].dir_y = dy / dlen;
  }
  agents[index].radius = Config<T>::RADIUS_AGENT;
  agents[index].energy = Config<T>::ENERGY_START_AGENT;
  agents[index].forward_speed = 0;
  agents[index].rotational_speed = 0;
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
  T radius = GenRadius();
  // place relative to the agent
  if (plen == 0) {
    food_pos_x[index] = agents[0].pos_x;
    food_pos_y[index] = spawn_distance + agents[0].pos_y;
  } else {
    food_pos_x[index] = std::fma(px / plen, spawn_distance, agents[0].pos_x);
    food_pos_y[index] = std::fma(py / plen, spawn_distance, agents[0].pos_y);
  }
  if (dlen == 0) {
    food_dir_x[index] = 0;
    food_dir_y[index] = 1;
  } else {
    food_dir_x[index] = dx / dlen;
    food_dir_y[index] = dy / dlen;
  }
  food_radius[index] = radius;
  food_energy[index] = Config<T>::ENERGY_START_FOOD;
  food_speed[index] = Config<T>::SPEED_FOOD_MOVE;
  return;
}


template <SimType T>
void Sim<T>::Step(void) noexcept {
  /***
   * Process-
   *   for each agent
   *     1. generate view
   *     2. make decisions
   *     3. take action
   *     for each food
   *       1. Check for possible intersections
   *       2. For each possible collision, check for agent/food collision and food out of energy (create new)
   *   4. Move food
   *
   * TODO:
   *   1. Share energy if multiple agents collide with the same food.
   *   2. Move food out of energy into food movement?
   *   3. Agent/agent + food/food collision? Reflection?
   *   4. Remove agent/food and introduce type; type same/same (noop), A/B (consume) fidelity of type? who consumes who (based of direction; conflict resolution)?
   */
  for (size_t agent_index=0; agent_index<agents.size(); ++agent_index) {
    // Load agent info.
    T px_a = agents[agent_index].pos_x;
    T py_a = agents[agent_index].pos_y;
    T dx_a = agents[agent_index].dir_x;
    T dy_a = agents[agent_index].dir_y;
    T r_a = agents[agent_index].radius;
    T energy_a = agents[agent_index].energy;
    T speed_a = agents[agent_index].forward_speed;

    auto& rays_dx = agents[agent_index].rays_dx;
    auto& rays_dy = agents[agent_index].rays_dy;
    auto& working_agent_view = agents[agent_index].working_agent_view;
    for (size_t index=0; index<Config<T>::VIEW_RAYS; ++index) {
      T cos_t = trig_vals[index].cos;
      T sin_t = trig_vals[index].sin;
      rays_dx[index] = std::fma(dx_a, cos_t, -dy_a * sin_t);
      rays_dy[index] = std::fma(dx_a, sin_t, dy_a * cos_t);
    }
    std::fill_n(working_agent_view.begin(), Config<T>::VIEW_RAYS, std::numeric_limits<T>::max());

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
      T rays_dx_r = rays_dx[Config<T>::VIEW_RAYS - 1];
      T rays_dy_r = rays_dy[Config<T>::VIEW_RAYS - 1];
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

      for (size_t ray_index; ray_index<Config<T>::VIEW_RAYS; ++ray_index) {
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

    for (size_t index; index<Config<T>::VIEW_RAYS; ++index) {
      agents[agent_index].agent_view[index] = LuminescenceValue(working_agent_view[index]);
    }

    // Make decision

    // Update agent
    if (speed_a > 0) {
      px_a = std::fma(speed_a, dx_a, px_a);
      py_a = std::fma(speed_a, dy_a, py_a);
      agents[agent_index].pos_x = px_a;
      agents[agent_index].pos_y = py_a;
    }
    agents[agent_index].energy = energy_a;

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
  }

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

  return;
}


} // namespace Circles2D_f
} // namespace sim
} // namespace jms


#endif // JMS_SIM_CIRCLES2D_H
