#ifndef JMS_SIM_CONTROLS_H
#define JMS_SIM_CONTROLS_H


#include <algorithm>
#include <cmath>
#include <limits>
#include <random>


namespace jms {
namespace sim {
namespace controls {


template <typename T> concept IntType = std::is_integral_v<T>;
template <typename T> concept RealType = std::is_floating_point_v<T>;


template <typename T, typename Value_t>
class Interface {
protected:
  T& ref;
  const Value_t min_v;
  const Value_t max_v;

  Interface(T& ref, const Value_t min_v_, const Value_t max_v_) noexcept
  : ref(ref), min_v(min_v_), max_v(max_v_ >= min_v_ ? max_v_ : min_v_)
  { return; }
  const T Set(const T t) noexcept { ref = t; return ref; }

public:
  virtual ~Interface(void) noexcept { return; }
  Value_t Rand(auto& rng) const noexcept {
    if constexpr (std::is_integral_v<Value_t>) {
      std::uniform_int_distribution<Value_t> dist(min_v, max_v);
      return dist(rng);
    } else {
      std::uniform_real_distribution<Value_t> dist(min_v, std::nextafter(max_v, std::numeric_limits<Value_t>::max()));
      return dist(rng);
    }
  };
  virtual const T Update(const Value_t v) noexcept { return this->Set(static_cast<T>(v)); }
  const T Value(void) const noexcept { return ref; };
};


class Toggle : public Interface<bool, uint8_t> {
public:
  Toggle(bool& ref, const bool default_v=false) noexcept
  : Interface<bool, uint8_t>{ref, 0, 1}
  { ref = default_v; return; }
  virtual ~Toggle(void) noexcept { return; }
};


template <typename Value_t>
class ToggleStep : public Interface<bool, Value_t> {
protected:
  const Value_t threshold;

public:
  ToggleStep(bool& ref, const Value_t min_v, const Value_t max_v, const Value_t threshold, const bool default_v=false) noexcept
  : Interface<bool, Value_t>{ref, min_v, max_v}, threshold(threshold)
  { this->Update(default_v); return; }
  virtual ~ToggleStep(void) noexcept { return; }
  const bool Update(const Value_t value) noexcept override { return this->Set(value >= threshold ? true : false); }
};


template <IntType T, typename Value_t>
class Dial : public Interface<T, Value_t> {
protected:
  Value_t tick_size;
  T num_ticks;

public:
  Dial(T& ref, const Value_t min_v, const Value_t max_v, const T num_ticks_, const Value_t default_v=0) noexcept
  : Interface<T, Value_t>{ref, min_v, max_v}, tick_size{0}, num_ticks{num_ticks_} {
    if (num_ticks <= 0) { num_ticks = 0; }
    else { tick_size = (this->max_v - this->min_v) / static_cast<Value_t>(num_ticks); }
    this->Update(default_v);
    return;
  }
  virtual ~Dial(void) noexcept { return; }
  const T Update(const Value_t value) noexcept override {
    if (!num_ticks || !tick_size) { return this->Set(0); }
    T tick = static_cast<T>(std::floor((value - this->min_v) / tick_size)) % num_ticks;
    if (tick < 0) { tick = num_ticks + tick; }
    return this->Set(tick);
  }
};


template <typename T, typename Value_t>
class Linear : public Interface<T, Value_t> {
protected:
  const Value_t slope;
  const Value_t intercept;
  const Value_t step_size;

public:
  Linear(T& ref, const Value_t slope=1, const Value_t intercept=0, const Value_t step_size=0, const Value_t default_v=0) noexcept
  : Interface<T, Value_t>{ref, std::numeric_limits<Value_t>::lowest(), std::numeric_limits<Value_t>::max()},
    slope(slope), intercept(intercept), step_size(step_size > 0 ? step_size : 0) {
    this->Update(default_v);
    return;
  }
  Linear(T& ref, const Value_t min_v, const Value_t max_v,
         const Value_t slope=1, const Value_t intercept=0, const Value_t step_size=0, const Value_t default_v=0) noexcept
  : Interface<T, Value_t>{ref, min_v, max_v}, slope(slope), intercept(intercept), step_size(step_size > 0 ? step_size : 0) {
    this->Update(default_v);
    return;
  }
  virtual ~Linear(void) noexcept { return; }
  const T Update(const Value_t value) noexcept override {
    Value_t x = std::fma(slope, value, intercept);
    if (step_size > 0) { x -= std::fmod(x, step_size); }
    x = std::clamp(x, this->min_v, this->max_v);
    return this->Set(static_cast<T>(x));
  }
};


} // namespace controls
} // namespace sim
} // namespace jms


#endif // JMS_SIM_CONTROLS_H
