#ifndef JMS_AGENT_BASE_H
#define JMS_AGENT_BASE_H


#include <algorithm>
#include <charconv>
#include <numeric>
#include <stdexcept>
#include <vector>

#include "jms/base/array.h"


namespace jms {
namespace agent {


template <typename T>
using Array_t = jms::base::array::AlignedDynamic64_t<T>;

template <typename T>
concept BoolCallable = std::is_move_constructible_v<T> && std::is_destructible_v<T> && std::is_nothrow_invocable_v<T&> &&
                       requires(T F) { {F()} -> std::convertible_to<bool>; };









/***
Concept- Agent "brain" consists of an input, controls, and optional internal structures.  The brain processes
input and using the controls updates properties of the agent in the physical realm.  Energy is consumed by the
physical execution determined outside the brain where the control value is expressed.  The brain should also consume
energy during processing.  Each brain could determine the amount of energy but should probably be set at the simulation
level.  Brains would be allowed to end to end process input to single during one step or alternatively migrate
data through the brain one step at a time; i.e. the brain would have multiple inputs passing through at the same time
staggered by step.  That would be part of the design.

Brain structures-
1. Potential to have a decision queue for updating the controller over time.
2. The concept of Node could be used to form a state machine/graph.  A node could be connected to any other node
   including the inputs and controls (though they have to be terminals).  There may be some requirements, adjustments,
   or different application patterns for nodes that don't have the same input/output shapes.
   Evolution-
   a. Weights can be adjusted individually or addition of a random delta for all
   b. "dropout" would be setting a weight to zero.
   c. New nodes can be created (require immediate hook up?) Node matching process would be required to hook up two
      nodes as mentioned above.
   d. Nodes can be removed and mathcing process would happen between the connected nodes.
   e. Nodes can be duplicated.
   f. Nodes weights can be increased, copy doubled, etc
*/
/*
namespace layer {


template <typename T, size_t input_size, size_t output_size>
class FC_Linear {
private:
  template <typename U, size_t N> using Array_t = std::array<U, N>;//jms::base::array::AlignedDynamic<U, 64>;

public:
  using Value_t = T;
  using Input_t = Array_t<Value_t, input_size>
  using Result_t = Array_t<Value_t, output_size>;
  using Matrix_t = Array_t<Array_t<Value_t, input_size>, output_size>;
  static constexpr Value_t input_size_v = input_size;
  static constexpr Value_t output_size_v = output_size;

private:
  Matrix_t data{}; // defaults to zero initialized
  Array_t result{}; // defaults to zero initialized

public:
  constexpr FullyConnected(void) noexcept = default;
  constexpr FullyConnected(Value_t default_v) noexcept {
    for (size_t i=0; i<data.size(); ++i) { data[i].fill(default_v); }
    return;
  }
  constexpr FullyConnected(const Matrix_t& src_data) noexcept : data(src_data) { return; }
  constexpr const Result_t& Forward(const std::vector<& input) noexcept {
    return;
  }
}

}; // namespace layer


class Base {
protected:
  A(void) noexcept = default;
public:
  virtual ~A(void) noexcept { return; };
  virtual void Process(void) noexcept = 0;
};




template <typename Input_t, typename Control_t, typename Rng_t>
class Random : public Base {
private:
  const Input_t& inputs;
  Control_t& controls;
  Rng_t& rng;
public:
  Random(const Input_t& inputs, Control_t& controls, Rng_t& rng) noexcept
  : Base{}, inputs(inputs), controls(controls), rng(rng)
  { return; }
  virtual ~Random(void) noexcept { return; }

  void Process(void) noexcept override {
    for (size_t i=0; i<controls.size(); ++i) {
      controls[i].Update(controls[i].Rand(rng));
    }
    return;
  }
};
*/



} // namespace agent
} // namespace jms


#endif // JMS_AGENT_BASE_H
