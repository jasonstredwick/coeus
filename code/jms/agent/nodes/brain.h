#ifndef JMS_AGENT_BRAIN_H
#define JMS_AGENT_BRAIN_H


#include <limits>
#include <memory>
#include <random>

#include "jms/agent/base.h"
#include "jms/agent/nodes/linear.h"
#include "jms/agent/nodes/node.h"


namespace jms {
namespace agent {
namespace nodes {


template <typename T> concept SimType = std::is_floating_point_v<T>;


template <SimType T, typename Rng_t>
class alignas(64) Brain {
private:
  size_t input_dim;
  size_t output_dim;
  std::vector<std::unique_ptr<Node<T>>> nodes;
  Array_t<T> empty{0};
  Rng_t rng;

public:
  Brain(const size_t input_dim, const size_t output_dim, Rng_t& rng)
  : input_dim(input_dim), output_dim(output_dim), rng(rng) {
    std::normal_distribution<T> dist_bias(0, 0.05);
    std::normal_distribution<T> dist_weights(0, 0.05);
    std::unique_ptr<Linear<T>> node{new Linear<T>{3, 3}};
    node->Generate([&rng, &dist_weights]() { return dist_weights(rng); },
                   [&rng, &dist_bias]() { return dist_bias(rng); });
    nodes.push_back(std::move(node));
    std::unique_ptr<Linear<T>> node{new Linear<T>{3, output_dim}};
    node->Generate([&rng, &dist_weights]() { return dist_weights(rng); },
                   [&rng, &dist_bias]() { return dist_bias(rng); });
    nodes.push_back(std::move(node));
    return;
  }

  [[nodiscard]] std::unique_ptr<Brain> CopyMutated(ThresholdCallable& PointMutation,
                                                   MutateCallable& InsertMutation,
                                                   MutateCallable& DeleteMutation) const override {
    std::vector<size_t> layers;
    inputs.reserve(nodes.size());
    for (size_t i=0; i<input_dim; ++i) {
      if (InsertMutation()) { inputs.push_back(i); inputs.push_back(i); }
      else if (!DeleteMutation()) { inputs.push_back(i); }
    }
    std::vector<size_t> outputs;
    outputs.reserve(output_dim);
    for (size_t i=0; i<output_dim; ++i) {
      if (InsertMutation()) { outputs.push_back(i); outputs.push_back(i); }
      else if (!DeleteMutation()) { outputs.push_back(i); }
    }
    Array_t<T> new_weights{inputs.size() * outputs.size()};
    Array_t<T> new_bias{outputs.size()};
    Array_t<T> new_result{outputs.size()};
    for (size_t i=0, row=0; i<outputs.size(); ++i, ++row) {
      const size_t src_row_index = outputs[i] * inputs_dim;
      const size_t dst_row_index = row * inputs.size();
      for (size_t j=0, col=0; j<inputs.size(); ++j, ++col) {
        const size_t src_index = src_row_index + inputs[j];
        const size_t dst_index = dst_row_index + col;
        new_weights[dst_index] = PointMutation(weights[src_index]);
      }
      new_bias[dst_index] = PointMutation(bias[src_index]);
      new_result[dst_index] = result[src_index];
    }
    return std::unique_ptr<Node<T>>{new Linear{new_weights, new_bias, new_result}};
  }

  const Array_t<T>& Step(const Array_t<T>& input) override {
    if (input.size() != input_dim) { throw std::runtime_error("Brain provided incorrect input dimension."); }
    if (!nodes.size() && input.size() != output_dim) { throw std::runtime("Brain result has incorrect output dimension; input != output."); }
    const auto& result = input;
    for (size_t i=0; i<nodes.size(); ++i) {
      result = nodes[i].Step(result);
    }
    return result;
  }

  const Array_t<T>& Result(void) const noexcept override {
    return nodes.size() ? nodes[nodes.size() - 1].Result() : empty;
  }
};


} // namespace nodes
} // namespace agent
} // namespace jms


#endif // JMS_AGENT_BRAIN_H
