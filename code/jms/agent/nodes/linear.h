#ifndef JMS_AGENT_NODE_H
#define JMS_AGENT_NODE_H


#include <algorithm>
#include <numeric>
#include <vector>

#include "jms/agent/base.h"
#include "jms/agent/nodes/node.h"


namespace jms {
namespace agent {
namespace nodes {


template <typename T>
class Linear : public Node<T> {
private:
  Array_t<T> weights;
  Array_t<T> bias;
  Array_t<T> result;

public:
  Linear(const size_t input_dim, const size_t output_dim) noexcept : Node<T>{input_dim, output_dim}, weights{input_dim * output_dim}, bias{output_dim}, result{output_dim} { return; }
  Linear(const Array_t<T>& weights, const Array_t<T>& bias, const Array_t<T>& result) noexcept
  : Node<T>{bias.size() ? weights.size() / bias.size() : 0, bias.size()}, weights(weights), bias(bias), result{result}
  { return; }
  Linear(const Linear&) noexcept = default;
  Linear(Linear&&) noexcept = default;
  virtual ~Linear(void) noexcept { return; }
  Linear& operator=(const Linear&) noexcept = default;
  Linear& operator=(Linear&&) noexcept = default;

  void Fill(T w) noexcept {
    for (size_t i=0; i<output_dim; ++i) { std::ranges::fill(weights[i * input_dim], t); }
    return;
  }

  void Fill(T w, T b) noexcept {
    for (size_t i=0; i<output_dim; ++i) { std::ranges::fill(weights[i * input_dim], t); }
    std::ranges::fill(bias, b);
    return;
  }

  void Generate(auto WeighGen) {
    for (size_t i=0; i<output_dim; ++i) { std::ranges::generate(weights[i * input_dim], WeightGen); }
    return;
  }

  void Generate(auto WeighGen, auto BiasGen) {
    for (size_t i=0; i<output_dim; ++i) { std::ranges::generate(weights[i * input_dim], WeightGen); }
    std::ranges::generate(bias, BiasGen);
    return;
  }

  [[nodiscard]] std::unique_ptr<Node<T>> Copy(void) const override {
    return std::unique_ptr<Node<T>>{new Linear<T>{weights, bias, result}};
  }

  [[nodiscard]] std::unique_ptr<Node<T>> CopyMutated(ThresholdCallable& PointMutation,
                                                     MutateCallable& InsertMutation,
                                                     MutateCallable& DeleteMutation) const override {
    std::vector<size_t> inputs;
    inputs.reserve(input_dim);
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
    // check for size issues. or mark assumption about size
    for (size_t i=0; i<output_dim; ++i) {
      result[i] = std::inner_product(input.begin(), input.end(), weights[i * input_dim], static_cast<T>(0)) + bias[i];
    }
    return result;
  }

  const Array_t<T>& Result(void) const noexcept override { return result; }
};


} // namespace nodes
} // namespace agent
} // namespace jms


#endif // JMS_AGENT_NODE_H
