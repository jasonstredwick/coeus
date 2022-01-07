#ifndef JMS_AGENT_NODES_NODE_H
#define JMS_AGENT_NODES_NODE_H


#include "jms/agent/base.h"


namespace jms {
namespace agent {
namespace nodes {


template <typename T>
class Node {
private:
  size_t input_dim;
  size_t output_dim;

protected:
  Node(const size_t input_dim, const size_t output_dim) noexcept : input_dim(input_dim), output_dim(output_dim) { return; }
  Node(const Node&) noexcept = default;
  Node(Node&&) noexcept = default;
  virtual ~Node(void) noexcept { return; }
  Node& operator=(const Node&) noexcept = default;
  Node& operator=(Node&&) noexcept = default;

public:
  const size_t InputDim(void) const noexcept { return input_dim; }
  const size_t OutputDim(void) const noexcept { return output_dim; }
  [[nodiscard]] virtual std::unique_ptr<Node<T>> Copy(void) const = 0;
  [[nodiscard]] std::unique_ptr<Node<T>> CopyMutated(ThresholdCallable& PointMutation,
                                                     MutateCallable& InsertMutation,
                                                     MutateCallable& DeleteMutation) const = 0;
  virtual const Array_t<T>& Step(const Array_t<T>&) = 0;
  virtual const Array_t<T>& Result(void) const noexcept = 0;
};


} // namespace nodes
} // namespace agent
} // namespace jms


#endif // JMS_AGENT_NODES_NODE_H
