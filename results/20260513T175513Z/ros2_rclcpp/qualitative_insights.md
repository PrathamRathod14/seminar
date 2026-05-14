# Qualitative Architecture Insights: ros2/rclcpp

The previous `architecture_metrics.jpg` is a metric dashboard, not an architecture diagram. The generated `my_node -> chatter` graph is also misleading for `rclcpp`: it comes from an example/template API in `rclcpp/include/rclcpp/node.hpp`, not from a real application node in this repository.

## Real architectural shape

`rclcpp` is the C++ client-library layer for ROS 2. Its architecture is package/API centered:

- `rclcpp` is the core public API: `Node`, `Context`, publishers, subscriptions, clients, services, timers, parameters, QoS, callback groups, executors, and wait sets.
- `rclcpp_action` extends the core API with action client/server support and depends on `rclcpp`, `rcl_action`, `rcl`, and `action_msgs`.
- `rclcpp_components` adds dynamic composition: component registration, loading, and container executables for multiple nodes in one process.
- `rclcpp_lifecycle` adds lifecycle-managed nodes and managed entities over `rclcpp` plus `rcl_lifecycle` and `lifecycle_msgs`.

## Key qualitative points

- The important boundary is not "node publishes topic"; it is "application code uses `rclcpp` APIs, and `rclcpp` maps those calls onto lower ROS 2 layers."
- The `Node` API is intentionally split behind node-interface classes such as base, topics, services, timers, parameters, graph, logging, and waitables. That keeps a large public facade while separating responsibilities internally.
- Executors, callback groups, and wait sets form the runtime coordination layer. They decide when callbacks execute and how concurrency rules are applied.
- `rclcpp_components` changes deployment architecture: multiple components can share one process and executor instead of each node needing its own process.
- Lifecycle support is an architectural specialization, not just another node. It introduces state transitions and managed publishers/entities, so lifecycle mistakes are a different class of error from ordinary topic mismatches.

## Research interpretation

For framework repositories like `rclcpp`, node/topic extraction alone under-represents the architecture. A good evaluation should include package dependencies, API layers, node-interface boundaries, executor/concurrency abstractions, and lifecycle/component extensions. Treating `rclcpp` as a runtime application graph creates false negatives and odd toy diagrams.

