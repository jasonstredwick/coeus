#include <algorithm>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "fmt/core.h"
#include "taskflow/taskflow.hpp"

#include "jms/sim/sim2d_circles.h"


namespace taskflow = tf;


int main(void) {
  fmt::print("start\n");

  size_t num_agents = 1000;
  size_t num_steps = 1024;
  size_t num_threads = static_cast<size_t>(std::thread::hardware_concurrency());
  fmt::print("RUN: {} {} {}\n", num_agents, num_threads, num_steps);

  if (num_agents > 0 && num_steps > 0) {
    std::chrono::high_resolution_clock clock{};
    double total = 0.0;

    std::vector<std::unique_ptr<jms::sim::Interface>> sims;
    for (size_t i=0; i<num_agents; ++i) {
      std::unique_ptr<jms::sim::Interface> sim = jms::sim::CreateSim2DCircles();
      sims.push_back(std::move(sim));
    }

    if (num_threads <= 1) {
      // run with no threading.
      for (size_t i=0; i<num_steps; ++i) {
        auto t = clock.now();
        for (size_t sim_index=0; sim_index<num_agents; ++sim_index) {
          sims[sim_index]->Step();
        }
        auto dt = clock.now() - t;
        auto dt_count = std::chrono::duration_cast<std::chrono::microseconds>(dt).count();
        total += dt_count;
      }
    } else {
      // use threads.
      size_t unit_size = num_agents / num_threads; // extra accounted later
      size_t num_units = std::min(num_agents, num_threads);
      size_t extra = num_agents % num_threads;
      fmt::print("Num units: {}\n", num_units);
      taskflow::Executor executor(num_units);
      taskflow::Taskflow taskflow;
      size_t start = 0;
      size_t end = start + unit_size + (extra ? 1 : 0);
      for (size_t unit_index=0; unit_index<num_units; ++unit_index) {
        if (end > sims.size()) { end = sims.size(); }
        taskflow.emplace([&sims, start, end]() {
          for (size_t i=start; i<end; ++i) {
            sims[i]->Step();
          }
        });
        start = end;
        end = start + unit_size + (unit_index + 1 < extra ? 1 : 0);
      }
      for (size_t step=0; step<num_steps; ++step) {
        auto t = clock.now();
        executor.run(taskflow);
        executor.wait_for_all();
        auto dt = clock.now() - t;
        auto dt_count = std::chrono::duration_cast<std::chrono::microseconds>(dt).count();
        total += dt_count;
      }
    }
    fmt::print("Total time: {}\n", total);
    fmt::print("Avg time: {}\n", (total / num_steps) / 1000000.0);
  }

  fmt::print("end\n");
  return 0;
}
