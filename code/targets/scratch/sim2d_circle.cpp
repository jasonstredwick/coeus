#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "fmt/core.h"
#include "taskflow/taskflow.hpp"

#include "jms/sim/sim2d_circles_d.h"
#include "jms/sim/sim2d_circles_f.h"


namespace taskflow = tf;


constexpr int32_t DEFAULT_AGENTS = 32;
constexpr int32_t DEFAULT_STEPS = 1024;
constexpr int32_t MAX_AGENTS = 1'000'000;
constexpr int32_t MAX_STEPS = 1'000'000;


struct ArgResults {
  bool success {true};
  std::string msg {};
  bool use_double = false;
  bool use_threads = false;
  int32_t num_agents {DEFAULT_AGENTS};
  int32_t num_steps {DEFAULT_STEPS};
  bool show_help {false};
};


void Help(std::optional<std::string> error_msg=std::nullopt) {
  fmt::print("scratch_sim2d_circle [args]\n");
  if (error_msg) { fmt::print("{}\n", error_msg.value()); }
  fmt::print("\n");
  fmt::print("    --use-double    Run simulation using double instead of float.; default=false\n");
  fmt::print("    --use-threads   Use threads; default=false\n");
  fmt::print("    --num-agents    Number of agents; default={}\n", DEFAULT_AGENTS);
  fmt::print("    --num-steps     Number of steps; default={}\n", DEFAULT_STEPS);
  return;
}


void Process(auto& sims, int32_t num_agents, int32_t num_steps, int32_t num_threads) {
  std::chrono::high_resolution_clock clock{};
  double total = 0.0;

  if (num_threads <= 1) {
    // run with no threading.
    auto t = clock.now();
    for (int32_t i : std::ranges::iota_view{0, num_steps}) {
      auto t1 = clock.now();
      for (auto& sim : sims) {
        sim->Step();
      }
      auto dt1 = clock.now() - t1;
      fmt::print("{}\n", std::chrono::duration_cast<std::chrono::microseconds>(dt1).count());
    }
    auto dt = clock.now() - t;
    total = std::chrono::duration_cast<std::chrono::microseconds>(dt).count();
  } else {
    // use threads.
    int32_t unit_size = num_agents / num_threads; // extra accounted later
    int32_t num_units = std::min(num_agents, num_threads);
    int32_t extra = num_agents % num_threads;
    fmt::print("Num units: {}\n", num_units);
    fmt::print("Unit size: {} ; extra: {}\n", unit_size, extra);
    taskflow::Executor executor(num_units);
    taskflow::Taskflow taskflow;
    size_t start = 0;
    size_t end = start + unit_size + (extra ? 1 : 0);
    for (int32_t unit_index : std::ranges::iota_view{0, num_units}) {
      if (end > sims.size()) { end = sims.size(); }
      taskflow.emplace([&sims, start, end]() {
        for (int32_t i : std::ranges::iota_view{start, end}) {
          sims[i]->Step();
        }
      });
      start = end;
      end = start + unit_size + (unit_index + 1 < extra ? 1 : 0);
    }
    auto t = clock.now();
    for (int32_t step : std::ranges::iota_view{0, num_steps}) {
      auto t1 = clock.now();
      executor.run(taskflow);
      executor.wait_for_all();
      auto dt1 = clock.now() - t1;
      fmt::print("{}\n", std::chrono::duration_cast<std::chrono::microseconds>(dt1).count());
    }
    auto dt = clock.now() - t;
    total = std::chrono::duration_cast<std::chrono::microseconds>(dt).count();
  }
  fmt::print("Total time: {}\n", total);
  fmt::print("Avg time: {}\n", (total / num_steps) / 1000000.0);

  return;
}


ArgResults ProcessArgs(int argc, char** argv) {
  ArgResults results;
  try {
    for (int32_t index=1; index<argc; ++index) {
      std::string arg {argv[index]};
      if (arg == "--help") {
        results.show_help = true;
        break;
      } else if (!arg.compare("--use-double")) {
        results.use_double = true;
      } else if (!arg.compare("--use-threads")) {
        results.use_threads = true;
      } else if (!arg.compare("--num-agents")) {
        index += 1;
        try {
          results.num_agents = std::stoul(std::string{argv[index]});
          if (results.num_agents <= 0 || results.num_agents >= MAX_AGENTS) {
            results.num_agents = DEFAULT_AGENTS;
          }
        } catch (std::exception& e) {
          results.success = false;
          results.msg = "Invalid number of agents provided.";
          break;
        }
      } else if (!arg.compare("--num-steps")) {
        index += 1;
        try {
          results.num_steps = std::stoul(std::string{argv[index]});
          if (results.num_steps <= 0 || results.num_steps >= MAX_STEPS) {
            results.num_steps = DEFAULT_STEPS;
          }
        } catch (std::exception& e) {
          results.success = false;
          results.msg = "Invalid number of steps provided.";
          break;
        }
      } else {
        results.success = false;
        results.msg = "Invalid argument";
        break;
      }
    }
  } catch (std::exception& e) {
    results.success = false;
    std::string msg = "Unexpected error processing arguments.";
  }
  return results;
}


int main(int argc, char** argv) {
  auto args = ProcessArgs(argc, argv);
  if (!args.success) {
    Help(args.msg);
    return 1;
  } else if (args.show_help) {
    Help();
    return 0;
  }
  bool use_double = args.use_double;
  int32_t num_agents = args.num_agents;
  int32_t num_steps = args.num_steps;
  int32_t num_threads = args.use_threads ? static_cast<int32_t>(std::thread::hardware_concurrency()) - 2 : 1; // -2 for master thread
  fmt::print("RUN: {} {} {}\n", num_agents, num_threads, num_steps);
  if (num_agents < 0 || num_steps < 0) {
    return 0;
  }

  if (use_double) {
    std::vector<std::unique_ptr<jms::sim::Sim2DCircles_d>> sims;
    for (int32_t i : std::ranges::iota_view{0, num_agents}) {
      sims.emplace_back(jms::sim::Sim2DCircles_d::Create());
    }
    Process(sims, num_agents, num_steps, num_threads);
  } else {
    std::vector<std::unique_ptr<jms::sim::Sim2DCircles_f>> sims;
    for (int32_t i : std::ranges::iota_view{0, num_agents}) {
      sims.emplace_back(jms::sim::Sim2DCircles_f::Create());
    }
    Process(sims, num_agents, num_steps, num_threads);
  }
  return 0;
}
