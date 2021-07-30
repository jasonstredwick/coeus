#include <algorithm>
#include <chrono>
#include <memory>
#include <pthread.h>
#include <string>
#include <thread>
#include <vector>

#include "fmt/core.h"
//#include "taskflow/taskflow.hpp"

#include "jms/sim/circles2d.h"
#include "jms/utils/barrier.h" // until <barrier> is available; gcc-11
#include "jms/utils/viz/sdl2_renderer.h"
#include "jms/utils/viz/color.h"
#include "jms/utils/viz/point_render.h"


//namespace taskflow = tf;


constexpr size_t BUFFER_DIM_Y = 1024;
constexpr size_t BUFFER_DIM_X = 1024;
constexpr size_t DEFAULT_AGENTS = 32;
constexpr size_t DEFAULT_STEPS = 1024;
constexpr size_t MAX_AGENTS = 1'000'000;
constexpr size_t MAX_STEPS = 1'000'000;


struct ArgResults {
  bool success {true};
  std::string msg {};
  bool use_double = false;
  bool use_threads = false;
  size_t num_agents {DEFAULT_AGENTS};
  size_t num_steps {DEFAULT_STEPS};
  bool show_help {false};
  bool use_viz {false};
};


template <typename T>
struct VizInfo {
  jms::utils::viz::SDL2Renderer renderer;
  jms::utils::viz::PointRender<T> point_render;
};


template <typename T>
std::optional<VizInfo<T>> CreateVizInfo(bool use_viz, T spawn_radius) {
  std::optional<VizInfo<T>> viz_info;
  if (use_viz) {
    viz_info = VizInfo<T> {
      .renderer=jms::utils::viz::SDL2Renderer {BUFFER_DIM_X, BUFFER_DIM_Y, jms::utils::viz::SDL2Renderer::Options {.title="Sim2DCircle", .render_draw_color=jms::utils::viz::COLOR_BLACK}},
      .point_render=jms::utils::viz::PointRender<T> {BUFFER_DIM_X, BUFFER_DIM_Y, jms::utils::viz::COLOR_BLACK, jms::utils::viz::COLOR_WHITE}
    };
    viz_info.value().point_render.ChangeTransformers(-spawn_radius, spawn_radius, -spawn_radius, spawn_radius);
  }
  return viz_info;
}


void Draw(auto& renderer, auto& point_render, auto& sim) {
  point_render.Clear();
  point_render.SetDrawColor(jms::utils::viz::COLOR_WHITE);
  auto fiter = sim.AgentPos();
  for (std::optional<auto> opt_pos=fiter(); opt_pos.has_value(); opt_pos=fiter()) {
    auto pos = opt_pos.value();
    point_render.DrawPoint(pos.x, pos.y);
  }
  point_render.SetDrawColor(jms::utils::viz::COLOR_RED);
  auto aiter = sim.FoodPos();
  for (std::optional<auto> opt_pos=aiter(); opt_pos.has_value(); opt_pos=aiter()) {
    auto pos = opt_pos.value();
    point_render.DrawPoint(pos.x, pos.y);
  }
  renderer.Draw(point_render.Data());
  return;
}


template <typename T>
auto GenerateSims(size_t num_agents) {
  alignas(64) std::vector<std::unique_ptr<jms::sim::Circles2D::Sim<T>>> sims;
  for (size_t i=0; i<num_agents; ++i) {
    sims.emplace_back(jms::sim::Circles2D::Sim<T>::Create(static_cast<int32_t>(i+100)));
  }
  return sims;
}


void Help(std::optional<std::string> error_msg=std::nullopt) {
  fmt::print("scratch_sim2d_circle [args]\n");
  if (error_msg) { fmt::print("{}\n", error_msg.value()); }
  fmt::print("\n");
  fmt::print("    --use-double    Run simulation using double instead of float.; default=false\n");
  fmt::print("    --use-threads   Use threads; default=false\n");
  fmt::print("    --num-agents    Number of agents; default={}\n", DEFAULT_AGENTS);
  fmt::print("    --num-steps     Number of steps; default={}\n", DEFAULT_STEPS);
  fmt::print("    --viz           Display visualization\n");
  return;
}


template <typename T>
void Process(size_t num_agents, size_t num_steps, size_t num_threads, bool use_viz) {
  std::chrono::high_resolution_clock clock{};
  double total = 0.0;
  double min = 1000000000000.0;
  double max = -1.0;
  double count = 0;

  if (num_threads <= 1) {
    // run with no threading.
    auto sims = GenerateSims<T>(num_agents);
    auto viz_info = CreateVizInfo(use_viz, jms::sim::Circles2D::Config<T>::SPAWN_RADIUS_MAX);
    if (viz_info.has_value() && !viz_info.value().renderer.IsReady()) {
      fmt::print("Failed to create window; window not ready!\n");
      return;
    }
    auto t = clock.now();
    for (size_t i=0; i<num_steps; ++i) {
      auto t1 = clock.now();
      for (auto& sim : sims) { sim->Step(); }
      auto dt1 = clock.now() - t1;
      double val = std::chrono::duration_cast<std::chrono::microseconds>(dt1).count();
      min = std::min(min, val);
      max = std::max(max, val);
      if (val > 1000) { count = count + 1; }
      if (viz_info.has_value()) {
        Draw(viz_info.value().renderer, viz_info.value().point_render, *sims[0]);
        if (!viz_info.value().renderer.ProcessEventsOrQuit()) { break; }
      }
    }
    auto dt = clock.now() - t;
    total = std::chrono::duration_cast<std::chrono::microseconds>(dt).count();
  } else {
    // use threads.
    size_t unit_size = num_agents / num_threads; // extra accounted later
    size_t num_units = std::min(num_agents, num_threads);
    size_t extra = num_agents % num_threads;
    fmt::print("Num units: {}\n", num_units);
    fmt::print("Unit size: {} ; extra: {}\n", unit_size, extra);
    alignas(64) auto sims = GenerateSims<T>(num_agents);
    num_steps += 1; // for stats since first step is ignored timing-wise to reduce noise due to setup.

    bool first = true;
    auto t1 = clock.now();
    auto OnComplete = [&first, &t1, &min, &max, &count, &clock]() noexcept {
      if (first) { first = false; t1 = clock.now(); return; }
      auto dt1 = clock.now() - t1;
      double val = std::chrono::duration_cast<std::chrono::microseconds>(dt1).count();
      min = std::min(min, val);
      max = std::max(max, val);
      if (val > 1000) { count = count + 1; }
      t1 = clock.now();
      return;
    };
    jms::utils::barrier sync_point(num_threads, OnComplete);
    auto Work = [&sims, &sync_point](size_t start, size_t end, size_t num_steps) {
      for (size_t step=0; step<num_steps; ++step) {
        for (size_t i=start; i<end; ++i) { sims[start]->Step(); }
        sync_point.arrive_and_wait();
      }
    };

    std::vector<std::thread> threads;
    size_t start = 0;
    size_t end = start + unit_size + (extra ? 1 : 0);
    for (size_t unit_index=0; unit_index<num_units; ++unit_index) {
      if (end > sims.size()) { end = sims.size(); }
      threads.push_back(std::thread{Work, start, end, num_steps});
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      CPU_SET(unit_index, &cpuset);
      int rc = pthread_setaffinity_np(threads[unit_index].native_handle(), sizeof(cpu_set_t), &cpuset);
      if (rc != 0) {
        fmt::print("ERROR: Failed to set affinitiy.\n");
      }
      start = end;
      end = start + unit_size + (unit_index + 1 < extra ? 1 : 0);
    }
    auto t = clock.now();
    for (auto& thread : threads) { thread.join(); }
    auto dt = clock.now() - t;
    total = std::chrono::duration_cast<std::chrono::microseconds>(dt).count();

    num_steps -= 1; // see above; reduced for stats purpose.
  }
  fmt::print("Total time: {}  {}\n", total, total / 1000000.0);
  fmt::print("Avg time: {}\n", (total / num_steps) / 1000000.0);
  fmt::print("Min time: {}  {}\n", min, min / 1000000.0);
  fmt::print("Max time: {}  {}\n", max, max / 1000000.0);
  fmt::print("Count: {}\n", count);

  return;
}


ArgResults ProcessArgs(int argc, char** argv) {
  ArgResults results;
  try {
    for (size_t index=1; index<argc; ++index) {
      std::string arg {argv[index]};
      if (arg == "--help") {
        results.show_help = true;
        break;
      } else if (!arg.compare("--viz")) {
        results.use_viz = true;
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
  bool use_viz = args.use_viz;
  size_t num_agents = args.num_agents;
  size_t num_steps = args.num_steps;
  size_t num_threads = args.use_threads ? static_cast<size_t>(std::thread::hardware_concurrency()) - 2 : 1; // -2 for master thread
  fmt::print("RUN: {} {} {}\n", num_agents, num_threads, num_steps);
  if (num_agents < 0 || num_steps < 0) {
    return 0;
  }
  if (use_double) {
    Process<double>(num_agents, num_steps, num_threads, use_viz);
  } else {
    Process<float>(num_agents, num_steps, num_threads, use_viz);
  }
  return 0;
}
