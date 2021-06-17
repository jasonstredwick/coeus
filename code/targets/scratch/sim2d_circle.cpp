#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "fmt/core.h"
#include "taskflow/taskflow.hpp"
#include "SDL2/SDL.h"

#include "jms/sim/sim2d_circles_d.h"
#include "jms/sim/sim2d_circles_f.h"
#include "jms/utils/viz/sdl2_renderer.h"
#include "jms/utils/viz/color.h"
#include "jms/utils/viz/point_render.h"


namespace taskflow = tf;


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


/*


  jms::utils::viz::SDL2Renderer renderer {SCREEN_WIDTH, SCREEN_HEIGHT, SDL2Renderer::Options {.render_draw_color=SDL2Renderer::Color {.red=0xff, .green=0xff, .blue=0xff}}};
  while (renderer.ProcessEventsOrQuit()) {
    int result = renderer.Draw(img);
    if (result) {
      std::cout << "Failed to render image: " << result << std::endl;
      break;
    }
  }
*/

void Draw(auto& renderer, auto& point_render, auto& sim) {
  point_render.Clear();
  auto pi = sim.GetPositionInfoAccess();
  point_render.SetDrawColor(jms::utils::viz::COLOR_WHITE);
  for (std::optional<auto> food = pi.NextFood(); food.has_value(); food = pi.NextFood()) {
    auto pos = food.value();
    point_render.DrawPoint(pos.x, pos.y);
  }
  auto agent_pos = pi.Agent();
  point_render.SetDrawColor(jms::utils::viz::COLOR_RED);
  point_render.DrawPoint(agent_pos.x, agent_pos.y);
  renderer.Draw(point_render.Data());
  return;
}


void Process(auto& sims, size_t num_agents, size_t num_steps, size_t num_threads, std::optional<auto> viz_info) {
  std::chrono::high_resolution_clock clock{};
  double total = 0.0;

  if (num_threads <= 1) {
    // run with no threading.
    auto t = clock.now();
    for (size_t i : std::ranges::iota_view {static_cast<size_t>(0), num_steps}) {
      auto t1 = clock.now();
      for (auto& sim : sims) {
        sim->Step();
      }
      auto t2 = clock.now();
      if (viz_info.has_value()) { Draw(viz_info.value().renderer, viz_info.value().point_render, *sims[0]); }
      auto t3 = clock.now();
      //fmt::print("{}  --  {}\n", std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count(), std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count());
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
    taskflow::Executor executor(num_units);
    taskflow::Taskflow taskflow;
    size_t start = 0;
    size_t end = start + unit_size + (extra ? 1 : 0);
    for (size_t unit_index : std::ranges::iota_view {static_cast<size_t>(0), num_units}) {
      if (end > sims.size()) { end = sims.size(); }
      taskflow.emplace([&sims, start, end]() {
        for (size_t i : std::ranges::iota_view {start, end}) {
          sims[i]->Step();
        }
      });
      start = end;
      end = start + unit_size + (unit_index + 1 < extra ? 1 : 0);
    }
    auto t = clock.now();
    for (size_t step : std::ranges::iota_view {static_cast<size_t>(0), num_steps}) {
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
    std::optional<VizInfo<double>> viz_info;
    if (use_viz) {
      viz_info = VizInfo<double> {
        .renderer=jms::utils::viz::SDL2Renderer {BUFFER_DIM_X, BUFFER_DIM_Y, jms::utils::viz::SDL2Renderer::Options {.title="Sim2DCircle", .render_draw_color=jms::utils::viz::COLOR_BLACK}},
        .point_render=jms::utils::viz::PointRender<double> {BUFFER_DIM_X, BUFFER_DIM_Y, jms::utils::viz::COLOR_BLACK, jms::utils::viz::COLOR_WHITE}
      };
      double sr = jms::sim::Circles2D_d::SPAWN_RADIUS_MAX;
      viz_info.value().point_render.ChangeTransformers(-sr, sr, -sr, sr);
    }
    std::vector<std::unique_ptr<jms::sim::Circles2D_d::Sim>> sims;
    for (size_t i : std::ranges::iota_view {static_cast<size_t>(0), num_agents}) {
      sims.emplace_back(jms::sim::Circles2D_d::Sim::Create());
    }
    Process(sims, num_agents, num_steps, num_threads, std::move(viz_info));
  } else {
    std::optional<VizInfo<float>> viz_info;
    if (use_viz) {
      viz_info = VizInfo<float> {
        .renderer=jms::utils::viz::SDL2Renderer {BUFFER_DIM_X, BUFFER_DIM_Y, jms::utils::viz::SDL2Renderer::Options {.title="Sim2DCircle", .render_draw_color=jms::utils::viz::COLOR_BLACK}},
        .point_render=jms::utils::viz::PointRender<float> {BUFFER_DIM_X, BUFFER_DIM_Y, jms::utils::viz::COLOR_BLACK, jms::utils::viz::COLOR_WHITE}
      };
      float sr = jms::sim::Circles2D_f::SPAWN_RADIUS_MAX;
      viz_info.value().point_render.ChangeTransformers(-sr, sr, -sr, sr);
    }
    std::vector<std::unique_ptr<jms::sim::Circles2D_f::Sim>> sims;
    for (size_t i : std::ranges::iota_view {static_cast<size_t>(0), num_agents}) {
      sims.emplace_back(jms::sim::Circles2D_f::Sim::Create());
    }
    Process(sims, num_agents, num_steps, num_threads, std::move(viz_info));
  }
  return 0;
}
