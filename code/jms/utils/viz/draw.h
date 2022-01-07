#ifndef JMS_UTILS_VIZ_DRAW_H
#define JMS_UTILS_VIZ_DRAW_H


#include <iostream>
#include <iomanip>
#include <vector>

#include "jms/utils/viz/sdl2_renderer.h"
#include "jms/utils/viz/color.h"
#include "jms/utils/viz/point_render.h"


namespace jms {
namespace utils {
namespace viz {


void Draw(uint32_t rows, uint32_t cols, const std::vector<uint8_t>& data) {
  auto renderer = viz::SDL2Renderer{static_cast<int>(cols), static_cast<int>(rows), viz::SDL2Renderer::Options {.title="Image", .render_draw_color=viz::COLOR_BLACK}};
  auto point_render = viz::PointRender<uint32_t>{cols, rows, viz::COLOR_BLACK, viz::COLOR_WHITE};
  point_render.Clear();
  for (uint32_t row=0; row<rows; ++row) {
    for (uint32_t col=0; col<cols; ++col) {
      uint8_t value = data[row * cols + col];
      viz::Color c{value, value, value, 0xff};
      point_render.SetDrawColor(c);
      point_render.DrawPoint(col, row);
      std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)value << " ";
    }
    std::cout << std::endl;
  }
  renderer.Draw(point_render.Data());
  while (renderer.ProcessEventsOrQuit()) { usleep(1000); }
  return;
}


} // namespace viz
} // namespace utils
} // namespace jms


#endif // JMS_UTILS_VIZ_DRAW_H
