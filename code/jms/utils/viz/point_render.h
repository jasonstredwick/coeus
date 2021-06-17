#ifndef JMS_UTILS_VIZ_POINT_RENDER_H
#define JMS_UTILS_VIZ_POINT_RENDER_H


#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <optional>
#include <vector>

#include "jms/utils/viz/color.h"


namespace jms {
namespace utils {
namespace viz {


template <typename T>
class PointRender {
private:
  size_t width;
  size_t height;
  Color clear_color;
  Color draw_color;
  std::function<size_t (T)> TransformX;
  std::function<size_t (T)> TransformY;
  std::vector<std::vector<Color>> data;

public:
  PointRender(size_t width, size_t height, std::optional<Color> opt_clear_color=std::nullopt, std::optional<Color> opt_draw_color=std::nullopt)
  : width(width),
    height(height),
    clear_color(opt_clear_color.value_or(COLOR_BLACK)),
    draw_color(opt_draw_color.value_or(COLOR_WHITE)),
    TransformX(CreateTransformer(static_cast<T>(0), static_cast<T>((width > 0) ? width - 1 : 0), width)),
    TransformY(CreateTransformer(static_cast<T>(0), static_cast<T>((height > 0) ? height - 1 : 0), height)) {
    if (height) {
      data.resize(height);
      if (width) {
        for (size_t y=0; y<height; ++y) {
          data[y].resize(width, clear_color);
        }
      }
    }
    return;
  }
  PointRender(const PointRender&) = default;
  PointRender(PointRender&&) noexcept = default;
  ~PointRender(void) noexcept = default;
  PointRender& operator=(const PointRender&) = default;
  PointRender& operator=(PointRender&&) noexcept = default;

  // Default: maps 1:1 [0..width-1] and [0..height-1] for grid of widthxheight
  void ChangeTransformers(T view_min_x, T view_max_x, T view_min_y, T view_max_y) {
    TransformX = CreateTransformer(view_min_x, view_max_x, width);
    TransformY = CreateTransformer(view_min_y, view_max_y, height);
    return;
  }

  void Clear(std::optional<Color> opt_color=std::nullopt) {
    Color color = opt_color.value_or(clear_color);
    for (size_t i=0; i<data.size(); ++i) {
      std::fill(data[i].begin(), data[i].end(), color);
    }
    return;
  }

  const std::vector<std::vector<Color>>& Data(void) const { return data; }

  void DrawPoint(T x, T y) {
    size_t x_index = TransformX(x);
    size_t y_index = TransformY(y);
    if (x_index < width && y_index < height) {
      data[y_index][x_index] = draw_color;
    }
    return;
  }

  void SetClearColor(Color c) { clear_color = c; return; }

  void SetDrawColor(Color c) { draw_color = c; return; }

private:
  std::function<size_t (T)> CreateTransformer(T view_min, T view_max, size_t total_indices) {
    if (!total_indices) { return [](T) -> size_t { return 0; }; }
    if (view_min == view_max) { return [view_min, total_indices](T v) -> size_t { return (view_min == v) ? 0 : total_indices; }; }
    T v0 = std::min(view_min, view_max);
    T v1 = std::max(view_min, view_max);
    if (total_indices == 1) { return [v0, v1](T v) -> size_t { return (v >= v0 && v <= v1) ? 0 : 1; }; }
    // Shifted value is scaled to [0..1] by 1/(v1-v0) and mapped onto indices by multiplying by (total_indices - 1) [0..total_indices-1]
    // Will use double here as it can represent most ranges with good precision.  If more is needed then review this function to specialize based on T.
    double map_over_scale = static_cast<double>(total_indices - 1) / static_cast<double>(v1 - v0);
    return [v0, v1, map_over_scale, total_indices](T v) -> size_t {
      // Not knowing the type T (could be unsigned) test for inclusion before calculation.
      if (v < v0 || v > v1) { return total_indices; }
      return static_cast<size_t>(std::floor(static_cast<double>(v - v0) * map_over_scale));
    };
  }
};


} // namespace viz
} // namespace utils
} // namespace jms


#endif // JMS_UTILS_VIZ_POINT_RENDER_H
