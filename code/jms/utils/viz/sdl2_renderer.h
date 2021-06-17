#ifndef JMS_UTILS_VIZ_SDL2_RENDERER_h
#define JMS_UTILS_VIZ_SDL2_RENDERER_h


#include <cstring>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <SDL2/SDL.h>

#include "jms/utils/viz/color.h"


namespace jms {
namespace utils {
namespace viz {


class SDL2Renderer {
public:
  struct Options {
    std::optional<std::string> title;
    std::optional<int> pos_x;
    std::optional<int> pos_y;
    std::optional<uint32_t> flags;
    std::optional<Color> render_draw_color;
  };

private:
  std::unique_ptr<SDL_Window, decltype(&SDL_DestroyWindow)> window {nullptr, SDL_DestroyWindow};
  SDL_Surface* surface {nullptr};
  std::unique_ptr<SDL_Renderer, decltype(&SDL_DestroyRenderer)> renderer {nullptr, SDL_DestroyRenderer};
  std::unique_ptr<SDL_Texture, decltype(&SDL_DestroyTexture)> drawable {nullptr, SDL_DestroyTexture};
  Color render_draw_color;
  bool ready {false};

public:
  SDL2Renderer(int width, int height, Options&& options={})
  : render_draw_color(options.render_draw_color.value_or(Color {})) {
    window.reset(
      SDL_CreateWindow(options.title.value_or(std::string {}).c_str(),
                       options.pos_x.value_or(SDL_WINDOWPOS_UNDEFINED),
                       options.pos_y.value_or(SDL_WINDOWPOS_UNDEFINED),
                       width,
                       height,
                       options.flags.value_or(0))
    );
    if (!window) {
      std::cout << "Window could not be created! SDL_Error: " << SDL_GetError() << std::endl;
      return;
    }
    surface = SDL_GetWindowSurface(window.get());
    renderer.reset(SDL_CreateSoftwareRenderer(surface));
    if (!renderer) {
      std::cout << "SDL could not create a software renderer: " << SDL_GetError() << std::endl;
      return;
    }

    // clear rendering surface
    SDL_SetRenderDrawColor(renderer.get(), render_draw_color.red, render_draw_color.green, render_draw_color.blue, render_draw_color.alpha);
    SDL_RenderClear(renderer.get());

    // Create texture for drawing.
    drawable.reset(SDL_CreateTexture(renderer.get(), SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STREAMING, width, height));
    if (!drawable) {
      std::cout << "SDL could not create a drawable: " << SDL_GetError() << std::endl;
      return;
    }

    ready = true;

    return;
  }
  SDL2Renderer(const SDL2Renderer&) = delete;
  SDL2Renderer(SDL2Renderer&&) = default;
  ~SDL2Renderer(void) {
    // Need to reset pointers prior to SDL_Quit
    ready = false;
    drawable.reset(nullptr);
    surface = nullptr;
    renderer.reset(nullptr);
    window.reset(nullptr);
    SDL_Quit();
  };
  SDL2Renderer& operator=(const SDL2Renderer&) = delete;
  SDL2Renderer& operator=(SDL2Renderer&&) = default;

  int Draw(const std::vector<std::vector<Color>>& img) {
    if (!ready) { return 1; }
    SDL_Rect viewport;
    SDL_RenderGetViewport(renderer.get(), &viewport);
    void* pixels;
    int pitch_i;
    SDL_LockTexture(drawable.get(), nullptr, &pixels, &pitch_i);
    size_t pitch = static_cast<size_t>(pitch_i);
    size_t height = std::min(img.size(), static_cast<size_t>(viewport.h));
    size_t viewport_width = std::min(pitch / sizeof(Color), static_cast<size_t>(viewport.w));
    uint8_t* pixel_bytes = static_cast<uint8_t*>(pixels);
    for (size_t y=0; y<height; ++y) {
      size_t width = std::min(img[y].size(), viewport_width);
      std::memcpy(pixel_bytes + (y * pitch), reinterpret_cast<const uint8_t*>(img[y].data()), width * sizeof(Color));
    }
    SDL_UnlockTexture(drawable.get());
    SDL_SetRenderDrawColor(renderer.get(), render_draw_color.red, render_draw_color.green, render_draw_color.blue, render_draw_color.alpha);
    SDL_RenderClear(renderer.get());
    SDL_RenderCopy(renderer.get(), drawable.get(), nullptr, nullptr);
    SDL_RenderPresent(renderer.get());
    SDL_UpdateWindowSurface(window.get());
    return 0;
  }

  bool ProcessEventsOrQuit(void) {
    if (!ready) { return true; }
    SDL_Event e;
    bool not_quit = true;
    while (SDL_PollEvent(&e) != 0) {
      switch (e.type) {
      case SDL_QUIT:
        not_quit = false;
        break;
      case SDL_KEYUP:
        switch (e.key.keysym.sym) {
        case SDLK_ESCAPE:
          not_quit = false;
          break;
        }
        break;
      }
    }
    return not_quit;
  }
};


} // namespace viz
} // namespace utils
} // namespace jms


#endif // JMS_UTILS_VIZ_SDL2_RENDERER_h
