#pragma once


#include <array>
#include <iostream> //temp
#include <format>
#include <memory>
#include <new>
#include <string>

#include <wayland-client.h>

#include "config.hpp"


namespace jms {


class SysWindow {
private:
    struct State {
        std::unique_ptr<wl_display,       decltype(&wl_display_disconnect)>    display      {nullptr, &wl_display_disconnect};
        std::unique_ptr<wl_registry,      decltype(&wl_registry_destroy)>      registry     {nullptr, &wl_registry_destroy};
        std::unique_ptr<wl_compositor,    decltype(&wl_compositor_destroy)>    compositor   {nullptr, &wl_compositor_destroy};
        std::unique_ptr<wl_surface,       decltype(&wl_surface_destroy)>       surface      {nullptr, &wl_surface_destroy};
        std::unique_ptr<wl_shell,         decltype(&wl_shell_destroy)>         shell        {nullptr, &wl_shell_destroy};
        std::unique_ptr<wl_shell_surface, decltype(&wl_shell_surface_destroy)> shell_surface{nullptr, &wl_shell_surface_destroy};
        std::unique_ptr<wl_seat,          decltype(&wl_seat_destroy)>          seat         {nullptr, &wl_seat_destroy};
        std::unique_ptr<wl_pointer,       decltype(&wl_pointer_destroy)>       pointer      {nullptr, &wl_pointer_destroy};
        std::unique_ptr<wl_keyboard,      decltype(&wl_keyboard_destroy)>      keyboard     {nullptr, &wl_keyboard_destroy};
    };

    std::unique_ptr<State> state{nullptr};
    wl_keyboard_listener keyboard_listeners{
        .keymap=nullptr,
        .enter=nullptr,
        .leave=nullptr,
        .key=[](void *data, struct wl_keyboard *keyboard, uint32_t serial, uint32_t time, uint32_t key, uint32_t key_state) {
            if (key_state != WL_KEYBOARD_KEY_STATE_RELEASED) { return; }
            auto* state = reinterpret_cast<State*>(data);
            switch (key) {
                case KEY_ESC:  // Escape
                    break;
                case KEY_UP:  // up arrow key
                    break;
                case KEY_DOWN:  // right arrow key
                    break;
                case KEY_SPACE:  // space bar
                    break;
                default:
                    break;
            }
            //state->game_.on_key(game_key);
        },
        .modifiers=nullptr,
        .repeat_info=nullptr
    };
    wl_pointer_listener pointer_listeners{
        .enter=nullptr,
        .leave=nullptr,
        .motion=nullptr,
        .button=[](void *data, struct wl_pointer *wl_pointer, uint32_t serial, uint32_t time, uint32_t button, uint32_t button_state) {
            if (button == BTN_LEFT && button_state == WL_POINTER_BUTTON_STATE_PRESSED) {
                auto* state = reinterpret_cast<State*>(data);
                wl_shell_surface_move(state->shell_surface.get(), state->seat.get(), serial);
            }
        },
        .axis=nullptr,
        .frame=nullptr,
        .axis_source=nullptr,
        .axis_stop=nullptr,
        .axis_discrete=nullptr
    };
    wl_registry_listener registry_listener{
        .global=[](void *data, wl_registry *registry, uint32_t id, const char *interface, uint32_t version) {
            auto* state = reinterpret_cast<State*>(data);
            std::string interface_name{interface};
            if (interface_name == std::string{"wl_compositor"}) {
                state->compositor.reset(
                    reinterpret_cast<wl_compositor*>(wl_registry_bind(registry, id, &wl_compositor_interface, 1)));
            } else if (interface_name == std::string{"wl_shell"}) {
                state->shell.reset(
                    reinterpret_cast<wl_shell*>(wl_registry_bind(registry, id, &wl_shell_interface, 1)));
            } else if (interface_name == std::string{"wl_seat"}) {
                state->seat.reset(
                    reinterpret_cast<wl_seat*>(wl_registry_bind(registry, id, &wl_seat_interface, 1)));
            }
        },
        .global_remove=nullptr
    };
    wl_seat_listener seat_listener{
        .capabilities=[](void *data, wl_seat *seat, uint32_t caps) {
            auto* state = reinterpret_cast<State*>(data);

            // Subscribe to pointer events
            if ((caps & WL_SEAT_CAPABILITY_POINTER) && !state->pointer) {
                state->pointer.reset(wl_seat_get_pointer(state->seat.get()));
                wl_pointer_add_listener(state->pointer.get(), &state->pointer_listeners, state);
            } else if (!(caps & WL_SEAT_CAPABILITY_POINTER) && state->pointer) {
                state->pointer.reset(nullptr);
            }

            // Subscribe to keyboard events
            if (caps & WL_SEAT_CAPABILITY_KEYBOARD) {
                state->keyboard.reset(wl_seat_get_keyboard(state->seat.get()));
                wl_keyboard_add_listener(state->keyboard.get(), &state->keyboard_listeners, state);
            } else if (!(caps & WL_SEAT_CAPABILITY_KEYBOARD)) {
                state->keyboard.reset(nullptr);
            }
        },
        .name=nullptr
    };
    wl_shell_surface_listener shell_surface_listener{
        .ping=[](void *data, wl_shell_surface *shell_surface, uint32_t serial) {
            wl_shell_surface_pong(shell_surface, serial);
        },
        .configure=nullptr,
        .popup_done=nullptr
    };

    SysWindow() noexcept = default;

public:
    SysWindow(const SysWindow&) = delete;
    SysWindow(SysWindow&& t) = default;
    ~SysWindow() = default;
    SysWindow& operator=(const SysWindow&) = delete;
    SysWindow& operator=(SysWindow&&) = default;

    static SysWindow Create(SysWindowConfigOptions&& options) {
        SysWindow sys_window{};

        sys_window.state.reset(new State{});

        sys_window.state->display.reset(wl_display_connect(nullptr));
        if (!sys_window.state->display) { throw std::runtime_error("Failed to connect to the display server."); }

        sys_window.state->registry.reset(wl_display_get_registry(display.get()));
        if (!sys_window.state->registry) { throw std::runtime_error("Failed to get registry."); }

        wl_registry_add_listener(sys_window.state->registry.get(), &registry_listener, sys_window.state->get());
        wl_display_roundtrip(sys_window.state->display.get()); // forces code to wait until registry query is complete.

        if (!sys_window.state->compositor) { throw std::runtime_error("Failed to bind compositor."); }

        sys_window.state->surface.reset(wl_compositor_create_surface(sys_window.state->compositor.get()));
        if (!sys_window.state->surface) { throw std::runtime_error("Failed to create surface."); }

        // "desktop window functionality"
        if (!sys_window.state->shell) { throw std::runtime_error("Failed to bind shell."); }
        if (!sys_window.state->seat) { throw std::runtime_error("Failed to bind seat."); }
        wl_seat_add_listener(sys_window.state->seat.get(), &seat_listener, sys_window.state->get());

        sys_window.state->shell_surface.reset(wl_shell_get_shell_surface(sys_window.state->shell.get(),
                                                                         sys_window.state->surface.get()));
        if (!sys_window.state->shell_surface) { throw std::runtime_error("Failed to shell_surface."); }

        wl_shell_surface_add_listener(sys_window.state->shell_surface.get(), &shell_surface_listener, sys_window.state->get());
        wl_shell_surface_set_title(sys_window.state->shell_surface.get(), options.title.c_str());
        wl_shell_surface_set_toplevel(sys_window.state->shell_surface->get());

        return sys_window;
    }

    static void SetHighDPI() noexcept {}
    bool ProcessEvents() noexcept { return true; }

#if 0
    HINSTANCE GetInstance() const noexcept { return GetModuleHandle(nullptr); }
    HWND GetHWND() const noexcept { return handle.get(); }
    std::array<int, 2> GetDims() const noexcept {
        RECT rect{};
        if (!GetClientRect(handle.get(), &rect)) {
            return {0, 0};
        }
        return {rect.right - rect.left, rect.bottom - rect.top};
    }
#endif
};


}
