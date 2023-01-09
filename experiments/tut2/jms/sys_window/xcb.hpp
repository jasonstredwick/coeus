#pragma once


#include <array>
#include <iostream> //temp
#include <format>
#include <memory>
#include <new>
#include <string>

#include <xcb/xcb.h>

#include "config.hpp"


namespace jms {


class SysWindow {
private:
    std::unique_ptr<xcb_connection_t, decltype(&xcb_disconnect)> connection{nullptr, &xcb_disconnect};
    xcb_screen_t* screen = nullptr;
    std::unique_ptr<xcb_window_t, > window = -1; // uint32_t, but assigned -1 in function upon error ...
    xcb_atom_t wm_protocols = 0;
    xcb_atom_t wm_delete_window = 0;

    SysWindow() noexcept = default;

    void HandleEvent(const xcb_generic_event_t *event) {
        switch (event->response_type & 0x7f) {
            //case XCB_CONFIGURE_NOTIFY: {
            //    const xcb_configure_notify_event_t *notify = reinterpret_cast<const xcb_configure_notify_event_t *>(event);
            //    resize_swapchain(notify->width, notify->height);
            //} break;
            case XCB_KEY_PRESS: {
                const xcb_key_press_event_t *press = reinterpret_cast<const xcb_key_press_event_t *>(event);

                // TODO translate xcb_keycode_t
                switch (press->detail) {
                    case 9:
                        //key = Game::KEY_ESC;
                        break;
                    case 111:
                        //key = Game::KEY_UP;
                        break;
                    case 116:
                        //key = Game::KEY_DOWN;
                        break;
                    case 65:
                        //key = Game::KEY_SPACE;
                        break;
                    case 41:
                        //key = Game::KEY_F;
                        break;
                    default:
                        //key = Game::KEY_UNKNOWN;
                        break;
                }

                //game_.on_key(key);
            } break;
            case XCB_CLIENT_MESSAGE: {
                const xcb_client_message_event_t *msg = reinterpret_cast<const xcb_client_message_event_t *>(event);
                //if (msg->type == wm_protocols_ && msg->data.data32[0] == wm_delete_window_) game_.on_key(Game::KEY_SHUTDOWN);
            } break;
            default:
                break;
        }
    }

    static xcb_intern_atom_cookie_t InternAtomCookie(xcb_connection_t *c, const std::string &s) {
        return xcb_intern_atom(c, false, s.size(), s.c_str());
    }

    static xcb_atom_t InternAtom(xcb_connection_t *c, xcb_intern_atom_cookie_t cookie) {
        std::unique_ptr<xcb_intern_atom_reply_t, decltype(&free)> reply{
            xcb_intern_atom_reply(c, cookie, nullptr), &free};
        xcb_atom_t atom = reply ? reply->atom : XCB_ATOM_NONE;
        return atom;
    }

public:
    SysWindow(const SysWindow&) = delete;
    SysWindow(SysWindow&& t) = default;
    ~SysWindow() {
        if (connection) {
            xcb_destroy_window(connection.get(), window);
            xcb_flush(connection.get());
        }
    }
    SysWindow& operator=(const SysWindow&) = delete;
    SysWindow& operator=(SysWindow&&) = default;

    static SysWindow Create(SysWindowConfigOptions&& options) {
        SysWindow sys_window{};

        int screen_num = -1;
        sys_window.connection.reset(xcb_connect(nullptr, &screen_num));
        if (!sys_window.connection || xcb_connection_has_error(sys_window.connection)) {
            throw std::runtime_error("Failed to connect to the display server.");
        }
        const xcb_setup_t* setup = xcb_get_setup(sys_window.connection.get());
        xcb_screen_iterator_t iter = xcb_setup_roots_iterator(setup);
        while (screen_id-- > 0) { xcb_screen_next(&iter); }
        sys_window.screen = iter.data;

        sys_window.window = xcb_generate_id(sys_window.connection.get());

        uint32_t value_mask = XCB_CW_BACK_PIXEL | XCB_CW_EVENT_MASK;
        std::array<uint32_t, 32> value_list{};
        value_list[0] = sys_window.screen->black_pixel;
        value_list[1] = XCB_EVENT_MASK_KEY_PRESS | XCB_EVENT_MASK_STRUCTURE_NOTIFY;
        xcb_create_window(sys_window.connection.get(),
                          XCB_COPY_FROM_PARENT,
                          sys_window.window,
                          sys_window.screen->root,
                          options.x < 0 ? 0 : static_cast<int16_t>(options.x),
                          options.y < 0 ? 0 : static_cast<int16_t>(options.y),
                          options.width <= 0 ? 640 : static_cast<int16_t>(std::min(options.width, 640)),
                          options.height <= 0 ? 480 : static_cast<int16_t>(std::min(options.height, 480)),
                          0,
                          XCB_WINDOW_CLASS_INPUT_OUTPUT,
                          sys_window.screen->root_visual,
                          value_mask,
                          value_list.data());

        xcb_intern_atom_cookie_t utf8_string_cookie = SysWindow::InternAtomCookie(sys_window.connection.get(),
                                                                                  "UTF8_STRING");
        xcb_intern_atom_cookie_t wm_name_cookie = SysWindow::InternAtomCookie(sys_window.connection.get(),
                                                                              "WM_NAME");
        xcb_intern_atom_cookie_t wm_protocols_cookie = SysWindow::InternAtomCookie(sys_window.connection.get(),
                                                                                   "WM_PROTOCOLS");
        xcb_intern_atom_cookie_t wm_delete_window_cookie = SysWindow::InternAtomCookie(sys_window.connection.get(),
                                                                                       "WM_DELETE_WINDOW");

        // set title
        xcb_atom_t utf8_string = SysWindow::InternAtom(sys_window.connection.get(), utf8_string_cookie);
        xcb_atom_t wm_name = SysWindow::InternAtom(sys_window.connection.get(), wm_name_cookie);
        auto title = options.title.empty() ? SysWindowConfigOptions_title_t{} : options.title;
        xcb_change_property(sys_window.connection.get(), XCB_PROP_MODE_REPLACE, sys_window.window,
                            wm_name, utf8_string, 8, title.size(), title.c_str());

        // advertise WM_DELETE_WINDOW
        sys_window.wm_protocols = SysWindow::InternAtom(sys_window.connection.get(), wm_protocols_cookie);
        sys_window.wm_delete_window = SysWindow::InternAtom(sys_window.connection.get(), wm_delete_window_cookie);
        xcb_change_property(sys_window.connection.get(), XCB_PROP_MODE_REPLACE, sys_window.window,
                            sys_window.wm_protocols, XCB_ATOM_ATOM, 32, 1, &sys_window.wm_delete_window);

        xcb_map_window(sys_window.connection.get(), sys_window.window);
        xcb_flush(sys_window.connection.get());

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
