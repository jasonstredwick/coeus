#pragma once


#include <array>
#include <format>
#include <memory>
#include <string>

#include <windows.h>
#undef min
#undef max


#include "interface.hpp"


namespace jms {


class SysWindow : public SysWindowInterface {
private:
    struct State {
        HWND handle{nullptr};
        LRESULT OnCreate() noexcept { return 0; }
        LRESULT OnDPIChanged(WPARAM wparam, LPARAM lparam) noexcept {
            int y_dpi = HIWORD(wparam);
            int x_dpi = LOWORD(wparam);
            RECT rect{*reinterpret_cast<RECT* const>(lparam)};
            int suggested_left = rect.left;
            int suggested_top = rect.top;
            int suggested_width = rect.right - rect.left;
            int suggested_height = rect.bottom - rect.top;
            auto possible_set_window_pos_flags = SWP_NOZORDER | SWP_NOACTIVATE;
            auto default_scaled_y = static_cast<double>(y_dpi) / USER_DEFAULT_SCREEN_DPI;
            auto default_scaled_x = static_cast<double>(x_dpi) / USER_DEFAULT_SCREEN_DPI;
            return 0;
        }
        LRESULT OnGetMinMaxInfo(LPARAM lparam) noexcept { return 0; }
        LRESULT OnPaint() noexcept { ValidateRect(handle, nullptr); return 0; }
        LRESULT OnSize() noexcept { /*resize swapchain*/return 0; }
    };

#ifdef UNICODE
    inline static const std::wstring WNDCLASS_NAME{L"jms__SysWindow__WndClassEx__"};
    inline static const std::wstring TITLE{L""};
#else
    inline static const std::string WNDCLASS_NAME{"jms__SysWindow__WndClassEx__"};
    inline static const std::string TITLE{""};
#endif
    // REQUIRED: pointer address cannot change between move operations for WndProc; pointer would point to from obj
    std::unique_ptr<State> state{nullptr};
    std::unique_ptr<HWND__, decltype([](HWND__* hwnd){ DestroyWindow(hwnd); })> handle{nullptr};
    WNDCLASSEX wnd_class{.cbSize=sizeof(WNDCLASSEX)};

    static LRESULT CALLBACK WndProc(HWND hwnd, UINT umsg, WPARAM wparam, LPARAM lparam) noexcept {
        // WM_DESTROY is called separately because WndProc may be called by destructor and thus the
        // interface pointer may already be deleted.  Thus potential dangling pointer issue.
        if (umsg == WM_DESTROY) {
            SetWindowLongPtr(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(nullptr));
            PostQuitMessage(0);
            return 0;
        }

        State* state = nullptr;
        if (umsg == WM_NCCREATE) {
            LPCREATESTRUCT create_struct = reinterpret_cast<LPCREATESTRUCT>(lparam);
            state = reinterpret_cast<State*>(create_struct->lpCreateParams);
            SetWindowLongPtr(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(state));
        } else {
            state = reinterpret_cast<State*>(GetWindowLongPtr(hwnd, GWLP_USERDATA));
        }

        if (state) {
            switch (umsg) {
            case WM_PAINT: return state->OnPaint();
            //case WM_CREATE: return state->OnCreate();
            //case WM_DPICHANGED: return state->OnDPIChanged(wparam, lparam);
            //case WM_GETMINMAXINFO: return state->OnGetMinMaxInfo(lparam);
            //case WM_SIZE: return state->OnSize();
            }
        }

        return DefWindowProc(hwnd, umsg, wparam, lparam);
    }

public:
    SysWindow() = default;
    SysWindow(const SysWindow&) = delete;
    SysWindow(SysWindow&& t) = default;
    ~SysWindow() = default;
    SysWindow& operator=(const SysWindow&) = delete;
    SysWindow& operator=(SysWindow&&) = default;

    void Create(const uint32_t width_pixel, const uint32_t height_pixel,
                const int32_t pos_x_pixel=0, const int32_t pos_y_pixel=0) override {
        // Prevent Windows from auto stretching content; just want a raw rectangle of pixels
        if (!SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2)) {
            auto error = GetLastError();
            if (error != ERROR_ACCESS_DENIED) { // ERROR_ACCESS_DENIED == already set; ignore error
                throw std::runtime_error(std::format("Failed to set dpi awareness: {}\n", error));
            }
        }

        auto root_handle = GetModuleHandle(nullptr);

        if (!GetClassInfoEx(root_handle, WNDCLASS_NAME.c_str(), &wnd_class)) {
            wnd_class.lpfnWndProc = WndProc;
            wnd_class.hInstance = root_handle;
            wnd_class.lpszClassName = WNDCLASS_NAME.c_str();
            if (!RegisterClassEx(&wnd_class)) {
                throw std::runtime_error(
                    std::format("Failed to register class {}: {}\n", WNDCLASS_NAME, GetLastError()));
            }
        }

        handle.reset(CreateWindowEx(
            0,
            wnd_class.lpszClassName,
            TITLE.c_str(),
            0, //WS_VISIBLE, //WS_OVERLAPPEDWINDOW,
            static_cast<int>(pos_x_pixel),
            static_cast<int>(pos_y_pixel),
            static_cast<int>(width_pixel),
            static_cast<int>(height_pixel),
            nullptr, // parent window
            nullptr, // menu
            root_handle,
            static_cast<void*>(state.get()) // additional application data
        ));
        if (!handle) {
            throw std::runtime_error(std::format("Failed to create window: {}\n", GetLastError()));
        } else {
            state.get()->handle = handle.get();
        }

        ShowWindow(handle.get(), SW_SHOW);
        SetForegroundWindow(handle.get());
        SetFocus(handle.get());
        UpdateWindow(handle.get());
    }

    bool ProcessEvents() noexcept {
        MSG msg;
        if (PeekMessage(&msg, handle.get(), 0, 0, PM_REMOVE)) {
            switch (msg.message) {
            case WM_KEYDOWN:
                return false;
            default:
                DispatchMessage(&msg);
            }
            //DispatchMessage(&msg);
        }
        return true;
    }

    HINSTANCE GetInstance() const noexcept { return GetModuleHandle(nullptr); }
    HWND GetHWND() const noexcept { return handle.get(); }
    std::array<int, 2> GetDims() const noexcept {
        RECT rect{};
        if (!GetClientRect(handle.get(), &rect)) {
            return {0, 0};
        }
        return {rect.right - rect.left, rect.bottom - rect.top};
    }
};


}


#if 0
    struct MsgInterface {
        HWND hwnd{nullptr};
        LRESULT OnCreate() noexcept { return 0; }
        LRESULT OnDPIChanged(WPARAM wparam, LPARAM lparam) noexcept {
            int y_dpi = HIWORD(wparam);
            int x_dpi = LOWORD(wparam);
            RECT rect{*reinterpret_cast<RECT* const>(lparam)};
            int suggested_left = rect.left;
            int suggested_top = rect.top;
            int suggested_width = rect.right - rect.left;
            int suggested_height = rect.bottom - rect.top;
            auto possible_set_window_pos_flags = SWP_NOZORDER | SWP_NOACTIVATE;
            auto default_scaled_y = static_cast<double>(y_dpi) / USER_DEFAULT_SCREEN_DPI;
            auto default_scaled_x = static_cast<double>(x_dpi) / USER_DEFAULT_SCREEN_DPI;
            return 0;
        }
        LRESULT OnGetMinMaxInfo(LPARAM lparam) noexcept { return 0; }
        LRESULT OnPaint() noexcept { std::cout << "Paint\n"; ValidateRect(hwnd, nullptr); return 0; }
        LRESULT OnSize() noexcept { /*resize swapchain*/return 0; }
    };
#endif
