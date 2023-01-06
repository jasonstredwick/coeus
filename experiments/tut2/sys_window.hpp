#pragma once


#include <algorithm>
#include <array>
#include <expected>
#include <format>
#include <memory>
#include <new>
#include <optional>
#include <string>
#include <vector>

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#undef min
#undef max
#endif


#include <iostream>


namespace jms {

//WAYLAND_VERSION

#if defined(_WIN32) || defined(_WIN64)

#if UNICODE
using SysWindowConfigOptions_title_t = std::wstring;
#else
using SysWindowConfigOptions_title_t = std::string;
#endif

#else
using SysWindowConfigOptions_title_t = std::string;
#endif

struct SysWindowConfigOptions {
    using title_t = SysWindowConfigOptions_title_t;
    title_t title{};
    int x{-1};
    int y{-1};
    int width{-1};
    int height{-1};
};


#if defined(_WIN32) || defined(_WIN64)


// TODO: Convert to title to wstring and change wide functions
class SysWindow {
private:
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
        LRESULT OnSize() noexcept { return 0; }
    };

#if UNICODE
    inline static const std::wstring WNDCLASS_NAME{L"jms__SysWindow__WndClassEx__"};
#else
    inline static const std::string WNDCLASS_NAME{"jms__SysWindow__WndClassEx__"};
#endif
    std::unique_ptr<MsgInterface> msg_interface{nullptr};
    std::unique_ptr<HWND__, decltype([](HWND__* hwnd){DestroyWindow(hwnd);})> handle{nullptr};
    WNDCLASSEX wnd_class{.cbSize=sizeof(WNDCLASSEX)};

    SysWindow() noexcept = default;

    static LRESULT CALLBACK WndProc(HWND hwnd, UINT umsg, WPARAM wparam, LPARAM lparam) noexcept {
        // WM_DESTROY is called separately because WndProc may be called by destructor and thus the
        // interface pointer may already be deleted.  Thus potential dangling pointer issue.
        if (umsg == WM_DESTROY) {
            SetWindowLongPtr(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(nullptr));
            PostQuitMessage(0);
            return 0;
        }

        MsgInterface* msg_interface = nullptr;
        if (umsg == WM_NCCREATE) {
            LPCREATESTRUCT create_struct = reinterpret_cast<LPCREATESTRUCT>(lparam);
            msg_interface = reinterpret_cast<MsgInterface*>(create_struct->lpCreateParams);
            SetWindowLongPtr(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(msg_interface));
        } else {
            msg_interface = reinterpret_cast<MsgInterface*>(GetWindowLongPtr(hwnd, GWLP_USERDATA));
        }

        if (msg_interface) {
            switch (umsg) {
            case WM_PAINT: return msg_interface->OnPaint();
            //case WM_CREATE: return msg_interface->OnCreate();
            //case WM_DPICHANGED: return msg_interface->OnDPIChanged(wparam, lparam);
            //case WM_GETMINMAXINFO: return msg_interface->OnGetMinMaxInfo(lparam);
            //case WM_SIZE: return msg_interface->OnSize();
            }
        }
        return DefWindowProc(hwnd, umsg, wparam, lparam);
    }

public:
    SysWindow(const SysWindow&) = delete;
    SysWindow(SysWindow&& t) = default;
    ~SysWindow() = default;
    SysWindow& operator=(const SysWindow&) = delete;
    SysWindow& operator=(SysWindow&&) = default;

    static std::expected<SysWindow, std::string> Create(SysWindowConfigOptions&& options) noexcept {
        SysWindow sys_window{};

        sys_window.msg_interface.reset(new(std::nothrow) MsgInterface{});
        if (!sys_window.msg_interface) {
            return std::unexpected{std::format("Failed to create message handling interface\n")};
        }

        auto root_handle = GetModuleHandle(nullptr);
        if (!GetClassInfoEx(root_handle, WNDCLASS_NAME.c_str(), &sys_window.wnd_class)) {
            sys_window.wnd_class.lpfnWndProc = WndProc;
            sys_window.wnd_class.hInstance = root_handle;
            sys_window.wnd_class.lpszClassName = WNDCLASS_NAME.c_str();
            if (!RegisterClassEx(&sys_window.wnd_class)) {
                return std::unexpected{std::format("Failed to register class: {}\n", GetLastError())};
            }
        }

        sys_window.handle.reset(CreateWindowEx(
            0,
            sys_window.wnd_class.lpszClassName,
            options.title.empty() ? SysWindowConfigOptions_title_t{}.c_str() : options.title.c_str(),
            WS_OVERLAPPEDWINDOW,
            options.x < 0 ? CW_USEDEFAULT : options.x,
            options.y < 0 ? CW_USEDEFAULT : options.y,
            options.width <= 0 ? CW_USEDEFAULT : std::min(options.width, 640),
            options.height <= 0 ? CW_USEDEFAULT : std::min(options.height, 480),
            nullptr, // parent window
            nullptr, // menu
            root_handle,
            static_cast<void*>(sys_window.msg_interface.get()) // additional application data
        ));
        if (!sys_window.handle) {
            return std::unexpected{std::format("Failed to create window: {}\n", GetLastError())};
        } else {
            sys_window.msg_interface.get()->hwnd = sys_window.handle.get();
        }

        ShowWindow(sys_window.handle.get(), SW_SHOW);
        SetForegroundWindow(sys_window.handle.get());
        SetFocus(sys_window.handle.get());
        UpdateWindow(sys_window.handle.get());

        return sys_window;
    }

    static void SetHighDPI() noexcept {
        if (!SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2)) {
            auto error = GetLastError();
            if (error == ERROR_ACCESS_DENIED) {
                std::cout << "DPI already set\n";
            } else {
                std::cout << std::format("Failed to set dpi awareness: {}\n", error);
            }
        }
        std::cout << "High DPI awareness set\n";
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


#else


static_assert("sys_window: Unsupported OS");


#endif


}
