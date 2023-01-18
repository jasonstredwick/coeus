#pragma once


#define JMS_USE_WM_WIN32


#if   defined(JMS_USE_WM_ANDROID)
static_assert(false, "Unsupported window manager: Android");
#elif defined(JMS_USE_WM_DIRECTFB)
static_assert(false, "Unsupported window manager: DirectFB");
#elif defined(JMS_USE_WM_DIRECT_TO_DISPLAY)
static_assert(false, "Unsupported window manager: Direct-to-display");
#elif defined(JMS_USE_WM_FUCHSIA)
static_assert(false, "Unsupported window manager: Fuchsia");
#elif defined(JMS_USE_WM_GOOGLE_GAMES)
static_assert(false, "Unsupported window manager: Google Games");
#elif defined(JMS_USE_WM_IOS)
static_assert(false, "Unsupported window manager: iOS");
#elif defined(JMS_USE_WM_MACOS)
static_assert(false, "Unsupported window manager: MacOS");
#elif defined(JMS_USE_WM_METAL)
static_assert(false, "Unsupported window manager: Metal");
#elif defined(JMS_USE_WM_VI)
static_assert(false, "Unsupported window manager: VI");
#elif defined(JMS_USE_WM_QNX)
static_assert(false, "Unsupported window manager: QNX");
#elif defined(JMS_USE_WM_WAYLAND)
#include "wayland.hpp"
#elif defined(JMS_USE_WM_WIN32)
#include "win32.hpp"
#elif defined(JMS_USE_WM_XCB)
#include "xcb.hpp"
#elif defined(JMS_USE_WM_XLIB)
static_assert(false, "Unsupported window manager: Xlib");
#endif
