#pragma once


#define VULKAN_HPP_NO_CONSTRUCTORS // uses c++ aggregate initialization
#define VULKAN_HPP_FLAGS_MASK_TYPE_AS_PUBLIC
#define VULKAN_HPP_NO_SETTERS // consider
//#define VULKAN_HPP_USE_REFLECT  // slow???

#if defined(_WIN32) || defined(_WIN64)
#define VK_USE_PLATFORM_WIN32_KHR
#endif
