/*

cl.exe /std:c++latest /utf-8 /EHsc -IC:\VulkanSDK\1.3.231.1\Include .\scratch\monitors.cpp User32.lib Kernel32.lib Gdi32.lib Shell32.lib shcore.lib C:\VulkanSDK\1.3.231.1\lib\vulkan-1.lib

*/

#if 1
#include <algorithm>
#include <array>
#include <cstdint>
#include <format>
#include <iostream>
#include <limits>
#include <ranges>
#include <vector>

#include <windows.h>
#include <ShellScalingApi.h>
#undef max
#undef min

#define VULKAN_HPP_NO_CONSTRUCTORS // uses c++ aggregate initialization
#define VULKAN_HPP_FLAGS_MASK_TYPE_AS_PUBLIC
#define VULKAN_HPP_NO_SETTERS // consider
#include <vulkan/vulkan_raii.hpp>


struct Display {
    HMONITOR handle{};
    std::array<int64_t, 2> pos_upper_left{};
    std::array<int64_t, 2> dims{};
    std::array<int64_t, 2> dims_dpi{96, 96};
    std::array<int64_t, 2> dims_dpi_raw{96, 96};
};


BOOL CALLBACK EnumDisplayMonitorCallback(HMONITOR handle, [[maybe_unused]] HDC hdc, LPRECT rect, LPARAM data_ptr) {
    std::vector<Display>* displays = reinterpret_cast<std::vector<Display>*>(data_ptr);
    int64_t bottom = static_cast<int64_t>(rect->bottom);
    int64_t left = static_cast<int64_t>(rect->left);
    int64_t right = static_cast<int64_t>(rect->right);
    int64_t top = static_cast<int64_t>(rect->top);
    int64_t width = right - left;
    int64_t height = bottom - top;

    UINT x_dpi_out=0, y_dpi_out=0;
    if (GetDpiForMonitor(handle, MDT_ANGULAR_DPI, &x_dpi_out, &y_dpi_out) == E_INVALIDARG) { return FALSE; }
    int64_t x_dpi = static_cast<int64_t>(x_dpi_out);
    int64_t y_dpi = static_cast<int64_t>(y_dpi_out);

    x_dpi_out=0, y_dpi_out=0;
    if (GetDpiForMonitor(handle, MDT_RAW_DPI, &x_dpi_out, &y_dpi_out) == E_INVALIDARG) { return FALSE; }

    MONITORINFOEX monitor_info{{.cbSize=sizeof(MONITORINFOEX)}};
    if (GetMonitorInfoW(handle, &monitor_info)) { return FALSE; }
    if (rect->bottom != monitor_info.rcMonitor.bottom ||
        rect->top != monitor_info.rcMonitor.top ||
        rect->left != monitor_info.rcMonitor.left ||
        rect->right != monitor_info.rcMonitor.right) {
        std::cout << "NOT THE SAME DIMS!!!" << std::endl;
    }
    std::cout << monitor_info.rcMonitor.bottom << " " << monitor_info.rcMonitor.top << " " << monitor_info.rcMonitor.left << " " << monitor_info.rcMonitor.right << std::endl;
    std::cout << monitor_info.rcWork.bottom << " " << monitor_info.rcWork.top << " " << monitor_info.rcWork.left << " " << monitor_info.rcWork.right << std::endl;
    std::cout << "FLAGS: " << monitor_info.dwFlags << std::endl;
    std::wcout << "FOO: " << monitor_info.szDevice << std::endl;
    std::wcout << "INFO: "<<std::endl << monitor_info.szDevice << std::endl;

    displays->push_back({
        .handle=handle,
        .pos_upper_left={left, top},
        .dims={width, height},
        .dims_dpi={x_dpi, y_dpi},
        .dims_dpi_raw={static_cast<int64_t>(x_dpi_out), static_cast<int64_t>(y_dpi_out)}
    });

    return TRUE;
}


int main() {
    if (!SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2)) {
        auto error = GetLastError();
        if (error != ERROR_ACCESS_DENIED) { // ERROR_ACCESS_DENIED == already set; ignore error
            std::cout << std::format("Failed to set dpi awareness: {}\n", error);
            return 0;
        }
    }

#if 0
    std::vector<DISPLAY_DEVICE> display_devices{};
    for (DWORD i=0; i<std::numeric_limits<DWORD>::max(); ++i) {
        DISPLAY_DEVICE dd{.cb=sizeof(DISPLAY_DEVICE)};
        if (!EnumDisplayDevices(nullptr, i, &dd, 0)) { break; }
        // ACTIVE and ATTACHED_TO_DESKTOP are the same defined value
        if (dd.StateFlags & DISPLAY_DEVICE_ACTIVE && !(dd.StateFlags & DISPLAY_DEVICE_MIRRORING_DRIVER)) {
            display_devices.push_back(std::move(dd));
        }
    }

    std::vector<std::vector<DEVMODE>> device_modes(display_devices.size());
    for (size_t index=0; index<display_devices.size(); ++index) {
        auto& device = display_devices[index];
        auto& modes = device_modes[index];
        for (DWORD i=0; i<std::numeric_limits<DWORD>::max(); ++i) {
            DEVMODE device_mode{.dmSize=sizeof(DEVMODE)};
            if (!EnumDisplaySettings(device.DeviceName, i, &device_mode)) { break; }
            modes.push_back(std::move(device_mode));
        }
    }

    for (size_t index=0; index<display_devices.size(); ++index) {
        auto& device = display_devices[index];
        auto& modes = device_modes[index];
        std::cout << "Device-" << std::endl;
        std::wcout << "  Name: " << device.DeviceName << std::endl;
        std::wcout << "  String: " << device.DeviceString << std::endl;
        std::wcout << "  ID: " << device.DeviceID << std::endl;
        std::wcout << "  Key: " << device.DeviceKey << std::endl;
        if (device.StateFlags) {
            std::wcout << "  State-" << std::endl;
            std::wcout << "    ACC_DRIVER: " << (device.StateFlags & DISPLAY_DEVICE_ACC_DRIVER) << std::endl;
            std::wcout << "    ACTIVE: " << (device.StateFlags & DISPLAY_DEVICE_ACTIVE) << std::endl;
            std::wcout << "    ATTACHED: " << (device.StateFlags & DISPLAY_DEVICE_ATTACHED) << std::endl;
            std::wcout << "    ATTACHED_DESKTOP: " << (device.StateFlags & DISPLAY_DEVICE_ATTACHED_TO_DESKTOP) << std::endl;
            std::wcout << "    DISCONNECT: " << (device.StateFlags & DISPLAY_DEVICE_DISCONNECT) << std::endl;
            std::wcout << "    MIRRORING: " << (device.StateFlags & DISPLAY_DEVICE_MIRRORING_DRIVER) << std::endl;
            std::wcout << "    MODESPRUNED: " << (device.StateFlags & DISPLAY_DEVICE_MODESPRUNED) << std::endl;
            std::wcout << "    MULTI: " << (device.StateFlags & DISPLAY_DEVICE_MULTI_DRIVER) << std::endl;
            std::wcout << "    PRIMARY: " << (device.StateFlags & DISPLAY_DEVICE_PRIMARY_DEVICE) << std::endl;
            std::wcout << "    RDPUDD: " << (device.StateFlags & DISPLAY_DEVICE_RDPUDD) << std::endl;
            std::wcout << "    REMOTE: " << (device.StateFlags & DISPLAY_DEVICE_REMOTE) << std::endl;
            std::wcout << "    REMOVABLE: " << (device.StateFlags & DISPLAY_DEVICE_REMOVABLE) << std::endl;
            std::wcout << "    TSCOMPAT: " << (device.StateFlags & DISPLAY_DEVICE_TS_COMPATIBLE) << std::endl;
            std::wcout << "    UNSAFEMODE: " << (device.StateFlags & DISPLAY_DEVICE_UNSAFE_MODES_ON) << std::endl;
            std::wcout << "    VGACOMPAT: " << (device.StateFlags & DISPLAY_DEVICE_VGA_COMPATIBLE) << std::endl;
        }
        std::wcout << "  NumModes: " << modes.size() << std::endl;
    }

    std::cout << std::endl;
#endif

    //Vulkan();

#if 1
    try {
        std::vector<Display> displays{};
        if (!EnumDisplayMonitors(nullptr, nullptr, EnumDisplayMonitorCallback, (LPARAM)&displays)) {
            throw std::runtime_error("Failed to enumerate displays.");
        }
        for (int i=0; i<displays.size(); ++i) {
            std::cout << "Screen id: " << i << std::endl;
            std::cout << "---------------------------------------------------" << std::endl;
            std::cout << " - screen left top corner coords: (" << displays[i].pos_upper_left[0] << ", " << displays[i].pos_upper_left[1] << ")" << std::endl;
            std::cout << " - screen bottom right corner coords: (" << (displays[i].pos_upper_left[0] + displays[i].dims[0]) << ", " <<
                                                                    (displays[i].pos_upper_left[1] + displays[i].dims[1]) << ")" << std::endl;
            std::cout << " - screen dims: (" << displays[i].dims[0] << ", " << displays[i].dims[1] << ")" << std::endl;
            std::cout << "  - Has HMONITOR: " << (bool)(displays[i].handle != nullptr) << std::endl;
            std::cout << "  - DPI: (" << displays[i].dims_dpi[0] << ", " << displays[i].dims_dpi[1] << ")" << std::endl;
            std::cout << "  - DPI RAW: (" << displays[i].dims_dpi_raw[0] << ", " << displays[i].dims_dpi_raw[1] << ")" << std::endl;
            std::cout << "---------------------------------------------------" << std::endl;
        }
    } catch (std::exception& exp) {
        std::cout << "Caught Exception\n" << exp.what() << std::endl;;
    }
#endif

    DISPLAY_DEVICEW display_device{.cb=sizeof(DISPLAY_DEVICE)};
    DWORD screen_id = 0;
    while (EnumDisplayDevicesW(nullptr, screen_id, &display_device, 0)) {
        DISPLAY_DEVICEW dd2 = display_device;
        if (EnumDisplayDevicesW(dd2.DeviceName, 0, &display_device, EDD_GET_DEVICE_INTERFACE_NAME)) {
            std::wcout << "DEV NAME: " << display_device.DeviceID << std::endl;
        std::wcout << "DD1: " << dd2.DeviceName << std::endl;
        std::wcout << "DD1: " << dd2.DeviceID << std::endl;
        std::wcout << "DD1: " << dd2.DeviceKey << std::endl;
        std::wcout << "DD1: " << dd2.DeviceString << std::endl;
        std::wcout << "DD2: " << display_device.DeviceName << std::endl;
        std::wcout << "DD2: " << display_device.DeviceID << std::endl;
        std::wcout << "DD2: " << display_device.DeviceKey << std::endl;
        std::wcout << "DD2: " << display_device.DeviceString << std::endl;
        }
        ++screen_id;
    }


    return 0;
}


struct DebugConfig {
    static constexpr PFN_vkDebugUtilsMessengerCallbackEXT DefaultDebugFn = +[](
        VkDebugUtilsMessageSeverityFlagBitsEXT,
        VkDebugUtilsMessageTypeFlagsEXT,
        const VkDebugUtilsMessengerCallbackDataEXT *,
        void*) -> VkBool32 { return VK_FALSE; };

    vk::DebugUtilsMessageSeverityFlagsEXT severity_flags{
        //vk::DebugUtilsMessageSeverityFlagBitsEXT::eVerbose |
        //vk::DebugUtilsMessageSeverityFlagBitsEXT::eInfo |
        vk::DebugUtilsMessageSeverityFlagBitsEXT::eWarning |
        vk::DebugUtilsMessageSeverityFlagBitsEXT::eError
    };
    vk::DebugUtilsMessageTypeFlagsEXT msg_type_flags{
        vk::DebugUtilsMessageTypeFlagBitsEXT::eGeneral |
        vk::DebugUtilsMessageTypeFlagBitsEXT::eValidation |
        vk::DebugUtilsMessageTypeFlagBitsEXT::ePerformance
    };
    PFN_vkDebugUtilsMessengerCallbackEXT debug_fn = DebugConfig::DefaultDebugFn;
};

VKAPI_ATTR VkBool32 VKAPI_CALL DebugMessage_cout(VkDebugUtilsMessageSeverityFlagBitsEXT msg_severity,
                                                 VkDebugUtilsMessageTypeFlagsEXT msg_type,
                                                 VkDebugUtilsMessengerCallbackDataEXT const* callback_data,
                                                 void* user_data) {
    auto& out = std::cout;
    out << std::format("{} ({}): {} {}\n{}\n",
        vk::to_string(static_cast<vk::DebugUtilsMessageSeverityFlagBitsEXT>(msg_severity)),
        vk::to_string(static_cast<vk::DebugUtilsMessageTypeFlagsEXT>(msg_type)),
        std::string{callback_data->pMessageIdName},
        callback_data->messageIdNumber,
        std::string{callback_data->pMessage});

    /*
    if (callback_data->queueLabelCount > 0) {
        out << std::format("\nQueue Labels:\n");
        for (size_t i : std::views::iota(static_cast<size_t>(0), static_cast<size_t>(callback_data->queueLabelCount))) {
            out << std::format("\tName: {}\n", callback_data->pQueueLabels[i].pLabelName);
        }
    }

    if (callback_data->cmdBufLabelCount > 0) {
        out << std::format("\nCommandBuffer Labels:\n");
        for (size_t i : std::views::iota(static_cast<size_t>(0), static_cast<size_t>(callback_data->cmdBufLabelCount))) {
            out << std::format("\tName: {}\n", callback_data->pCmdBufLabels[i].pLabelName);
        }
    }

    if (callback_data->objectCount > 0) {
        out << std::format("\nObjects:\n");
        for (size_t i : std::views::iota(static_cast<size_t>(0), static_cast<size_t>(callback_data->objectCount))) {
            std::string obj_name = (callback_data->pObjects[i].pObjectName) ?
                std::format("\t{}", std::string_view{callback_data->pObjects[i].pObjectName}) : "";
            out << std::format("\t{}\t{}\t{}{}\n",
                i, callback_data->pObjects[i].objectType, callback_data->pObjects[i].objectHandle, obj_name);
        }
    }
    */

    return VK_FALSE;
}


void Vulkan() {
    std::cout << "VULKAN" << std::endl;
    vk::raii::Context context{};
    vk::ApplicationInfo application_info{ .apiVersion=context.enumerateInstanceVersion() };
    std::vector<const char*> layer_names_vec{
        "VK_LAYER_KHRONOS_synchronization2",
        "VK_LAYER_KHRONOS_validation"
    };
    std::vector<const char*> ext_names_vec{};
    /*
        "VK_KHR_display",
        "VK_KHR_surface"
        "VK_KHR_get_display_properties2",
        "VK_KHR_get_physical_device_properties2"
    };
    */
    auto exts = context.enumerateInstanceExtensionProperties();
    for (auto& ext : exts) {
        ext_names_vec.push_back(ext.extensionName);
        //std::cout << "EXT: " << ext.extensionName << std::endl;
    }
    vk::raii::Instance instance = vk::raii::Instance{context, vk::InstanceCreateInfo{
        .flags=vk::InstanceCreateFlags(),
        .pApplicationInfo=&application_info,
        .enabledLayerCount=static_cast<uint32_t>(layer_names_vec.size()),
        .ppEnabledLayerNames=layer_names_vec.data(),
        .enabledExtensionCount=static_cast<uint32_t>(ext_names_vec.size()),
        .ppEnabledExtensionNames=ext_names_vec.data()
    }};

    DebugConfig debug{.debug_fn=DebugMessage_cout};
    vk::DebugUtilsMessengerCreateInfoEXT debug_messenger_create_info{
        .flags=vk::DebugUtilsMessengerCreateFlagsEXT(),
        .messageSeverity=debug.severity_flags,
        .messageType=debug.msg_type_flags,
        .pfnUserCallback=debug.debug_fn
    };
    vk::raii::DebugUtilsMessengerEXT debug_messenger = vk::raii::DebugUtilsMessengerEXT{instance, debug_messenger_create_info};

    vk::raii::PhysicalDevices physical_devices = vk::raii::PhysicalDevices{instance};
    std::cout << "Num physical devices: " << physical_devices.size() << std::endl;
    for (size_t index=0; index<physical_devices.size(); ++index) {
        vk::raii::PhysicalDevice& physical_device = physical_devices.at(index);
        std::vector<vk::DisplayPropertiesKHR> props = physical_device.getDisplayPropertiesKHR();
        std::cout << "Device " << index << std::endl;
        for (auto& prop : props) {
            std::cout << "DisplayProps-" << std::endl;
            std::cout << "  DriverVersion: " << prop.displayName << std::endl;
            std::cout << "  Dimensions: " << prop.physicalDimensions.width << " x " << prop.physicalDimensions.height << std::endl;
            std::cout << "  Resolution: " << prop.physicalResolution.width << " x " << prop.physicalResolution.height << std::endl;
        }
        std::cout << "DevProps-" << std::endl;
        vk::PhysicalDeviceProperties2 dev_props2 = physical_device.getProperties2();
        vk::PhysicalDeviceProperties dev_props = dev_props2.properties;
        std::cout << "  DriverVersion: " << dev_props.driverVersion << std::endl;
        std::cout << "  VendorId:" << dev_props.vendorID << std::endl;
        std::cout << "  DeviceId: " << dev_props.deviceID << std::endl;
        //std::cout << "  DeviceType: " << dev_props.deviceType << std::endl;
        std::cout << "  DeviceName: " << dev_props.deviceName << std::endl;
        std::cout << "IDProps" << std::endl;
        VkBaseOutStructure* next = reinterpret_cast<VkBaseOutStructure*>(dev_props2.pNext);
        int count = 0;
        std::cout << "IS NULL: " << (bool)(dev_props2.pNext == nullptr) << std::endl;
        while (next != nullptr) {
            switch (next->sType) {
            case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ID_PROPERTIES:
                VkPhysicalDeviceIDProperties* id_props = reinterpret_cast<VkPhysicalDeviceIDProperties*>(next);
                std::cout << "  deviceUUID: ";
                for (auto& x : id_props->deviceUUID) { std::cout << x << " "; } std::cout << std::endl;
                std::cout << "  driverUUID: ";
                for (auto& x : id_props->driverUUID) { std::cout << x << " "; } std::cout << std::endl;
                if (id_props->deviceLUIDValid) {
                    std::cout << "  LUID: ";
                    for (auto& x : id_props->deviceLUID) { std::cout << x << " "; } std::cout << std::endl;
                } else {
                    std::cout << "  LUID: NONE" << std::endl;
                }
                break;
            }
            next = reinterpret_cast<VkBaseOutStructure*>(next->pNext);
            count++;
        }
        std::cout << "COunt: " << count << std::endl;
        std::cout << std::endl;
    }
}


#endif
