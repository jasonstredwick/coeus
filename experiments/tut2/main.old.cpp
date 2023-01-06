#include <algorithm>
#include <cstddef>
#include <exception>
#include <expected>
#include <format>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <ranges>
#include <string>
#include <vector>

#include "vulkan/vulkan.h"
#include <windows.h>
#include "vulkan/vulkan_win32.h"

#include "sys_window.h"


void Run(const jms::SysWindow& sys_window) noexcept;


int main(char** argv, int argc) {
    char a = 0;
    std::cout << std::format("Start\n");
    jms::SysWindow::SetHighDPI();
    std::expected<jms::SysWindow, std::wstring> sys_window_result = jms::SysWindow::Create({});
    if (!sys_window_result) {
        std::wcout << L"Failed to create SysWindow.\n" << sys_window_result.error();
        return 0;
    }
    jms::SysWindow sys_window{std::move(sys_window_result.value())};
    Run(sys_window);
    std::wcout << "---------------------\n";
    while (sys_window.ProcessEvents()) { ; }
    //std::cin >> a;
    std::cout << std::format("End\n");
    return 0;
}


template <typename T>
struct ExtractResult {
    VkResult extraction_result = VK_ERROR_UNKNOWN;
    uint32_t num = 0;
    std::vector<T> values{};
};


template <typename T, typename GetFn_t, typename ...Args>
ExtractResult<T> Extract(GetFn_t GetFn, Args&&... args) noexcept {
    uint32_t num = 0;
    VkResult result = GetFn(std::forward<Args>(args)..., &num, nullptr);
    if (result != VK_SUCCESS) { return {.extraction_result=result}; }
    std::vector<T> values(num);
    if (num) { result = GetFn(std::forward<Args>(args)..., &num, values.data()); }
    return {.extraction_result=result, .num=num, .values=std::move(values)};
}


std::vector<uint32_t> LoadShader(const std::string& filename) noexcept {
    std::ifstream f{filename, std::ios::ate | std::ios::binary};
    if (!f.is_open()) {
        return std::vector<uint32_t>{};
    }
    size_t file_size = static_cast<size_t>(f.tellg());
    size_t num_uint32 = (file_size / sizeof(uint32_t)) + ((file_size % sizeof(uint32_t) ? 1 : 0));
    std::vector<uint32_t> buffer(num_uint32);
    std::cout << "Load: " << file_size << " - " << num_uint32 * sizeof(uint32_t) << std::endl;
    f.seekg(0);
    f.read(reinterpret_cast<char*>(buffer.data()), file_size);
    return buffer;
}


VKAPI_ATTR VkBool32 VKAPI_CALL DebugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT msg_severity,
                                             VkDebugUtilsMessageTypeFlagsEXT msg_type,
                                             const VkDebugUtilsMessengerCallbackDataEXT* callback_data,
                                             void* pUserData) {
    switch (msg_severity) {
    case VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT:
        std::cout << "INFO";
        break;
    case VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT:
        std::cout << "WARNING";
        break;
    case VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT:
        std::cout << "ERROR";
        break;
    default:
        std::cout << "VERBOSE";
        break;
    }
    std::cout << " (";
    switch (msg_type) {
    case VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT:
        std::cout << "general";
        break;
    case VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT:
        std::cout << "validation";
        break;
    case VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT:
        std::cout << "performance";
        break;
    default:
        std::cout << "unknown";
        break;
    }
    std::cout << "): ";
    std::cout << "ValidationLayer: " << callback_data->pMessage << std::endl;
    return VK_FALSE;
}


// surface and swap chain are only required if rendering to screen is desired
void Run(const jms::SysWindow& sys_window) noexcept {
    // Grab API version
    uint32_t api_version = 0;
    if (vkEnumerateInstanceVersion(&api_version) != VK_SUCCESS) {
        std::wcout << std::format(L"Failed to retrieve Vulkan api version.\n");
        return;
    }

    // Validation layers
    const std::vector<const char*> validation_layers = {"VK_LAYER_KHRONOS_validation"};
    const bool enable_validation_layers = false;
    auto layer_result = Extract<VkLayerProperties>(
        []<typename... Args>(int dummy, Args... args) {
            return vkEnumerateInstanceLayerProperties(std::forward<Args>(args)...);
        }, 0);
    if (layer_result.extraction_result != VK_SUCCESS) {
        std::wcout << std::format(L"Failed to enumerate layers.\n");
        return;
    }
    std::vector<VkLayerProperties> layers{std::move(layer_result.values)};
    bool enable_validation = false;
    if (enable_validation_layers) {
        for (auto& layer : layers) {
            if (std::string{layer.layerName} == std::string{"VK_LAYER_KHRONOS_validation"}) {
                enable_validation = true;
                break;
            }
        }
    }
    std::cout << "EnableValidation: " << enable_validation << std::endl;

    // Get list of extensions
    auto ext_prop_result = Extract<VkExtensionProperties>(vkEnumerateInstanceExtensionProperties, nullptr);
    if (ext_prop_result.extraction_result != VK_SUCCESS) {
        std::wcout << std::format(L"Failed to retrieve instance extension properties.\n");
        return;
    }
    std::vector<char const*> extension_names{};
    extension_names.reserve(ext_prop_result.values.size());
    std::ranges::transform(ext_prop_result.values, std::back_inserter(extension_names),
                           [](auto& prop) { return prop.extensionName; });

    // Prepare and construct Vulkan instance
    VkApplicationInfo application_info{
        .sType=VK_STRUCTURE_TYPE_APPLICATION_INFO,
        .pNext=nullptr,
        .pApplicationName="VulkanExample",
        .applicationVersion=0,
        .pEngineName="VulkanEngine",
        .engineVersion=0,
        .apiVersion=api_version
    };
    VkInstanceCreateInfo create_info{
        .sType=VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,
        .pNext=nullptr,
        .flags=0,
        .pApplicationInfo=&application_info,
        .enabledLayerCount=(enable_validation) ? static_cast<uint32_t>(validation_layers.size()) : 0,
        .ppEnabledLayerNames=(enable_validation) ? validation_layers.data() : nullptr,
        .enabledExtensionCount=static_cast<uint32_t>(extension_names.size()),
        .ppEnabledExtensionNames=extension_names.data()
    };

    auto DestroyInstance = [allocator_fn=nullptr](auto ptr) { vkDestroyInstance(ptr, allocator_fn); };
    VkInstance instance_raw = nullptr;
    VkResult result = vkCreateInstance(&create_info, nullptr, &instance_raw);
    if (result == VK_ERROR_INCOMPATIBLE_DRIVER) {
        std::wcout << std::format(L"Failed to create a Vulkan instance.\nCannot find compatible client driver.\n");
        return;
    } else if (result != VK_SUCCESS) {
        std::wcout << std::format(L"Failed to create a Vulkan instance.\n");
        return;
    }
    std::unique_ptr<VkInstance_T, decltype(DestroyInstance)> instance{instance_raw, DestroyInstance};

    // Turn on debugging callback
    VkDebugUtilsMessengerCreateInfoEXT debug_create_info{
        .sType=VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT,
        .pNext=nullptr,
        .messageSeverity=VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT,
        .messageType=VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT,
        .pfnUserCallback=DebugCallback,
        .pUserData=nullptr
    };

    auto CreateDebugUtilsMessengerEXT = (PFN_vkCreateDebugUtilsMessengerEXT)vkGetInstanceProcAddr(
        instance->get(), "vkCreateDebugUtilsMessengerEXT");
    if (!CreateDebugUtilsMessengerEXT) {
        std::wcout << std::format(L"Failed to load extension function vkCreateDebugUtilsMessengerEXT\n");
        return;
    }

    auto DestroyDebugUtilsMessengerEXT = (PFN_vkDestroyDebugUtilsMessengerEXT)vkGetInstanceProcAddr(
        instance->get(), "vkDestroyDebugUtilsMessengerEXT");
    if (!DestroyDebugUtilsMessengerEXT) {
        std::wcout << std::format(L"Failed to load extension function vkDestroyDebugUtilsMessengerEXT\n");
        return;
    }

    auto DestroyDebug = [pinstance=&instance, allocator_fn=nullptr, DestroyDebugUtilsMessengerEXT](auto ptr) {
        DestroyDebugUtilsMessengerEXT(pinstance->get(), ptr, allocator_fn);
    };
    VkDebugUtilsMessengerEXT messenger_raw = nullptr;
    if (CreateDebugUtilsMessengerEXT(instance->get(), &debug_create_info, nullptr, &messenger_raw) != VK_SUCCESS) {
        std::wcout << std::format(L"Failed to initialize debug utilities.\n");
        return;
    }
    std::unique_ptr<VkDebugUtilsMessengerEXT_T, decltype(DestroyDebug)> messenger{messenger_raw, DestroyDebug};

    // Create surface; must happen after instance creation, but before examining/creating devices.
    VkWin32SurfaceCreateInfoKHR surface_create_info{
        .sType=VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR,
        .pNext=nullptr,
        .flags=0, // reserved
        .hinstance=sys_window.GetInstance(),
        .hwnd=sys_window.GetHWND()
    };

    auto DestroySurface = [vkinstance=instance->get(), allocator_fn=nullptr](auto pptr) {
        vkDestroySurfaceKHR(vkinstance, *pptr, allocator_fn);
    };
    VkSurfaceKHR surface_raw = nullptr;
    if (vkCreateWin32SurfaceKHR(instance->get(), &surface_create_info, nullptr, &surface_raw) != VK_SUCCESS) {
        std::wcout << std::format(L"Failed to create win32 surface.\n");
        return;
    }
    std::unique_ptr<VkSurfaceKHR, decltype(DestroySurface)> surface{&surface_raw, DestroySurface};

    // Physical devices
    auto devices_result = Extract<VkPhysicalDevice>(vkEnumeratePhysicalDevices, instance->get());
    if (devices_result.extraction_result != VK_SUCCESS && devices_result.extraction_result != VK_INCOMPLETE) {
        std::wcout << std::format(L"Failed to retrieve instance devices properties.\n");
        return;
    }
    auto devices{std::move(devices_result.values)};

    auto dev_group_results = Extract<VkPhysicalDeviceGroupProperties>(vkEnumeratePhysicalDeviceGroups, instance->get());
    if (dev_group_results.extraction_result != VK_SUCCESS && dev_group_results.extraction_result != VK_INCOMPLETE) {
        std::wcout << std::format(L"Failed to enumerate physical device groups.\n");
        return;
    }
    auto device_group_properties{std::move(dev_group_results.values)};

    std::vector<VkPhysicalDeviceProperties> devices_properties(devices.size());
    for (auto i : std::views::iota(static_cast<size_t>(0), devices.size())) {
        vkGetPhysicalDeviceProperties(devices[i], &devices_properties[i]);
    }
    std::vector<VkPhysicalDeviceFeatures> devices_features(devices.size());
    for (auto i : std::views::iota(static_cast<size_t>(0), devices.size())) {
        vkGetPhysicalDeviceFeatures(devices[i], &devices_features[i]);
    }
    std::vector<std::vector<const char*>> device_extension_names{};
    std::vector<std::vector<VkExtensionProperties>> device_extension_props{};
    std::vector<std::vector<VkQueueFamilyProperties>> device_queue_family_props{};
    device_extension_names.reserve(devices.size());
    device_extension_props.reserve(devices.size());
    device_queue_family_props.reserve(devices.size());
    for (auto device : devices) {
        auto dev_ext_props_results = Extract<VkExtensionProperties>(vkEnumerateDeviceExtensionProperties,
                                                                    device, nullptr);
        if (dev_ext_props_results.extraction_result != VK_SUCCESS) {
            std::wcout << std::format(L"Failed to enumerate physical device queue families.\n");
            return;
        }
        device_extension_props.emplace_back(std::move(dev_ext_props_results.values));
        std::vector<const char*> ext_names{};
        ext_names.reserve(dev_ext_props_results.values.size());
        std::ranges::transform(dev_ext_props_results.values, std::back_inserter(ext_names),
                               [](auto& i) { return i.extensionName; });
        // without this addition; creating the swapchain aborts with no error
        ext_names.push_back(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
        device_extension_names.emplace_back(std::move(ext_names));

        auto dev_queue_family_results = Extract<VkQueueFamilyProperties>(
            []<typename... Args>(Args... args) {
                vkGetPhysicalDeviceQueueFamilyProperties(std::forward<Args>(args)...);
                return VK_SUCCESS;
            }, device);
        if (dev_queue_family_results.extraction_result != VK_SUCCESS) {
            std::wcout << std::format(L"Failed to enumerate physical device queue families.\n");
            return;
        }
        device_queue_family_props.emplace_back(std::move(dev_queue_family_results.values));
    }

    uint32_t queue_family_index = 0;

    VkBool32 present_support = false;
    vkGetPhysicalDeviceSurfaceSupportKHR(devices[0], queue_family_index, *surface, &present_support);

    // Create logical device
    float queue_priority = 1.0f;
    std::vector<VkDeviceQueueCreateInfo> queue_create_infos{
        // graphics queue + presentation queue
        {
            .sType=VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO,
            .pNext=nullptr,
            .flags=0,
            .queueFamilyIndex=queue_family_index,
            .queueCount=2,
            .pQueuePriorities=&queue_priority
        }
    };
    VkDeviceCreateInfo device_create_info{
        .sType=VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO,
        .pNext=nullptr,
        .flags=0,
        .queueCreateInfoCount=static_cast<uint32_t>(queue_create_infos.size()),
        .pQueueCreateInfos=queue_create_infos.data(),
        .enabledLayerCount=(enable_validation) ? static_cast<uint32_t>(validation_layers.size()) : 0,
        .ppEnabledLayerNames=(enable_validation) ? validation_layers.data() : nullptr,
        .enabledExtensionCount=static_cast<uint32_t>(device_extension_names[0].size()),
        .ppEnabledExtensionNames=device_extension_names[0].data(),
        .pEnabledFeatures=&devices_features[0]
    };

    auto DestroyDevice = [allocator_fn=nullptr](auto ptr) { vkDestroyDevice(ptr, allocator_fn); };
    VkDevice device_raw = nullptr;
    if (vkCreateDevice(devices[0], &device_create_info, nullptr, &device_raw) != VK_SUCCESS) {
        std::wcout << std::format(L"Failed to create logical device.\n");
        return;
    }
    std::unique_ptr<VkDevice_T, decltype(DestroyDevice)> device{device_raw, DestroyDevice};

    VkQueue graphics_queue = nullptr;
    VkQueue present_queue = nullptr;
    vkGetDeviceQueue(device->get(), 0, 0, &graphics_queue);
    vkGetDeviceQueue(device->get(), 0, 1, &present_queue);

    VkSurfaceCapabilitiesKHR surface_caps{};
    if (vkGetPhysicalDeviceSurfaceCapabilitiesKHR(devices[0], *surface, &surface_caps) != VK_SUCCESS) {
        std::wcout << std::format(L"Failed to extract surface capabilities for physical device.\n");
        return;
    }

    auto s_formats_result = Extract<VkSurfaceFormatKHR>(vkGetPhysicalDeviceSurfaceFormatsKHR, devices[0], *surface);
    if (s_formats_result.extraction_result != VK_SUCCESS) {
        std::wcout << std::format(L"Failed to retrieve surface formats for physical device.\n");
        return;
    }
    auto surface_formats{std::move(s_formats_result.values)};
    VkSurfaceFormatKHR surface_format_khr{surface_formats[0]};
    bool has_expected_format = false;
    for (auto& surface_format : surface_formats) {
        if (surface_format.format == VK_FORMAT_B8G8R8A8_SRGB &&
            surface_format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
            has_expected_format = true;
            surface_format_khr = surface_format;
        }
    }

    auto sp_mode_result = Extract<VkPresentModeKHR>(vkGetPhysicalDeviceSurfacePresentModesKHR, devices[0], *surface);
    if (sp_mode_result.extraction_result != VK_SUCCESS) {
        std::wcout << std::format(L"Failed to retrieve surface present mdoes for physical device.\n");
        return;
    }
    auto surface_present_modes{std::move(sp_mode_result.values)};
    auto present_mode = VK_PRESENT_MODE_FIFO_KHR;
    for (auto& sp_mode : surface_present_modes) {
        if (sp_mode == VK_PRESENT_MODE_MAILBOX_KHR) {
            present_mode = VK_PRESENT_MODE_MAILBOX_KHR;
            break;
        }
    }

    VkExtent2D extent{surface_caps.currentExtent};
    if (surface_caps.currentExtent.width == std::numeric_limits<uint32_t>::max()) {
        std::array<int, 2> client_dims = sys_window.GetDims();
        if (client_dims[0] == 0) {
            std::wcout << std::format(L"Failed to get window dimensions.\n");
            return;
        }
        extent.width = std::clamp(static_cast<uint32_t>(client_dims[0]),
                                  surface_caps.minImageExtent.width,
                                  surface_caps.maxImageExtent.width);
        extent.height = std::clamp(static_cast<uint32_t>(client_dims[1]),
                                   surface_caps.minImageExtent.height,
                                   surface_caps.maxImageExtent.height);
    }
    uint32_t image_count = surface_caps.minImageCount + 1;
    if (surface_caps.maxImageCount > 0 && image_count > surface_caps.maxImageCount) {
        image_count = surface_caps.maxImageCount;
    }
    VkFormat swapchain_format = surface_format_khr.format;
    VkColorSpaceKHR swapchain_color_space = surface_format_khr.colorSpace;
    VkExtent2D swapchain_extent = extent;

    VkSwapchainCreateInfoKHR swapchain_create_info{
        .sType=VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR,
        .pNext=nullptr,
        .flags=0,
        .surface=*surface,
        .minImageCount=image_count,
        .imageFormat=swapchain_format,
        .imageColorSpace=swapchain_color_space,
        .imageExtent=swapchain_extent,
        .imageArrayLayers=1, // mono
        .imageUsage=VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
        .imageSharingMode=VK_SHARING_MODE_EXCLUSIVE,
        .queueFamilyIndexCount=0,
        .pQueueFamilyIndices=nullptr,
        .preTransform=surface_caps.currentTransform,
        .compositeAlpha=VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR,
        .presentMode=present_mode,
        .clipped=VK_TRUE,
        .oldSwapchain=VK_NULL_HANDLE
    };
    auto DestroySwapchain = [pdevice=device->get(), allocator_fn=nullptr](auto pptr) {
        vkDestroySwapchainKHR(pdevice, *pptr, allocator_fn);
    };
    VkSwapchainKHR swapchain_raw = nullptr;
    if (vkCreateSwapchainKHR(device->get(), &swapchain_create_info, nullptr, &swapchain_raw) != VK_SUCCESS) {
        std::wcout << std::format(L"Failed to create swapchain.\n");
        return;
    }
    std::unique_ptr<VkSwapchainKHR, decltype(DestroySwapchain)> swapchain{&swapchain_raw, DestroySwapchain};

    // Grab swap chain images
    auto sc_images_result = Extract<VkImage>(vkGetSwapchainImagesKHR, device->get(), *swapchain);
    if (sc_images_result.extraction_result != VK_SUCCESS) {
        std::wcout << "Failed to retrieve swapchain images." << std::endl;
        return;
    }
    std::vector<VkImage> swapchain_images{std::move(sc_images_result.values)};

    auto DestroyImageView = [pdevice=device->get(), allocator_fn=nullptr](auto pptr) {
        vkDestroyImageView(pdevice, *pptr, allocator_fn);
    };
    using UniqueImageView = std::unique_ptr<VkImageView, decltype(DestroyImageView)>;
    std::vector<UniqueImageView> swapchain_image_views{};
    swapchain_image_views.reserve(swapchain_images.size());
    for (auto i : std::views::iota(static_cast<size_t>(0), swapchain_images.size())) {
        VkImageViewCreateInfo create_info{
            .sType=VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
            .pNext=nullptr,
            .image=swapchain_images[i],
            .viewType=VK_IMAGE_VIEW_TYPE_2D,
            .format=swapchain_format,
            .components={
                .r = VK_COMPONENT_SWIZZLE_IDENTITY,
                .g = VK_COMPONENT_SWIZZLE_IDENTITY,
                .b = VK_COMPONENT_SWIZZLE_IDENTITY,
                .a = VK_COMPONENT_SWIZZLE_IDENTITY
            },
            .subresourceRange={
                .aspectMask=VK_IMAGE_ASPECT_COLOR_BIT,
                .baseMipLevel=0,
                .levelCount=1,
                .baseArrayLayer=0,
                .layerCount=1
            }
        };
        VkImageView image_view_raw = nullptr;
        if (vkCreateImageView(device->get(), &create_info, nullptr, &image_view_raw) != VK_SUCCESS) {
            std::wcout << std::format(L"Failed to create image view.\n");
            return;
        }
        swapchain_image_views.emplace_back(&image_view_raw, DestroyImageView);
    }

    // Display info
    std::wcout << std::format(L"Vulkan version: {} - {}\n", api_version, VK_MAKE_API_VERSION(0, 1, 3, 231));
    std::wcout << std::format(L"Number of extensions: {}\n", extension_names.size());
    for (auto i : extension_names) {
        std::cout << std::format("Extension: {}\n", i);
    }
    std::wcout << std::format(L"Number of devices found: {}\n", devices.size());
    std::wcout << std::format(L"Number of device groups: {}\n", device_group_properties.size());
    for (auto& group : device_group_properties) {
        std::wcout << std::format(L"Group size: {}\n", group.physicalDeviceCount);
    }
    for (auto& device_props : devices_properties) {
        std::cout << std::format("Device: {} -- {}\n", device_props.deviceName, device_props.apiVersion);
    }
    for (auto& device_feats : devices_features) {
        std::cout << std::format("multiViewport: {}\n", device_feats.multiViewport);
    }
    for (auto i : std::views::iota(static_cast<size_t>(0), devices.size())) {
        std::wcout << std::format(L"Queue family for device {}-\n", i);
        for (auto& q_fam_props : device_queue_family_props[i]) {
            std::wcout << std::format(L"{} -- {}\n", q_fam_props.queueFlags, q_fam_props.queueCount);
        }
    }
#if 0
    for (auto i : std::views::iota(static_cast<size_t>(0), devices.size())) {
        std::wcout << std::format(L"Extensions for device {}-\n", i);
        bool found = false;
        for (auto& dev_ext : device_extension_props[i]) {
            std::cout << dev_ext.extensionName << std::endl;
            if (std::string{dev_ext.extensionName} == std::string{VK_KHR_SWAPCHAIN_EXTENSION_NAME}) {
                found = true;
            }
        }
        std::cout << std::endl;
        std::wcout << std::format(L"Has swapchain ext: {}\n", found);
        std::cout << std::endl;
    }
#endif
    std::wcout << std::format(L"Has presentation support ... {}\n", present_support);
    std::wcout << std::endl;
    std::wcout << std::format(L"Format: {} -- PresentMode: {}\n", has_expected_format, static_cast<uint32_t>(present_mode));
    std::wcout << std::format(L"Extent: {}, {}\n", extent.width, extent.height);




    // Shaders
    std::vector<uint32_t> frag_shader_code = LoadShader("shaders/shader.frag.spv");
    if (frag_shader_code.empty()) {
        std::wcout << std::format(L"Failed to open fragment shader.\n");
        return;
    }
    std::vector<uint32_t> vert_shader_code = LoadShader("shaders/shader.vert.spv");
    if (vert_shader_code.empty()) {
        std::wcout << std::format(L"Failed to open vertex shader.\n");
        return;
    }

    auto DestroyShaderModule = [pdevice=device->get(), allocator_fn=nullptr](auto pptr) {
        vkDestroyShaderModule(pdevice, *pptr, allocator_fn);
    };

    VkShaderModuleCreateInfo frag_shader_create_info{
        .sType=VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO,
        .pNext=nullptr,
        .flags=0,
        .codeSize=frag_shader_code.size()*sizeof(uint32_t),
        .pCode=frag_shader_code.data()
    };
    VkShaderModule frag_shader_raw = nullptr;
    if (vkCreateShaderModule(device->get(), &frag_shader_create_info, nullptr, &frag_shader_raw) != VK_SUCCESS) {
        std::wcout << std::format(L"Failed to create fragment shader module.\n");
        return;
    }
    std::unique_ptr<VkShaderModule, decltype(DestroyShaderModule)> frag_shader{&frag_shader_raw, DestroyShaderModule};

    VkShaderModuleCreateInfo vert_shader_create_info{
        .sType=VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO,
        .pNext=nullptr,
        .flags=0,
        .codeSize=vert_shader_code.size()*sizeof(uint32_t),
        .pCode=vert_shader_code.data()
    };
    VkShaderModule vert_shader_raw = nullptr;
    if (vkCreateShaderModule(device->get(), &vert_shader_create_info, nullptr, &vert_shader_raw) != VK_SUCCESS) {
        std::wcout << std::format(L"Failed to create vertex shader module.\n");
        return;
    }
    std::unique_ptr<VkShaderModule, decltype(DestroyShaderModule)> vert_shader{&vert_shader_raw, DestroyShaderModule};

    // Create RenderPass
    VkAttachmentDescription color_attachment{
        .format=swapchain_format,
        .samples=VK_SAMPLE_COUNT_1_BIT,
        .loadOp=VK_ATTACHMENT_LOAD_OP_CLEAR,
        .storeOp=VK_ATTACHMENT_STORE_OP_STORE,
        .stencilLoadOp=VK_ATTACHMENT_LOAD_OP_DONT_CARE,
        .stencilStoreOp=VK_ATTACHMENT_STORE_OP_DONT_CARE,
        .initialLayout=VK_IMAGE_LAYOUT_UNDEFINED,
        .finalLayout=VK_IMAGE_LAYOUT_PRESENT_SRC_KHR
    };

    VkAttachmentReference color_attachment_reference{
        .attachment=0,
        .layout=VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL
    };

    VkSubpassDescription subpass_desc{
        .pipelineBindPoint=VK_PIPELINE_BIND_POINT_GRAPHICS,
        .inputAttachmentCount=0,
        .pInputAttachments=nullptr,
        .colorAttachmentCount=1,
        .pColorAttachments=&color_attachment_reference,
        .pResolveAttachments=nullptr,
        .pDepthStencilAttachment=nullptr,
        .preserveAttachmentCount=0,
        .pPreserveAttachments=nullptr
    };

    VkRenderPassCreateInfo render_pass_create_info{
        .sType=VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO,
        .pNext=nullptr,
        .attachmentCount=1,
        .pAttachments=&color_attachment,
        .subpassCount=1,
        .pSubpasses=&subpass_desc
    };
    auto DestroyRenderPass = [pdevice=device->get(), allocate_fn=nullptr](auto pptr) {
        vkDestroyRenderPass(pdevice, *pptr, allocate_fn);
    };
    VkRenderPass render_pass_raw = nullptr;
    if (vkCreateRenderPass(device->get(), &render_pass_create_info, nullptr, &render_pass_raw) != VK_SUCCESS) {
        std::wcout << std::format(L"Failed to create render pass.\n");
        return;
    }
    std::unique_ptr<VkRenderPass, decltype(DestroyRenderPass)> render_pass{&render_pass_raw, DestroyRenderPass};

    // Create pipeline
    std::vector<VkPipelineShaderStageCreateInfo> shader_stages{
        {
            .sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
            .pNext=nullptr,
            .stage=VK_SHADER_STAGE_VERTEX_BIT,
            .module=*vert_shader,
            .pName="main"
        },
        {
            .sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
            .pNext=nullptr,
            .stage=VK_SHADER_STAGE_FRAGMENT_BIT,
            .module=*frag_shader,
            .pName="main"
        }
    };

    std::vector<VkDynamicState> dynamic_states = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};

    VkPipelineDynamicStateCreateInfo dynamic_state{
        .sType=VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO,
        .pNext=nullptr,
        .dynamicStateCount=static_cast<uint32_t>(dynamic_states.size()),
        .pDynamicStates=dynamic_states.data()
    };

    VkPipelineVertexInputStateCreateInfo vertex_create_info{
        .sType=VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO,
        .pNext=nullptr,
        .vertexBindingDescriptionCount=0,
        .pVertexBindingDescriptions=nullptr,
        .vertexAttributeDescriptionCount=0,
        .pVertexAttributeDescriptions=nullptr
    };

    VkPipelineInputAssemblyStateCreateInfo input_assembly_create_info{
        .sType=VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO,
        .pNext=nullptr,
        .topology=VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST,
        .primitiveRestartEnable=VK_FALSE
    };

    VkViewport viewport{
        .x=0.0f,
        .y=0.0f,
        .width=static_cast<float>(swapchain_extent.width),
        .height=static_cast<float>(swapchain_extent.height),
        .minDepth=0.0f,
        .maxDepth=1.0f
    };

    VkRect2D scissor{
        .offset={0, 0},
        .extent=swapchain_extent
    };

    VkPipelineViewportStateCreateInfo viewport_create_info{
        .sType=VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO,
        .pNext=nullptr,
        .viewportCount=1,
        .pViewports=&viewport,
        .scissorCount=1,
        .pScissors=&scissor
    };

    VkPipelineRasterizationStateCreateInfo rasterizer_create_info{
        .sType=VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,
        .pNext=nullptr,
        .depthClampEnable=VK_FALSE,
        .rasterizerDiscardEnable=VK_FALSE,
        .polygonMode=VK_POLYGON_MODE_FILL,
        .cullMode=VK_CULL_MODE_BACK_BIT,
        .frontFace=VK_FRONT_FACE_CLOCKWISE,
        .depthBiasEnable=VK_FALSE,
        .depthBiasConstantFactor=0.0f,
        .depthBiasClamp=0.0f,
        .depthBiasSlopeFactor=0.0f,
        .lineWidth=1.0f
    };

    VkPipelineMultisampleStateCreateInfo multisampling_create_info{
        .sType=VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO,
        .pNext=nullptr,
        .rasterizationSamples=VK_SAMPLE_COUNT_1_BIT,
        .sampleShadingEnable=VK_FALSE,
        .minSampleShading=1.0f,
        .pSampleMask=nullptr,
        .alphaToCoverageEnable=VK_FALSE,
        .alphaToOneEnable=VK_FALSE
    };

    VkPipelineColorBlendAttachmentState color_blend_attachment{
        .blendEnable=VK_FALSE,
        .srcColorBlendFactor=VK_BLEND_FACTOR_ONE,
        .dstColorBlendFactor=VK_BLEND_FACTOR_ZERO,
        .colorBlendOp=VK_BLEND_OP_ADD,
        .srcAlphaBlendFactor=VK_BLEND_FACTOR_ONE,
        .dstAlphaBlendFactor=VK_BLEND_FACTOR_ZERO,
        .alphaBlendOp=VK_BLEND_OP_ADD,
        .colorWriteMask=VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT |
                        VK_COLOR_COMPONENT_A_BIT
    };

    VkPipelineColorBlendStateCreateInfo color_blend_create_info{
        .sType=VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO,
        .pNext=nullptr,
        .logicOpEnable=VK_FALSE,
        .logicOp=VK_LOGIC_OP_COPY,
        .attachmentCount=1,
        .pAttachments=&color_blend_attachment,
        .blendConstants={0.0f, 0.0f, 0.0f, 0.0f}
    };

    VkPipelineLayoutCreateInfo pipeline_layout_create_info{
        .sType=VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO,
        .pNext=nullptr,
        .setLayoutCount=0,
        .pSetLayouts=nullptr,
        .pushConstantRangeCount=0,
        .pPushConstantRanges=nullptr
    };
    auto DestroyPipelineLayout = [pdevice=device->get(), allocate_fn=nullptr](auto pptr) {
        vkDestroyPipelineLayout(pdevice, *pptr, allocate_fn);
    };
    VkPipelineLayout pipeline_layout_raw = nullptr;
    if (vkCreatePipelineLayout(device->get(), &pipeline_layout_create_info, nullptr, &pipeline_layout_raw) != VK_SUCCESS) {
        std::wcout << std::format(L"Failed to create pipeline layout.\n");
        return;
    }
    std::unique_ptr<VkPipelineLayout, decltype(DestroyPipelineLayout)> pipeline_layout{&pipeline_layout_raw, DestroyPipelineLayout};

    VkGraphicsPipelineCreateInfo pipeline_create_info{
        .sType=VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO,
        .pNext=nullptr,
        .stageCount=static_cast<uint32_t>(shader_stages.size()),
        .pStages=shader_stages.data(),
        .pVertexInputState=&vertex_create_info,
        .pInputAssemblyState=&input_assembly_create_info,
        .pViewportState=&viewport_create_info,
        .pRasterizationState=&rasterizer_create_info,
        .pMultisampleState=&multisampling_create_info,
        .pDepthStencilState=nullptr,
        .pColorBlendState=&color_blend_create_info,
        .pDynamicState=&dynamic_state,
        .layout=*pipeline_layout,
        .renderPass=*render_pass,
        .subpass=0,
        .basePipelineHandle=VK_NULL_HANDLE,
        .basePipelineIndex=-1
    };
    auto DestroyGraphicsPipeline = [pdevice=device->get(), allocate_fn=nullptr](auto pptr) {
        vkDestroyPipeline(pdevice, *pptr, allocate_fn);
    };
    VkPipeline graphics_pipeline_raw = nullptr;
    if (vkCreateGraphicsPipelines(device->get(), VK_NULL_HANDLE, 1, &pipeline_create_info, nullptr, &graphics_pipeline_raw) != VK_SUCCESS) {
        std::wcout << std::format(L"Failed to create the graphics pipeline.\n");
        return;
    }
    std::unique_ptr<VkPipeline, decltype(DestroyGraphicsPipeline)> graphics_pipeline{&graphics_pipeline_raw, DestroyGraphicsPipeline};
}
