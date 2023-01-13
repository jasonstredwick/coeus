#if 0
#include <windows.h>
#include <vector>
#include <iostream>
#include "jms/vulkan/state.hpp"


struct cMonitorsVec {
    std::vector<int> iMonitors;
    std::vector<HMONITOR> hMonitors;
    std::vector<HDC> hdcMonitors;
    std::vector<RECT> rcMonitors;
    static BOOL CALLBACK MonitorEnum(HMONITOR hMon, HDC hdc, LPRECT lprcMonitor, LPARAM pData) {
        cMonitorsVec* pThis = reinterpret_cast<cMonitorsVec*>(pData);
        pThis->hMonitors.push_back(hMon);
        pThis->hdcMonitors.push_back(hdc);
        pThis->rcMonitors.push_back(*lprcMonitor);
        pThis->iMonitors.push_back(pThis->hdcMonitors.size());
        return TRUE;
    }
    cMonitorsVec() {
        EnumDisplayMonitors(0, 0, MonitorEnum, (LPARAM)this);
    }
};


#include <format>
int main() {
        if (!SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2)) {
            auto error = GetLastError();
            if (error != ERROR_ACCESS_DENIED) { // ERROR_ACCESS_DENIED == already set; ignore error
                throw std::runtime_error(std::format("Failed to set dpi awareness: {}\n", error));
            }
        }
    cMonitorsVec monitors{};
    for (int i=0; i<monitors.iMonitors.size(); ++i) {
        std::wcout << "Screen id: " << i << std::endl;
        std::wcout << "---------------------------------------------------" << std::endl;
        std::wcout << " - screen left top corner coords: (" << monitors.rcMonitors[i].left << ", " << monitors.rcMonitors[i].top << ")" << std::endl;
        std::wcout << " - screen bottom right corner coords: (" << monitors.rcMonitors[i].right << ", " << monitors.rcMonitors[i].bottom << ")" << std::endl;
        std::wcout << " - screen dims: (" << (monitors.rcMonitors[i].right - monitors.rcMonitors[i].left) << ", " <<
                                             (monitors.rcMonitors[i].bottom - monitors.rcMonitors[i].top) << ")" << std::endl;
        std::wcout << "---------------------------------------------------" << std::endl;
    }
    jms::vulkan::State vulkan_state{};
    std::vector<vk::ExtensionProperties> extension_props = vulkan_state.context.enumerateInstanceExtensionProperties();
    for (auto& i : extension_props) {
        std::cout << i.extensionName << std::endl;
    }
    return 0;
}
#endif


#if 1
#include <array>
#include <cstddef>
#include <exception>
#include <expected>
#include <format>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "jms/vulkan/include_config.hpp"
#include <vulkan/vulkan_raii.hpp>
#include <GLFW/glfw3.h>

#include "jms/vulkan/render_info.hpp"
#include "jms/vulkan/state.hpp"

#if defined(_WIN32) || defined(_WIN64)
//#include "jms/sys_window/win.hpp"
#endif


vk::raii::SurfaceKHR CreateDisplaySurface(const vk::raii::Instance& instance, const vk::raii::PhysicalDevice& physical_device);
void DrawFrame(const jms::vulkan::State& vulkan_state, const vk::raii::Buffer& vertex_buffer, const std::vector<Vertex>& vertices);


struct GLFWState {
    std::unique_ptr<GLFWwindow, decltype(&glfwDestroyWindow)> window{nullptr, &glfwDestroyWindow};

    GLFWState() { if (!glfwInit()) { throw std::runtime_error("Failed to initialize GLFW."); } }
    GLFWState(const GLFWState&) = delete;
    GLFWState(GLFWState&&) = default;
    ~GLFWState() { glfwTerminate(); }
    GLFWState& operator=(const GLFWState&) = delete;
    GLFWState& operator=(GLFWState&&) = default;

    void DefaultCreate(int width, int height, GLFWmonitor* monitor=nullptr) {
        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
        glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
        glfwWindowHint(GLFW_DECORATED, GLFW_FALSE);
        window.reset(glfwCreateWindow(width, height, "", monitor, nullptr));
    }

    std::vector<std::string> GetInstanceExtensions() {
        uint32_t count = 0;
        const char** exts = glfwGetRequiredInstanceExtensions(&count);
        std::vector<std::string> out{};
        for (int i=0; i< count; ++i) {
            out.push_back(std::string{exts[i]});
        }
        return out;
    }

    vk::raii::SurfaceKHR CreateSurface(const vk::raii::Instance& instance, const VkAllocationCallbacks* allocator=nullptr) {
        VkSurfaceKHR surface = nullptr;
        glfwCreateWindowSurface(*instance, window.get(), allocator, &surface);
        return vk::raii::SurfaceKHR{instance, surface, *allocator};
    }
};


int main(char** argv, int argc) {
    char a = 0;
    std::cout << std::format("Start\n");

    try {
        //jms::SysWindow sys_window = jms::SysWindow::Create({});

        // Vertex buffer
        std::vector<Vertex> vertices = {
            {{ 0.0f,  0.0f}, {1.0f, 0.0f, 0.0f}},
            {{ 0.5f,  0.0f}, {0.0f, 1.0f, 0.0f}},
            {{ 0.5f,  0.5f}, {0.0f, 0.0f, 1.0f}},
            {{ 0.5f,  0.5f}, {0.0f, 0.0f, 1.0f}},
            {{ 0.0f,  0.5f}, {0.0f, 0.0f, 1.0f}},
            {{ 0.0f,  0.0f}, {1.0f, 0.0f, 1.0f}}
        };
        size_t size_in_bytes = sizeof(Vertex) * vertices.size();

        jms::vulkan::State vulkan_state{};

        GLFWState glfw_state{};
        glfw_state.DefaultCreate(1024, 768);
        std::vector<std::string> window_instance_extensions= glfw_state.GetInstanceExtensions();

        std::vector<std::string> required_instance_extensions{
            /* VK_KHR_display
            std::string{VK_KHR_DISPLAY_EXTENSION_NAME},
            std::string{VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME},
            std::string{VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME}
            */
        };
        std::set<std::string> instance_extensions_set{window_instance_extensions.begin(),
                                                      window_instance_extensions.end()};
        for (auto& i : required_instance_extensions) { instance_extensions_set.insert(i); }
        std::vector<std::string> instance_extensions{instance_extensions_set.begin(), instance_extensions_set.end()};

        vulkan_state.InitInstance(jms::vulkan::InstanceConfig{
            .app_name=std::string{"tut2"},
            .engine_name=std::string{"tut2.e"},
            .layer_names={
                std::string{"VK_LAYER_KHRONOS_synchronization2"},
                std::string{"VK_LAYER_KHRONOS_validation"}
            },
            .extension_names=instance_extensions
        });

        // Create surface; must happen after instance creation, but before examining/creating devices.
        // will be moved from
        vulkan_state.surface = glfw_state.CreateSurface(*vulkan_state.instance);

        vk::raii::PhysicalDevice& physical_device = vulkan_state.physical_devices.at(0);
        //vulkan_state.surface = CreateDisplaySurface(vulkan_state.instance, physical_device);
        auto dims = sys_window.GetDims();
        auto width = dims[0];
        auto height = dims[1];

        vulkan_state.render_info = jms::vulkan::SurfaceInfo(vulkan_state.surface,
                                                            physical_device,
                                                            static_cast<uint32_t>(width),
                                                            static_cast<uint32_t>(height));
        uint32_t queue_family_index = 0;
        std::vector<float> queue_priority(2, 1.0f); // graphics + presentation
        vulkan_state.InitDevice(physical_device, jms::vulkan::DeviceConfig {
            //.layer_names={
            //    std::string{"VK_LAYER_KHRONOS_validation"} // deprecated; but may still need for validation?
            //},
            .extension_names={
                std::string{VK_KHR_SWAPCHAIN_EXTENSION_NAME}//,
                //std::string{"VK_KHR_portability_subset"}  // VK_KHR_display
            },
            //.features=physical_device.getFeatures(),
            .queue_infos=std::move(std::vector<vk::DeviceQueueCreateInfo>{
                // graphics queue + presentation queue
                {
                    .queueFamilyIndex=queue_family_index,
                    .queueCount=static_cast<uint32_t>(queue_priority.size()),
                    .pQueuePriorities=queue_priority.data()
                }
            })
        });
        vulkan_state.InitRenderPass(vulkan_state.devices.at(0), vulkan_state.render_info.format, vulkan_state.render_info.extent);
        vulkan_state.InitPipeline(vulkan_state.devices.at(0), vulkan_state.render_passes.at(0), vulkan_state.render_info.extent);
        vulkan_state.InitQueues(vulkan_state.devices.at(0), queue_family_index);
        vulkan_state.InitSwapchain(vulkan_state.devices.at(0), vulkan_state.render_info, vulkan_state.surface, vulkan_state.render_passes.at(0));
        const vk::raii::Semaphore& image_available_semaphore = vulkan_state.semaphores.at(0);
        const vk::raii::Semaphore& render_finished_semaphore = vulkan_state.semaphores.at(0);
        const vk::raii::Fence& in_flight_fence = vulkan_state.fences.at(0);

        vulkan_state.InitMemory(size_in_bytes, vulkan_state.physical_devices.at(0), vulkan_state.devices.at(0));
        const vk::raii::DeviceMemory& device_memory = vulkan_state.device_memory.at(0);
        const vk::MemoryRequirements& buffers_mem_reqs = vulkan_state.buffers_mem_reqs.at(0);
        void* data = device_memory.mapMemory(0, buffers_mem_reqs.size);
        std::memcpy(data, vertices.data(), size_in_bytes);
        device_memory.unmapMemory();
        vulkan_state.buffers.at(0).bindMemory(*device_memory, 0);

        DrawFrame(vulkan_state, vulkan_state.buffers.at(0), vertices);
        vulkan_state.devices.at(0).waitIdle();

        std::cout << "---------------------\n";
        while (sys_window.ProcessEvents()) { ; }
        //std::cin >> a;
    } catch (std::exception const& exp) {
        std::cout << "Exception caught\n" << exp.what() << std::endl;
    }

    glfwTerminate();

    std::cout << std::format("End\n");
    return 0;
}


vk::raii::SurfaceKHR CreateDisplaySurface(const vk::raii::Instance& instance,
                                          const vk::raii::PhysicalDevice& physical_device) {
    auto display_properties_return = physical_device.getDisplayPropertiesKHR();
    if (!display_properties_return.size()) {
        throw std::runtime_error("No non-managed displays available.");
    }
    auto display = display_properties_return.at(0).display;
    auto display_mode_props_return = (*physical_device).getDisplayModePropertiesKHR(display);

    if (display_mode_props_return.size() == 0) {
        printf("Cannot find any mode for the display!\n");
        fflush(stdout);
        exit(1);
    }
    auto display_mode_prop = display_mode_props_return.at(0);

    // Get the list of planes
    auto display_plane_props = physical_device.getDisplayPlanePropertiesKHR();
    if (display_plane_props.size() == 0) {
        printf("Cannot find any plane!\n");
        fflush(stdout);
        exit(1);
    }

    vk::Bool32 found_plane = VK_FALSE;
    uint32_t plane_found = 0;
    // Find a plane compatible with the display
    for (uint32_t plane_index = 0; plane_index < display_plane_props.size(); plane_index++) {
        // Disqualify planes that are bound to a different display
        if (display_plane_props[plane_index].currentDisplay && (display_plane_props[plane_index].currentDisplay != display)) {
            continue;
        }

        auto display_plane_supported_displays_return = physical_device.getDisplayPlaneSupportedDisplaysKHR(plane_index);
        if (display_plane_supported_displays_return.size() == 0) {
            continue;
        }

        for (const auto &supported_display : display_plane_supported_displays_return) {
            if (*supported_display == display) {
                found_plane = VK_TRUE;
                plane_found = plane_index;
                break;
            }
        }

        if (found_plane) {
            break;
        }
    }

    if (!found_plane) {
        printf("Cannot find a plane compatible with the display!\n");
        fflush(stdout);
        exit(1);
    }

    vk::DisplayPlaneCapabilitiesKHR planeCaps =
        (*physical_device).getDisplayPlaneCapabilitiesKHR(display_mode_prop.displayMode, plane_found);
    // Find a supported alpha mode
    vk::DisplayPlaneAlphaFlagBitsKHR alphaMode = vk::DisplayPlaneAlphaFlagBitsKHR::eOpaque;
    std::array<vk::DisplayPlaneAlphaFlagBitsKHR, 4> alphaModes = {
        vk::DisplayPlaneAlphaFlagBitsKHR::eOpaque,
        vk::DisplayPlaneAlphaFlagBitsKHR::eGlobal,
        vk::DisplayPlaneAlphaFlagBitsKHR::ePerPixel,
        vk::DisplayPlaneAlphaFlagBitsKHR::ePerPixelPremultiplied,
    };
    for (const auto &alpha_mode : alphaModes) {
        if (planeCaps.supportedAlpha & alpha_mode) {
            alphaMode = alpha_mode;
            break;
        }
    }

    vk::Extent2D image_extent{
        .width=display_mode_prop.parameters.visibleRegion.width,
        .height=display_mode_prop.parameters.visibleRegion.height
    };

    vk::raii::SurfaceKHR surface{instance, vk::DisplaySurfaceCreateInfoKHR{
        .displayMode=display_mode_prop.displayMode,
        .planeIndex=plane_found,
        .planeStackIndex=display_plane_props[plane_found].currentStackIndex,
        .globalAlpha=1.0f,
        .alphaMode=alphaMode,
        .imageExtent=image_extent
    }};

    return surface;
}


void DrawFrame(const jms::vulkan::State& vulkan_state, const vk::raii::Buffer& vertex_buffer, const std::vector<Vertex>& vertices) {
    const auto& image_available_semaphore = vulkan_state.semaphores.at(0);
    const auto& render_finished_semaphore = vulkan_state.semaphores.at(0);
    const auto& in_flight_fence = vulkan_state.fences.at(0);
    const auto& device = vulkan_state.devices.at(0);
    const auto& swapchain = vulkan_state.swapchain;
    const auto& swapchain_framebuffers = vulkan_state.swapchain_framebuffers;
    const auto& command_buffer = vulkan_state.command_buffers.at(0);
    const auto& render_pass = vulkan_state.render_passes.at(0);
    const auto& swapchain_extent = vulkan_state.render_info.extent;
    const auto& pipeline = vulkan_state.pipelines.at(0);
    const auto& viewport = vulkan_state.viewport;
    const auto& scissor = vulkan_state.scissor;
    const auto& graphics_queue = vulkan_state.graphics_queue;
    const auto& present_queue = vulkan_state.present_queue;

    vk::Result result = device.waitForFences({*in_flight_fence}, VK_TRUE, std::numeric_limits<uint64_t>::max());
    device.resetFences({*in_flight_fence});
    uint32_t image_index = 0;
    std::tie(result, image_index) = swapchain.acquireNextImage(std::numeric_limits<uint64_t>::max(), *image_available_semaphore);
    assert(result == vk::Result::eSuccess);
    assert(image_index < swapchain_framebuffers.size());
    vk::ClearValue clear_color{.color={std::array<float, 4>{0.0f, 0.0f, 0.0f, 0.0f}}};

    command_buffer.reset();
    command_buffer.begin({.pInheritanceInfo=nullptr});
    command_buffer.beginRenderPass({
        .renderPass=*render_pass,
        .framebuffer=*swapchain_framebuffers[image_index],
        .renderArea={
            .offset={0, 0},
            .extent=swapchain_extent
        },
        .clearValueCount=1,
        .pClearValues=&clear_color // count + values can be array
    }, vk::SubpassContents::eInline);
    command_buffer.bindPipeline(vk::PipelineBindPoint::eGraphics, *pipeline);
    command_buffer.setViewport(0, viewport);
    command_buffer.setScissor(0, scissor);
    command_buffer.bindVertexBuffers(0, {*vertex_buffer}, {0});
    command_buffer.draw(static_cast<uint32_t>(vertices.size()), 1, 0, 0);
    command_buffer.endRenderPass();
    command_buffer.end();

    std::vector<vk::Semaphore> wait_semaphores{*image_available_semaphore};
    std::vector<vk::Semaphore> signal_semaphores{*render_finished_semaphore};
    std::vector<vk::PipelineStageFlags> dst_stage_mask{vk::PipelineStageFlagBits::eColorAttachmentOutput};
    std::vector<vk::CommandBuffer> command_buffers{*command_buffer};
    graphics_queue.submit(std::array<vk::SubmitInfo, 1>{vk::SubmitInfo{
        .waitSemaphoreCount=static_cast<uint32_t>(wait_semaphores.size()),
        .pWaitSemaphores=wait_semaphores.data(),
        .pWaitDstStageMask=dst_stage_mask.data(),
        .commandBufferCount=static_cast<uint32_t>(command_buffers.size()),
        .pCommandBuffers=command_buffers.data(),
        .signalSemaphoreCount=static_cast<uint32_t>(signal_semaphores.size()),
        .pSignalSemaphores=signal_semaphores.data()
    }}, *in_flight_fence);
    std::vector<vk::SwapchainKHR> swapchains{*swapchain};
    result = present_queue.presentKHR({
        .waitSemaphoreCount=static_cast<uint32_t>(signal_semaphores.size()),
        .pWaitSemaphores=signal_semaphores.data(),
        .swapchainCount=static_cast<uint32_t>(swapchains.size()),
        .pSwapchains=swapchains.data(),
        .pImageIndices=&image_index,
        .pResults=nullptr
    });
}
#endif
