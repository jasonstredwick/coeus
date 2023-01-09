#include <array>
#include <cstddef>
#include <exception>
#include <expected>
#include <format>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "jms/vulkan/include_config.hpp"
#include <vulkan/vulkan_raii.hpp>

#include "jms/vulkan/render_info.hpp"
#include "jms/vulkan/state.hpp"

#if defined(_WIN32) || defined(_WIN64)
#include "jms/sys_window/win.hpp"
#endif

void DrawFrame(const jms::vulkan::State& vulkan_state, const vk::raii::Buffer& vertex_buffer, const std::vector<Vertex>& vertices);


int main(char** argv, int argc) {
    char a = 0;
    std::cout << std::format("Start\n");

    try {
        jms::SysWindow::SetHighDPI();
        jms::SysWindow sys_window = jms::SysWindow::Create({});

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

        vulkan_state.InitInstance(jms::vulkan::InstanceConfig{
            .app_name=std::string{"tut2"},
            .engine_name=std::string{"tut2.e"},
            .layer_names={
                std::string{"VK_LAYER_KHRONOS_synchronization2"},
                std::string{"VK_LAYER_KHRONOS_validation"}
            },
            .extension_names={
                std::string{VK_KHR_SURFACE_EXTENSION_NAME},
                std::string{VK_KHR_WIN32_SURFACE_EXTENSION_NAME}
            }
        });

        // Create surface; must happen after instance creation, but before examining/creating devices.
        // will be moved from
        vulkan_state.surface = vulkan_state.instance.createWin32SurfaceKHR({
            .flags=vk::Win32SurfaceCreateFlagsKHR(),
            .hinstance=sys_window.GetInstance(),
            .hwnd=sys_window.GetHWND()
        });

        vk::raii::PhysicalDevice& physical_device = vulkan_state.physical_devices.at(0);
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
                std::string{VK_KHR_SWAPCHAIN_EXTENSION_NAME}
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

    std::cout << std::format("End\n");
    return 0;
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
