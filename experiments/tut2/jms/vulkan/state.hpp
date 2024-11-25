#include <algorithm>
#include <exception>
#if defined(_GNUG_)
#include <fmt/core.h>
#else
#include <format>
#endif
#include <iterator>
#include <optional>
#include <ranges>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "include_config.hpp"
#include "vulkan/vulkan_raii.hpp"

#include "debug.hpp"
#include "render_info.hpp"
#include "shader.hpp"


struct Vertex {
    std::array<float, 2> position{};
    std::array<float, 3> color{};

    static vk::VertexInputBindingDescription GetBindingDesc() {
        return {
            .binding=0,
            .stride=sizeof(Vertex),
            .inputRate=vk::VertexInputRate::eVertex
        };
    }

    static std::array<vk::VertexInputAttributeDescription, 2> GetAttributeDesc() {
        return {
            vk::VertexInputAttributeDescription{
                .location=0,
                .binding=0,
                .format=vk::Format::eR32G32Sfloat,
                .offset=offsetof(Vertex, position) // conditionally supported; no good alternative?  check hpp examples
            },
            vk::VertexInputAttributeDescription{
                .location=1,
                .binding=0,
                .format=vk::Format::eR32G32B32Sfloat,
                .offset=offsetof(Vertex, color) // conditionally supported
            }
        };
    }
};
static_assert(std::is_standard_layout_v<Vertex>); // requires for offsetof macro






namespace jms {
namespace vulkan {


struct alignas(16) InstanceConfig {
    uint32_t api_version=0;
    uint32_t app_version = 0;
    uint32_t engine_version = 0;
    std::string app_name{};
    std::string engine_name{};
    std::vector<std::string> layer_names{};
    std::vector<std::string> extension_names{};
    std::optional<jms::vulkan::DebugConfig> debug{};
};


struct alignas(16) DeviceConfig {
    std::vector<std::string> layer_names{};
    std::vector<std::string> extension_names{};
    vk::PhysicalDeviceFeatures features{};
    std::vector<vk::DeviceQueueCreateInfo> queue_infos{};
};


struct State {
    InstanceConfig instance_config{};
    DeviceConfig device_config{};
    vk::raii::Context context{};
    vk::raii::Instance instance{nullptr};
    vk::raii::DebugUtilsMessengerEXT debug_messenger{nullptr};
    vk::raii::SurfaceKHR surface{nullptr};
    vk::raii::PhysicalDevices physical_devices{nullptr};
    jms::vulkan::RenderInfo render_info{};
    std::vector<vk::raii::Device> devices{};
    std::vector<vk::raii::RenderPass> render_passes{};
    vk::Viewport viewport{};
    vk::Rect2D scissor{};
    std::vector<vk::raii::Pipeline> pipelines{};
    vk::raii::Queue graphics_queue{nullptr};
    vk::raii::Queue present_queue{nullptr};
    std::vector<vk::raii::Semaphore> semaphores{};
    std::vector<vk::raii::Fence> fences{};
    vk::raii::CommandPool command_pool{nullptr};
    vk::raii::CommandBuffers command_buffers{nullptr};
    vk::raii::SwapchainKHR swapchain{nullptr};
    std::vector<vk::raii::ImageView> swapchain_image_views{};
    std::vector<vk::raii::Framebuffer> swapchain_framebuffers{};
    std::vector<vk::raii::Buffer> buffers{};
    std::vector<vk::MemoryRequirements> buffers_mem_reqs{};
    std::vector<vk::raii::DeviceMemory> device_memory{};

    State() = default;
    State(const State&) = delete;
    State(State&&) = default;
    ~State() = default;
    State& operator=(const State&) = delete;
    State& operator=(State&&) = default;

    vk::raii::PhysicalDevice& PhysicalDevice(size_t index) { return physical_devices.at(index); }

    void InitDevice(const vk::raii::PhysicalDevice& physical_device, DeviceConfig&& cfg);
    void InitInstance(InstanceConfig&& cfg);
    void InitMemory(const size_t size_in_bytes, const vk::raii::PhysicalDevice& physical_device, const vk::raii::Device& device);
    void InitPipeline(const vk::raii::Device& device, const vk::raii::RenderPass& render_pass, const vk::Extent2D target_extent);
    void InitQueues(const vk::raii::Device& device, const uint32_t queue_family_index);
    void InitRenderPass(const vk::raii::Device& device, const vk::Format pixel_format, const vk::Extent2D target_extent);
    void InitSwapchain(const vk::raii::Device& device, const jms::vulkan::RenderInfo& render_info, const vk::raii::SurfaceKHR& surface, const vk::raii::RenderPass& render_pass);
};


void State::InitDevice(const vk::raii::PhysicalDevice& physical_device, DeviceConfig&& cfg) {
    device_config = cfg;

    auto StrToCharP = [](std::string& i) { return i.c_str(); };
    std::vector<const char*> layers_name_vec{};
    std::ranges::transform(device_config.layer_names, std::back_inserter(layers_name_vec), StrToCharP);
    std::vector<const char*> extensions_name_vec{};
    std::ranges::transform(device_config.extension_names, std::back_inserter(extensions_name_vec), StrToCharP);

    devices.push_back(vk::raii::Device{physical_device, {
        .queueCreateInfoCount=static_cast<uint32_t>(device_config.queue_infos.size()),
        .pQueueCreateInfos=device_config.queue_infos.data(),
        .enabledLayerCount=static_cast<uint32_t>(layers_name_vec.size()),
        .ppEnabledLayerNames=layers_name_vec.data(),
        .enabledExtensionCount=static_cast<uint32_t>(extensions_name_vec.size()),
        .ppEnabledExtensionNames=extensions_name_vec.data(),
        .pEnabledFeatures=&device_config.features
    }});
}


void State::InitInstance(InstanceConfig&& cfg) {
    instance_config = cfg;

    vk::ApplicationInfo application_info{
        .pApplicationName=instance_config.app_name.c_str(),
        .applicationVersion=instance_config.app_version,
        .pEngineName=instance_config.engine_name.c_str(),
        .engineVersion=instance_config.engine_version,
        .apiVersion=(instance_config.api_version) ? instance_config.api_version : context.enumerateInstanceVersion()
    };

    std::vector<vk::LayerProperties> layer_props = context.enumerateInstanceLayerProperties();
    std::set<std::string> layer_names{};
    std::ranges::transform(layer_props, std::inserter(layer_names, layer_names.begin()),
                            [](auto& i) -> std::string { return i.layerName; });
    std::set<std::string> requested_layer_names{instance_config.layer_names.begin(), instance_config.layer_names.end()};
    if (instance_config.debug.has_value()) { requested_layer_names.insert(std::string{"VK_LAYER_KHRONOS_validation"}); }
    for (auto& i : requested_layer_names) {
        if (!layer_names.contains(i)) {
#if defined(_GNUG_)
            throw std::runtime_error(fmt::format("Requested layer \"{}\" not available.", i));
#else
            throw std::runtime_error(std::format("Requested layer \"{}\" not available.", i));
#endif
        }
    }

    std::vector<vk::ExtensionProperties> extension_props = context.enumerateInstanceExtensionProperties();
    std::set<std::string> extension_names{};
    std::ranges::transform(extension_props, std::inserter(extension_names, extension_names.begin()),
                            [](auto& i) -> std::string { return i.extensionName; });
    for (auto& i : instance_config.extension_names) {
        if (!extension_names.contains(i)) {
#if defined(_GNUG_)
            throw std::runtime_error(fmt::format("Requested instance extension \"{}\" not available.", i));
#else
            throw std::runtime_error(std::format("Requested instance extension \"{}\" not available.", i));
#endif
        }
    }

    auto TransFn = [](auto& i) { return i.c_str(); };
    std::vector<const char*> layer_names_vec{};
    std::ranges::transform(requested_layer_names, std::back_insert_iterator(layer_names_vec), TransFn);
    std::vector<const char*> ext_names_vec{};
    std::ranges::transform(instance_config.extension_names, std::back_insert_iterator(ext_names_vec), TransFn);

    instance = vk::raii::Instance{context, vk::InstanceCreateInfo{
        .flags=vk::InstanceCreateFlags(),
        .pApplicationInfo=&application_info,
        .enabledLayerCount=static_cast<uint32_t>(layer_names_vec.size()),
        .ppEnabledLayerNames=layer_names_vec.data(),
        .enabledExtensionCount=static_cast<uint32_t>(ext_names_vec.size()),
        .ppEnabledExtensionNames=ext_names_vec.data()
    }};

    if (instance_config.debug.has_value()) {
        vk::DebugUtilsMessengerCreateInfoEXT debug_messenger_create_info{
            .flags=vk::DebugUtilsMessengerCreateFlagsEXT(),
            .messageSeverity=instance_config.debug.value().severity_flags,
            .messageType=instance_config.debug.value().msg_type_flags,
            .pfnUserCallback=instance_config.debug.value().debug_fn
        };
        debug_messenger = vk::raii::DebugUtilsMessengerEXT{instance, debug_messenger_create_info};
    }

    physical_devices = vk::raii::PhysicalDevices{instance};
}


void State::InitMemory(const size_t size_in_bytes,
                        const vk::raii::PhysicalDevice& physical_device,
                        const vk::raii::Device& device) {
    buffers.push_back(vk::raii::Buffer{device, {
        .size=size_in_bytes,
        .usage=vk::BufferUsageFlagBits::eVertexBuffer,
        .sharingMode=vk::SharingMode::eExclusive
    }});
    const auto& buffer = buffers.back();
    buffers_mem_reqs.push_back(buffer.getMemoryRequirements());
    const auto& vertex_mem_reqs = buffers_mem_reqs.back();
    uint32_t vertex_mem_type_filter = vertex_mem_reqs.memoryTypeBits;

    auto desired_props = vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent;
    vk::PhysicalDeviceMemoryProperties physical_device_mem_props = physical_device.getMemoryProperties();
    uint32_t memory_type_index = physical_device_mem_props.memoryTypeCount;
    for (auto i : std::views::iota(static_cast<uint32_t>(0), physical_device_mem_props.memoryTypeCount)) {
        auto prop_check = physical_device_mem_props.memoryTypes[i].propertyFlags & desired_props;
        if ((1 << i) & vertex_mem_type_filter && prop_check == desired_props) { memory_type_index = i; }
    }
    if (memory_type_index == physical_device_mem_props.memoryTypeCount) {
#if defined(_GNUG_)
        throw std::runtime_error(fmt::format("Failed to find the appropriate memory type on physical device.\n"));
#else
        throw std::runtime_error(std::format("Failed to find the appropriate memory type on physical device.\n"));
#endif
    }

    device_memory.push_back(device.allocateMemory({
        .allocationSize=vertex_mem_reqs.size,
        .memoryTypeIndex=memory_type_index
    }));
}


void State::InitPipeline(const vk::raii::Device& device,
                         const vk::raii::RenderPass& render_pass,
                         const vk::Extent2D target_extent) {
    vk::raii::ShaderModule frag_shader = jms::vulkan::shader::Load("shaders/shader.frag.spv", device);
    vk::raii::ShaderModule vert_shader = jms::vulkan::shader::Load("shaders/shader.vert.spv", device);
    std::vector<vk::PipelineShaderStageCreateInfo> shader_stages{
        {
            .stage=vk::ShaderStageFlagBits::eVertex,
            .module=*vert_shader,
            .pName="main"
        },
        {
            .stage=vk::ShaderStageFlagBits::eFragment,
            .module=*frag_shader,
            .pName="main"
        }
    };

    // *********************************************************
    // pass in?
    auto vertex_binding_desc = Vertex::GetBindingDesc();
    auto vertex_attr_desc = Vertex::GetAttributeDesc();
    // *********************************************************
    vk::PipelineVertexInputStateCreateInfo vertex_info{
        .vertexBindingDescriptionCount=1,
        .pVertexBindingDescriptions=&vertex_binding_desc,
        .vertexAttributeDescriptionCount=static_cast<uint32_t>(vertex_attr_desc.size()),
        .pVertexAttributeDescriptions=vertex_attr_desc.data()
    };

    vk::PipelineInputAssemblyStateCreateInfo input_assembly_info{
        .topology=vk::PrimitiveTopology::eTriangleList,
        .primitiveRestartEnable=VK_FALSE
    };

    std::vector<vk::DynamicState> dynamic_states = {vk::DynamicState::eViewport, vk::DynamicState::eScissor};

    vk::PipelineDynamicStateCreateInfo dynamic_state_info{
        .dynamicStateCount=static_cast<uint32_t>(dynamic_states.size()),
        .pDynamicStates=dynamic_states.data()
    };

    // *********************************************************
    // Needed since it is dynamic?
    viewport = vk::Viewport{
        .x=0.0f,
        .y=0.0f,
        .width=static_cast<float>(target_extent.width),
        .height=static_cast<float>(target_extent.height),
        .minDepth=0.0f,
        .maxDepth=1.0f
    };

    scissor = vk::Rect2D{
        .offset={0, 0},
        .extent=target_extent
    };

    vk::PipelineViewportStateCreateInfo viewport_info{
        .viewportCount=1,
        .pViewports=&viewport,
        .scissorCount=1,
        .pScissors=&scissor
    };
    // *********************************************************

    vk::PipelineRasterizationStateCreateInfo rasterizer_info{
        .depthClampEnable=VK_FALSE,
        .rasterizerDiscardEnable=VK_FALSE,
        .polygonMode=vk::PolygonMode::eFill,
        .cullMode=vk::CullModeFlagBits::eBack,
        .frontFace=vk::FrontFace::eClockwise,
        .depthBiasEnable=VK_FALSE,
        .depthBiasConstantFactor=0.0f,
        .depthBiasClamp=0.0f,
        .depthBiasSlopeFactor=0.0f,
        .lineWidth=1.0f
    };

    vk::PipelineMultisampleStateCreateInfo multisampling_info{
        .rasterizationSamples=vk::SampleCountFlagBits::e1,
        .sampleShadingEnable=VK_FALSE,
        .minSampleShading=1.0f,
        .pSampleMask=nullptr,
        .alphaToCoverageEnable=VK_FALSE,
        .alphaToOneEnable=VK_FALSE
    };

    vk::PipelineColorBlendAttachmentState color_blend_attachment{
        .blendEnable=VK_FALSE,
        .srcColorBlendFactor=vk::BlendFactor::eOne,
        .dstColorBlendFactor=vk::BlendFactor::eZero,
        .colorBlendOp=vk::BlendOp::eAdd,
        .srcAlphaBlendFactor=vk::BlendFactor::eOne,
        .dstAlphaBlendFactor=vk::BlendFactor::eZero,
        .alphaBlendOp=vk::BlendOp::eAdd,
        .colorWriteMask=vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG |
                        vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA
    };

    vk::PipelineColorBlendStateCreateInfo color_blend_info{
        .logicOpEnable=VK_FALSE,
        .logicOp=vk::LogicOp::eCopy,
        .attachmentCount=1,
        .pAttachments=&color_blend_attachment,
        .blendConstants=std::array<float, 4>{0.0f, 0.0f, 0.0f, 0.0f}
    };

    vk::raii::PipelineLayout pipeline_layout{device, {
        .setLayoutCount=0,
        .pSetLayouts=nullptr,
        .pushConstantRangeCount=0,
        .pPushConstantRanges=nullptr
    }};

    vk::PipelineLayout vk_pipeline_layout = *pipeline_layout;
    vk::RenderPass vk_render_pass = *render_pass;
    uint32_t subpass_id = 0;
    pipelines.push_back(vk::raii::Pipeline(device, nullptr, vk::GraphicsPipelineCreateInfo{
        .stageCount=static_cast<uint32_t>(shader_stages.size()),
        .pStages=shader_stages.data(),
        .pVertexInputState=&vertex_info,
        .pInputAssemblyState=&input_assembly_info,
        .pViewportState=&viewport_info,
        .pRasterizationState=&rasterizer_info,
        .pMultisampleState=&multisampling_info,
        .pDepthStencilState=nullptr,
        .pColorBlendState=&color_blend_info,
        .pDynamicState=&dynamic_state_info,
        .layout=vk_pipeline_layout,
        .renderPass=vk_render_pass,
        .subpass=subpass_id,
        .basePipelineHandle=VK_NULL_HANDLE,
        .basePipelineIndex=-1
    }));
}


void State::InitQueues(const vk::raii::Device& device, const uint32_t queue_family_index) {
    graphics_queue = vk::raii::Queue{device, 0, 0};
    present_queue = vk::raii::Queue{device, 0, 1};
    command_pool = device.createCommandPool({
        .flags=vk::CommandPoolCreateFlagBits::eResetCommandBuffer,
        .queueFamilyIndex=queue_family_index
    });
    command_buffers = vk::raii::CommandBuffers(device, {
        .commandPool=*command_pool,
        .level=vk::CommandBufferLevel::ePrimary,
        .commandBufferCount=1
    });
    semaphores.push_back(device.createSemaphore({}));
    semaphores.push_back(device.createSemaphore({}));
    fences.push_back(device.createFence({.flags=vk::FenceCreateFlagBits::eSignaled}));
}


void State::InitRenderPass(const vk::raii::Device& device, const vk::Format pixel_format, const vk::Extent2D target_extent) {
    vk::AttachmentDescription color_attachment{
        .format=pixel_format,
        .samples=vk::SampleCountFlagBits::e1,
        .loadOp=vk::AttachmentLoadOp::eClear,
        .storeOp=vk::AttachmentStoreOp::eStore,
        .stencilLoadOp=vk::AttachmentLoadOp::eDontCare,
        .stencilStoreOp=vk::AttachmentStoreOp::eDontCare,
        .initialLayout=vk::ImageLayout::eUndefined,
        .finalLayout=vk::ImageLayout::ePresentSrcKHR
    };

    vk::AttachmentReference color_attachment_reference{
        .attachment=0,
        .layout=vk::ImageLayout::eColorAttachmentOptimal
    };

    vk::SubpassDescription subpass_desc{
        .pipelineBindPoint=vk::PipelineBindPoint::eGraphics,
        .inputAttachmentCount=0,
        .pInputAttachments=nullptr,
        .colorAttachmentCount=1,
        .pColorAttachments=&color_attachment_reference,
        .pResolveAttachments=nullptr,
        .pDepthStencilAttachment=nullptr,
        .preserveAttachmentCount=0,
        .pPreserveAttachments=nullptr
    };

    std::vector<vk::SubpassDependency> subpass_deps{
        {
            .srcSubpass=VK_SUBPASS_EXTERNAL,// vk::SubpassDependency::eE...?
            .dstSubpass=0,
            .srcStageMask=vk::PipelineStageFlagBits::eColorAttachmentOutput,
            .dstStageMask=vk::PipelineStageFlagBits::eColorAttachmentOutput,
            .srcAccessMask=vk::AccessFlagBits::eNone,
            .dstAccessMask=vk::AccessFlagBits::eColorAttachmentWrite
        }
    };

    render_passes.push_back(vk::raii::RenderPass{device, {
        .attachmentCount=1,
        .pAttachments=&color_attachment,
        .subpassCount=1,
        .pSubpasses=&subpass_desc,
        .dependencyCount=static_cast<uint32_t>(subpass_deps.size()),
        .pDependencies=subpass_deps.data()
    }});
}


void State::InitSwapchain(const vk::raii::Device& device,
                          const jms::vulkan::RenderInfo& render_info,
                          const vk::raii::SurfaceKHR& surface,
                          const vk::raii::RenderPass& render_pass) {
    swapchain = vk::raii::SwapchainKHR{device, {
        .flags=vk::SwapchainCreateFlagsKHR(),
        .surface=*surface,
        .minImageCount=render_info.image_count,
        .imageFormat=render_info.format,
        .imageColorSpace=render_info.color_space,
        .imageExtent=render_info.extent,
        .imageArrayLayers=1, // mono
        .imageUsage=vk::ImageUsageFlagBits::eColorAttachment,
        .imageSharingMode=vk::SharingMode::eExclusive,
        .queueFamilyIndexCount=0,
        .pQueueFamilyIndices=nullptr,
        .preTransform=render_info.transform_bits,
        .compositeAlpha=vk::CompositeAlphaFlagBitsKHR::eOpaque,
        .presentMode=render_info.present_mode,
        .clipped=VK_TRUE,
        .oldSwapchain=VK_NULL_HANDLE
    }};

    std::vector<vk::Image> swapchain_images = swapchain.getImages();
    for (auto image : swapchain_images) {
        swapchain_image_views.push_back({device, {
            .image=image,
            .viewType=vk::ImageViewType::e2D,
            .format=render_info.format,
            .components={
                .r = vk::ComponentSwizzle::eIdentity,
                .g = vk::ComponentSwizzle::eIdentity,
                .b = vk::ComponentSwizzle::eIdentity,
                .a = vk::ComponentSwizzle::eIdentity
            },
            .subresourceRange={
                .aspectMask=vk::ImageAspectFlagBits::eColor,
                .baseMipLevel=0,
                .levelCount=1,
                .baseArrayLayer=0,
                .layerCount=1
            }
        }});
    }

    for (auto& image_view : swapchain_image_views) {
        std::array<vk::ImageView, 1> attachments = {*image_view};
        swapchain_framebuffers.push_back(vk::raii::Framebuffer{device, vk::FramebufferCreateInfo{
            .renderPass=*render_pass,
            .attachmentCount=static_cast<uint32_t>(attachments.size()),
            .pAttachments=attachments.data(),
            .width=render_info.extent.width,
            .height=render_info.extent.height,
            .layers=1
        }});
    }
}


}
}
