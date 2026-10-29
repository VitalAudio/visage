/* Copyright Vital Audio, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "gpu_backend.h"

#define VK_NO_PROTOTYPES
#include <X11/Xlib.h>
#include <vulkan/vulkan.h>
#include <vulkan/vulkan_xlib.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <functional>
#include <unordered_map>

namespace visage::gpu {
  namespace {
#define VISAGE_VK_GLOBAL_FUNCTIONS(X)        \
  X(vkCreateInstance)                        \
  X(vkEnumerateInstanceExtensionProperties) \
  X(vkEnumerateInstanceLayerProperties)

#define VISAGE_VK_INSTANCE_FUNCTIONS(X)            \
  X(vkDestroyInstance)                             \
  X(vkEnumeratePhysicalDevices)                    \
  X(vkGetPhysicalDeviceProperties)                 \
  X(vkGetPhysicalDeviceQueueFamilyProperties)      \
  X(vkGetPhysicalDeviceMemoryProperties)           \
  X(vkGetPhysicalDeviceFormatProperties)           \
  X(vkCreateDevice)                                \
  X(vkGetDeviceProcAddr)                           \
  X(vkEnumerateDeviceExtensionProperties)          \
  X(vkDestroySurfaceKHR)                           \
  X(vkGetPhysicalDeviceSurfaceSupportKHR)          \
  X(vkGetPhysicalDeviceSurfaceCapabilitiesKHR)     \
  X(vkGetPhysicalDeviceSurfaceFormatsKHR)          \
  X(vkGetPhysicalDeviceSurfacePresentModesKHR)     \
  X(vkGetPhysicalDeviceXlibPresentationSupportKHR) \
  X(vkCreateXlibSurfaceKHR)

#define VISAGE_VK_DEBUG_FUNCTIONS(X)  \
  X(vkCreateDebugUtilsMessengerEXT) \
  X(vkDestroyDebugUtilsMessengerEXT)

#define VISAGE_VK_DEVICE_FUNCTIONS(X)  \
  X(vkDestroyDevice)                   \
  X(vkGetDeviceQueue)                  \
  X(vkDeviceWaitIdle)                  \
  X(vkQueueSubmit)                     \
  X(vkQueuePresentKHR)                 \
  X(vkCreateSwapchainKHR)              \
  X(vkDestroySwapchainKHR)             \
  X(vkGetSwapchainImagesKHR)           \
  X(vkAcquireNextImageKHR)             \
  X(vkCreateCommandPool)               \
  X(vkDestroyCommandPool)              \
  X(vkResetCommandPool)                \
  X(vkAllocateCommandBuffers)          \
  X(vkBeginCommandBuffer)              \
  X(vkEndCommandBuffer)                \
  X(vkCreateFence)                     \
  X(vkDestroyFence)                    \
  X(vkWaitForFences)                   \
  X(vkResetFences)                     \
  X(vkCreateSemaphore)                 \
  X(vkDestroySemaphore)                \
  X(vkAllocateMemory)                  \
  X(vkFreeMemory)                      \
  X(vkMapMemory)                       \
  X(vkCreateBuffer)                    \
  X(vkDestroyBuffer)                   \
  X(vkGetBufferMemoryRequirements)     \
  X(vkBindBufferMemory)                \
  X(vkCreateImage)                     \
  X(vkDestroyImage)                    \
  X(vkGetImageMemoryRequirements)      \
  X(vkBindImageMemory)                 \
  X(vkCreateImageView)                 \
  X(vkDestroyImageView)                \
  X(vkCreateSampler)                   \
  X(vkDestroySampler)                  \
  X(vkCreateShaderModule)              \
  X(vkDestroyShaderModule)             \
  X(vkCreateDescriptorSetLayout)       \
  X(vkDestroyDescriptorSetLayout)      \
  X(vkCreatePipelineLayout)            \
  X(vkDestroyPipelineLayout)           \
  X(vkCreateGraphicsPipelines)         \
  X(vkCreateComputePipelines)          \
  X(vkDestroyPipeline)                 \
  X(vkCreateDescriptorPool)            \
  X(vkDestroyDescriptorPool)           \
  X(vkResetDescriptorPool)             \
  X(vkAllocateDescriptorSets)          \
  X(vkUpdateDescriptorSets)            \
  X(vkCreateRenderPass)                \
  X(vkDestroyRenderPass)               \
  X(vkCreateFramebuffer)               \
  X(vkDestroyFramebuffer)              \
  X(vkCmdBeginRenderPass)              \
  X(vkCmdEndRenderPass)                \
  X(vkCmdBindPipeline)                 \
  X(vkCmdBindDescriptorSets)           \
  X(vkCmdBindVertexBuffers)            \
  X(vkCmdBindIndexBuffer)              \
  X(vkCmdDraw)                         \
  X(vkCmdDrawIndexed)                  \
  X(vkCmdDispatch)                     \
  X(vkCmdSetViewport)                  \
  X(vkCmdSetScissor)                   \
  X(vkCmdPipelineBarrier)              \
  X(vkCmdCopyBuffer)                   \
  X(vkCmdCopyBufferToImage)            \
  X(vkCmdCopyImageToBuffer)            \
  X(vkCmdCopyImage)                    \
  X(vkCmdFillBuffer)                   \
  X(vkCmdClearColorImage)

#define VISAGE_VK_DECLARE(name) PFN_##name name = nullptr;
    PFN_vkGetInstanceProcAddr vkGetInstanceProcAddr = nullptr;
    VISAGE_VK_GLOBAL_FUNCTIONS(VISAGE_VK_DECLARE)
    VISAGE_VK_INSTANCE_FUNCTIONS(VISAGE_VK_DECLARE)
    VISAGE_VK_DEBUG_FUNCTIONS(VISAGE_VK_DECLARE)
    VISAGE_VK_DEVICE_FUNCTIONS(VISAGE_VK_DECLARE)
#undef VISAGE_VK_DECLARE

    constexpr int kFramesInFlight = 2;
    constexpr int kResourceBindingOffset = 16;
    constexpr VkDeviceSize kRingChunkSize = 1 << 20;
    constexpr uint32_t kDescriptorPoolSets = 1024;
    constexpr uint64_t kAcquireTimeoutNs = 1000000000;

    struct FormatInfo {
      VkFormat format;
      uint32_t bytes;
    };

    FormatInfo formatInfo(TextureFormat format) {
      switch (format) {
      case TextureFormat::RGBA8: return { VK_FORMAT_R8G8B8A8_UNORM, 4 };
      case TextureFormat::BGRA8: return { VK_FORMAT_B8G8R8A8_UNORM, 4 };
      case TextureFormat::R8: return { VK_FORMAT_R8_UNORM, 1 };
      case TextureFormat::R32F: return { VK_FORMAT_R32_SFLOAT, 4 };
      case TextureFormat::RGBA16F: return { VK_FORMAT_R16G16B16A16_SFLOAT, 8 };
      case TextureFormat::RGBA32F: return { VK_FORMAT_R32G32B32A32_SFLOAT, 16 };
      default: return { VK_FORMAT_UNDEFINED, 0 };
      }
    }

    VkBlendFactor blendFactor(BlendFactor factor) {
      switch (factor) {
      case BlendFactor::Zero: return VK_BLEND_FACTOR_ZERO;
      case BlendFactor::One: return VK_BLEND_FACTOR_ONE;
      case BlendFactor::SrcColor: return VK_BLEND_FACTOR_SRC_COLOR;
      case BlendFactor::InvSrcColor: return VK_BLEND_FACTOR_ONE_MINUS_SRC_COLOR;
      case BlendFactor::SrcAlpha: return VK_BLEND_FACTOR_SRC_ALPHA;
      case BlendFactor::InvSrcAlpha: return VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
      case BlendFactor::DstColor: return VK_BLEND_FACTOR_DST_COLOR;
      case BlendFactor::InvDstColor: return VK_BLEND_FACTOR_ONE_MINUS_DST_COLOR;
      case BlendFactor::DstAlpha: return VK_BLEND_FACTOR_DST_ALPHA;
      case BlendFactor::InvDstAlpha: return VK_BLEND_FACTOR_ONE_MINUS_DST_ALPHA;
      }
      return VK_BLEND_FACTOR_ONE;
    }

    VkBlendOp blendOp(BlendOp op) {
      switch (op) {
      case BlendOp::Add: return VK_BLEND_OP_ADD;
      case BlendOp::Subtract: return VK_BLEND_OP_SUBTRACT;
      case BlendOp::ReverseSubtract: return VK_BLEND_OP_REVERSE_SUBTRACT;
      case BlendOp::Min: return VK_BLEND_OP_MIN;
      case BlendOp::Max: return VK_BLEND_OP_MAX;
      }
      return VK_BLEND_OP_ADD;
    }

    struct Allocation {
      VkDeviceMemory memory = VK_NULL_HANDLE;
      void* mapped = nullptr;
    };

    struct HostBuffer {
      VkBuffer buffer = VK_NULL_HANDLE;
      Allocation allocation;
      VkDeviceSize size = 0;
    };

    struct RingAllocation {
      VkBuffer buffer = VK_NULL_HANDLE;
      VkDeviceSize offset = 0;
      uint8_t* data = nullptr;
    };

    struct Texture {
      bool used = false;
      TextureDesc desc;
      VkImage image = VK_NULL_HANDLE;
      VkImageView view = VK_NULL_HANDLE;
      Allocation allocation;
      VkImageLayout layout = VK_IMAGE_LAYOUT_UNDEFINED;
      VkFormat format = VK_FORMAT_UNDEFINED;
      uint32_t bytes = 0;
    };

    struct Buffer {
      bool used = false;
      BufferDesc desc;
      VkBuffer buffer = VK_NULL_HANDLE;
      Allocation allocation;
      VkDeviceSize size = 0;
    };

    struct Shader {
      bool used = false;
      ShaderInfo info;
      VkShaderModule module = VK_NULL_HANDLE;
    };

    struct Program {
      bool used = false;
      bool compute = false;
      uint16_t vertex = kInvalidHandle;
      uint16_t fragment = kInvalidHandle;
      std::vector<ShaderAttribute> attributes;
      VkDescriptorSetLayout set_layout = VK_NULL_HANDLE;
      VkPipelineLayout pipeline_layout = VK_NULL_HANDLE;
      VkPipeline compute_pipeline = VK_NULL_HANDLE;
      bool compute_uniforms = false;
      std::vector<ShaderStorage> storage;
      std::vector<ShaderSampler> samplers;
    };

    struct SwapChain {
      void* window_handle = nullptr;
      void* display = nullptr;
      bool surface_lost = false;
      VkSurfaceKHR surface = VK_NULL_HANDLE;
      VkSwapchainKHR swap_chain = VK_NULL_HANDLE;
      VkFormat format = VK_FORMAT_UNDEFINED;
      VkExtent2D extent {};
      int width = 0;
      int height = 0;
      bool needs_recreate = false;
      std::vector<VkImage> images;
      std::vector<VkImageView> views;
      std::vector<VkFramebuffer> framebuffers;
      std::vector<VkSemaphore> render_finished;
      std::vector<VkImageLayout> layouts;
      uint32_t current_image = UINT32_MAX;
      bool acquired = false;
    };

    struct FrameBuffer {
      bool used = false;
      bool swap_chain = false;
      uint16_t texture = kInvalidHandle;
      VkFramebuffer framebuffer = VK_NULL_HANDLE;
      VkFormat format = VK_FORMAT_UNDEFINED;
      std::unique_ptr<SwapChain> chain;
    };

    struct DescriptorKey {
      VkBuffer vertex_buffer = VK_NULL_HANDLE;
      VkBuffer fragment_buffer = VK_NULL_HANDLE;
      uint32_t vertex_size = 0;
      uint32_t fragment_size = 0;
      TextureBinding textures[kMaxTextureStages];

      bool operator==(const DescriptorKey& other) const {
        if (vertex_buffer != other.vertex_buffer || fragment_buffer != other.fragment_buffer ||
            vertex_size != other.vertex_size || fragment_size != other.fragment_size)
          return false;
        for (int i = 0; i < kMaxTextureStages; ++i) {
          if (textures[i].texture != other.textures[i].texture ||
              textures[i].sampler_flags != other.textures[i].sampler_flags)
            return false;
        }
        return true;
      }
    };

    struct DescriptorKeyHash {
      size_t operator()(const DescriptorKey& key) const {
        size_t hash = std::hash<uint64_t>()(reinterpret_cast<uint64_t>(key.vertex_buffer));
        auto mix = [&hash](uint64_t value) { hash ^= value + 0x9e3779b97f4a7c15ull + (hash << 6) + (hash >> 2); };
        mix(reinterpret_cast<uint64_t>(key.fragment_buffer));
        mix(key.vertex_size | (static_cast<uint64_t>(key.fragment_size) << 32));
        for (const TextureBinding& binding : key.textures)
          mix(binding.texture | (static_cast<uint64_t>(binding.sampler_flags) << 16));
        return hash;
      }
    };

    struct FrameSlot {
      VkCommandPool command_pool = VK_NULL_HANDLE;
      VkCommandBuffer command_buffer = VK_NULL_HANDLE;
      VkFence fence = VK_NULL_HANDLE;
      bool submitted = false;
      std::vector<VkDescriptorPool> descriptor_pools;
      size_t descriptor_pool_index = 0;
      std::unordered_map<DescriptorKey, VkDescriptorSet, DescriptorKeyHash> descriptor_cache;
      std::vector<HostBuffer> ring_chunks;
      size_t ring_chunk = 0;
      VkDeviceSize ring_offset = 0;
      std::vector<VkSemaphore> acquire_semaphores;
      size_t acquire_semaphores_used = 0;
      std::vector<std::function<void()>> releases;
    };

    struct PendingInit {
      bool texture = false;
      uint16_t idx = kInvalidHandle;
      std::vector<uint8_t> data;
      int pitch = 0;
    };

    struct ReadBack {
      HostBuffer buffer;
      void* destination = nullptr;
      uint32_t row_bytes = 0;
      uint32_t rows = 0;
      uint16_t frame_buffer = kInvalidHandle;
      bool screenshot = false;
      bool bgra = false;
      int width = 0;
      int height = 0;
    };

    template<typename T>
    T& entry(std::vector<T>& entries, uint16_t idx) {
      if (entries.size() <= idx)
        entries.resize(idx + 1);
      return entries[idx];
    }
  }

  class VulkanBackend : public Backend {
  public:
    ~VulkanBackend() override { shutdown(); }

    bool init(const Init& init, Caps& caps) override {
      init_ = init;
      if (!loadLibrary() || !createInstance() || !selectDevice() || !createDevice())
        return false;

      createCommonObjects();
      fillCaps(caps);
      caps_ = caps;
      return true;
    }

    void shutdown() override {
      if (device_) {
        vkDeviceWaitIdle(device_);
        for (FrameSlot& slot : slots_)
          runReleases(slot);

        for (uint16_t i = 0; i < frame_buffers_.size(); ++i) {
          if (frame_buffers_[i].used)
            destroyFrameBufferNow(i);
        }
        for (uint16_t i = 0; i < programs_.size(); ++i) {
          if (programs_[i].used)
            destroyProgramNow(i);
        }
        for (uint16_t i = 0; i < shaders_.size(); ++i) {
          if (shaders_[i].used)
            vkDestroyShaderModule(device_, shaders_[i].module, nullptr);
        }
        for (uint16_t i = 0; i < textures_.size(); ++i) {
          if (textures_[i].used)
            destroyTextureNow(textures_[i]);
        }
        for (uint16_t i = 0; i < buffers_.size(); ++i) {
          if (buffers_[i].used)
            destroyBufferNow(buffers_[i]);
        }
        frame_buffers_.clear();
        programs_.clear();
        shaders_.clear();
        textures_.clear();
        buffers_.clear();

        for (auto& pipeline : pipelines_)
          vkDestroyPipeline(device_, pipeline.second, nullptr);
        pipelines_.clear();
        for (auto& sampler : samplers_)
          vkDestroySampler(device_, sampler.second, nullptr);
        samplers_.clear();
        for (auto& render_pass : render_passes_)
          vkDestroyRenderPass(device_, render_pass.second, nullptr);
        render_passes_.clear();

        for (FrameSlot& slot : slots_) {
          for (VkDescriptorPool pool : slot.descriptor_pools)
            vkDestroyDescriptorPool(device_, pool, nullptr);
          for (HostBuffer& chunk : slot.ring_chunks)
            destroyHostBuffer(chunk);
          for (VkSemaphore semaphore : slot.acquire_semaphores)
            vkDestroySemaphore(device_, semaphore, nullptr);
          vkDestroyFence(device_, slot.fence, nullptr);
          vkDestroyCommandPool(device_, slot.command_pool, nullptr);
          slot = FrameSlot();
        }

        destroyHostBuffer(placeholder_uniforms_);
        vkDestroyPipelineLayout(device_, graphics_pipeline_layout_, nullptr);
        vkDestroyDescriptorSetLayout(device_, graphics_set_layout_, nullptr);
        vkDestroyDevice(device_, nullptr);
        device_ = VK_NULL_HANDLE;
      }

      if (instance_) {
        if (debug_messenger_)
          vkDestroyDebugUtilsMessengerEXT(instance_, debug_messenger_, nullptr);
        vkDestroyInstance(instance_, nullptr);
        instance_ = VK_NULL_HANDLE;
      }
      if (library_) {
        dlclose(library_);
        library_ = nullptr;
      }
    }

    bool createTexture(uint16_t idx, const TextureDesc& desc, const uint8_t* data, int pitch) override {
      Texture& texture = entry(textures_, idx);
      texture = Texture();
      FormatInfo info = formatInfo(desc.format);
      if (info.format == VK_FORMAT_UNDEFINED)
        return false;

      texture.desc = desc;
      texture.format = info.format;
      texture.bytes = info.bytes;

      VkImageCreateInfo image_info { VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
      image_info.imageType = VK_IMAGE_TYPE_2D;
      image_info.format = info.format;
      image_info.extent = { static_cast<uint32_t>(desc.width), static_cast<uint32_t>(desc.height), 1 };
      image_info.mipLevels = 1;
      image_info.arrayLayers = 1;
      image_info.samples = VK_SAMPLE_COUNT_1_BIT;
      image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
      image_info.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT |
                         VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
      if (desc.flags & kTextureRenderTarget)
        image_info.usage |= VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
      if (desc.flags & kTextureStorage)
        image_info.usage |= VK_IMAGE_USAGE_STORAGE_BIT;
      image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
      if (vkCreateImage(device_, &image_info, nullptr, &texture.image) != VK_SUCCESS)
        return false;

      VkMemoryRequirements requirements;
      vkGetImageMemoryRequirements(device_, texture.image, &requirements);
      if (!allocate(requirements, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, texture.allocation) ||
          vkBindImageMemory(device_, texture.image, texture.allocation.memory, 0) != VK_SUCCESS) {
        destroyTextureNow(texture);
        return false;
      }

      VkImageViewCreateInfo view_info { VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
      view_info.image = texture.image;
      view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
      view_info.format = info.format;
      view_info.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
      if (vkCreateImageView(device_, &view_info, nullptr, &texture.view) != VK_SUCCESS) {
        destroyTextureNow(texture);
        return false;
      }

      texture.used = true;
      PendingInit init;
      init.texture = true;
      init.idx = idx;
      if (data) {
        uint32_t row_bytes = pitch ? pitch : desc.width * info.bytes;
        init.data.assign(data, data + static_cast<size_t>(row_bytes) * desc.height);
        init.pitch = row_bytes;
      }
      pending_inits_.push_back(std::move(init));
      return true;
    }

    void updateTexture(uint16_t idx, int x, int y, int width, int height, const uint8_t* data,
                       int pitch) override {
      Texture& texture = textures_[idx];
      uint32_t row_bytes = width * texture.bytes;
      uint32_t source_pitch = pitch ? pitch : row_bytes;
      RingAllocation staging = ringAllocate(static_cast<VkDeviceSize>(row_bytes) * height, 16);
      if (staging.data == nullptr)
        return;
      for (int row = 0; row < height; ++row)
        std::memcpy(staging.data + row * row_bytes, data + row * source_pitch, row_bytes);
      copyToImage(texture, staging, x, y, width, height);
    }

    void destroyTexture(uint16_t idx) override {
      Texture texture = textures_[idx];
      textures_[idx] = Texture();
      deferRelease([this, texture]() mutable { destroyTextureNow(texture); });
    }

    bool createBuffer(uint16_t idx, const BufferDesc& desc, const uint8_t* data) override {
      Buffer& buffer = entry(buffers_, idx);
      buffer = Buffer();
      buffer.desc = desc;
      buffer.size = (desc.size + 3) & ~VkDeviceSize(3);

      VkBufferCreateInfo info { VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
      info.size = buffer.size;
      info.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT |
                   VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
      if (desc.flags & kBufferComputeReadWrite)
        info.usage |= VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
      info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
      if (vkCreateBuffer(device_, &info, nullptr, &buffer.buffer) != VK_SUCCESS)
        return false;

      VkMemoryRequirements requirements;
      vkGetBufferMemoryRequirements(device_, buffer.buffer, &requirements);
      if (!allocate(requirements, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, buffer.allocation) ||
          vkBindBufferMemory(device_, buffer.buffer, buffer.allocation.memory, 0) != VK_SUCCESS) {
        destroyBufferNow(buffer);
        return false;
      }

      buffer.used = true;
      PendingInit init;
      init.idx = idx;
      if (data)
        init.data.assign(data, data + desc.size);
      pending_inits_.push_back(std::move(init));
      return true;
    }

    void updateBuffer(uint16_t idx, uint32_t offset, const uint8_t* data, uint32_t size) override {
      Buffer& buffer = buffers_[idx];
      RingAllocation staging = ringAllocate(size, 16);
      if (staging.data == nullptr)
        return;
      std::memcpy(staging.data, data, size);
      copyToBuffer(buffer, staging, offset, size);
    }

    void destroyBuffer(uint16_t idx) override {
      Buffer buffer = buffers_[idx];
      buffers_[idx] = Buffer();
      deferRelease([this, buffer]() mutable { destroyBufferNow(buffer); });
    }

    bool createShader(uint16_t idx, const ShaderInfo& info) override {
      if (info.code_type != ShaderCode::Spirv || info.code.empty() || info.code.size() % 4)
        return false;

      Shader& shader = entry(shaders_, idx);
      shader = Shader();
      VkShaderModuleCreateInfo module_info { VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
      module_info.codeSize = info.code.size();
      module_info.pCode = reinterpret_cast<const uint32_t*>(info.code.data());
      if (vkCreateShaderModule(device_, &module_info, nullptr, &shader.module) != VK_SUCCESS)
        return false;

      shader.used = true;
      shader.info = info;
      shader.info.code.clear();
      return true;
    }

    void destroyShader(uint16_t idx) override {
      VkShaderModule module = shaders_[idx].module;
      shaders_[idx] = Shader();
      vkDestroyShaderModule(device_, module, nullptr);
    }

    bool createProgram(uint16_t idx, uint16_t vertex, uint16_t fragment) override {
      Program& program = entry(programs_, idx);
      program = Program();
      program.used = true;
      program.vertex = vertex;
      program.fragment = fragment;
      program.attributes = shaders_[vertex].info.attributes;
      program.pipeline_layout = graphics_pipeline_layout_;
      return true;
    }

    bool createComputeProgram(uint16_t idx, uint16_t compute) override {
      Program& program = entry(programs_, idx);
      program = Program();
      const ShaderInfo& info = shaders_[compute].info;

      std::vector<VkDescriptorSetLayoutBinding> bindings;
      if (info.uniform_block_size) {
        bindings.push_back({ 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr });
        program.compute_uniforms = true;
      }
      for (const ShaderStorage& storage : info.storage) {
        VkDescriptorType type = storage.image ? VK_DESCRIPTOR_TYPE_STORAGE_IMAGE : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        bindings.push_back({ static_cast<uint32_t>(kResourceBindingOffset + storage.stage), type, 1,
                             VK_SHADER_STAGE_COMPUTE_BIT, nullptr });
      }
      for (const ShaderSampler& sampler : info.samplers) {
        bindings.push_back({ static_cast<uint32_t>(kResourceBindingOffset + sampler.stage),
                             VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr });
      }

      VkDescriptorSetLayoutCreateInfo set_info { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
      set_info.bindingCount = static_cast<uint32_t>(bindings.size());
      set_info.pBindings = bindings.data();
      if (vkCreateDescriptorSetLayout(device_, &set_info, nullptr, &program.set_layout) != VK_SUCCESS)
        return false;

      VkPipelineLayoutCreateInfo layout_info { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
      layout_info.setLayoutCount = 1;
      layout_info.pSetLayouts = &program.set_layout;
      if (vkCreatePipelineLayout(device_, &layout_info, nullptr, &program.pipeline_layout) != VK_SUCCESS) {
        vkDestroyDescriptorSetLayout(device_, program.set_layout, nullptr);
        return false;
      }

      VkComputePipelineCreateInfo pipeline_info { VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO };
      pipeline_info.stage = { VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
      pipeline_info.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
      pipeline_info.stage.module = shaders_[compute].module;
      pipeline_info.stage.pName = "main";
      pipeline_info.layout = program.pipeline_layout;
      if (vkCreateComputePipelines(device_, VK_NULL_HANDLE, 1, &pipeline_info, nullptr,
                                   &program.compute_pipeline) != VK_SUCCESS) {
        vkDestroyPipelineLayout(device_, program.pipeline_layout, nullptr);
        vkDestroyDescriptorSetLayout(device_, program.set_layout, nullptr);
        return false;
      }

      program.used = true;
      program.compute = true;
      program.vertex = compute;
      program.storage = info.storage;
      program.samplers = info.samplers;
      return true;
    }

    void destroyProgram(uint16_t idx) override {
      Program program = programs_[idx];
      programs_[idx] = Program();
      std::vector<VkPipeline> pipelines;
      for (auto it = pipelines_.begin(); it != pipelines_.end();) {
        if (it->first.program == idx) {
          pipelines.push_back(it->second);
          it = pipelines_.erase(it);
        }
        else
          ++it;
      }
      deferRelease([this, program, pipelines]() {
        for (VkPipeline pipeline : pipelines)
          vkDestroyPipeline(device_, pipeline, nullptr);
        Program copy = program;
        destroyProgramObjects(copy);
      });
    }

    bool createFrameBuffer(uint16_t idx, uint16_t texture_idx) override {
      FrameBuffer& frame_buffer = entry(frame_buffers_, idx);
      frame_buffer = FrameBuffer();
      Texture& texture = textures_[texture_idx];
      VkRenderPass render_pass = renderPass(texture.format);
      if (render_pass == VK_NULL_HANDLE)
        return false;

      VkFramebufferCreateInfo info { VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO };
      info.renderPass = render_pass;
      info.attachmentCount = 1;
      info.pAttachments = &texture.view;
      info.width = texture.desc.width;
      info.height = texture.desc.height;
      info.layers = 1;
      if (vkCreateFramebuffer(device_, &info, nullptr, &frame_buffer.framebuffer) != VK_SUCCESS)
        return false;

      frame_buffer.used = true;
      frame_buffer.texture = texture_idx;
      frame_buffer.format = texture.format;
      return true;
    }

    bool createSwapChain(uint16_t idx, void* window_handle, void* display, int width, int height,
                         TextureFormat) override {
      FrameBuffer& frame_buffer = entry(frame_buffers_, idx);
      frame_buffer = FrameBuffer();
      if (display == nullptr)
        return false;

      auto chain = std::make_unique<SwapChain>();
      chain->window_handle = window_handle;
      chain->display = display;
      if (!createSurface(*chain))
        return false;

      chain->width = width;
      chain->height = height;
      if (!buildSwapChain(*chain)) {
        destroySwapChainObjects(*chain);
        vkDestroySurfaceKHR(instance_, chain->surface, nullptr);
        return false;
      }

      frame_buffer.used = true;
      frame_buffer.swap_chain = true;
      frame_buffer.format = chain->format;
      frame_buffer.chain = std::move(chain);
      return true;
    }

    void resizeSwapChain(uint16_t idx, int width, int height) override {
      SwapChain* chain = frame_buffers_[idx].chain.get();
      if (chain == nullptr)
        return;
      chain->width = width;
      chain->height = height;
      chain->needs_recreate = true;
    }

    void destroyFrameBuffer(uint16_t idx) override {
      FrameBuffer& frame_buffer = frame_buffers_[idx];
      if (frame_buffer.swap_chain) {
        vkDeviceWaitIdle(device_);
        destroyFrameBufferNow(idx);
        return;
      }

      VkFramebuffer framebuffer = frame_buffer.framebuffer;
      frame_buffers_[idx] = FrameBuffer();
      deferRelease([this, framebuffer]() { vkDestroyFramebuffer(device_, framebuffer, nullptr); });
    }

    bool waitForFrame(uint16_t, int) override { return true; }
    bool deviceLost() override { return device_lost_; }

    void beginFrame(uint32_t frame_number) override {
      slot_index_ = frame_number % kFramesInFlight;
      FrameSlot& slot = slots_[slot_index_];
      if (slot.submitted) {
        checkResult(vkWaitForFences(device_, 1, &slot.fence, VK_TRUE, UINT64_MAX));
        slot.submitted = false;
      }
      vkResetFences(device_, 1, &slot.fence);
      runReleases(slot);
      resetRing(slot);
      for (size_t i = 0; i <= slot.descriptor_pool_index && i < slot.descriptor_pools.size(); ++i)
        vkResetDescriptorPool(device_, slot.descriptor_pools[i], 0);
      slot.descriptor_pool_index = 0;
      slot.descriptor_cache.clear();
      slot.acquire_semaphores_used = 0;

      vkResetCommandPool(device_, slot.command_pool, 0);
      VkCommandBufferBeginInfo begin { VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
      begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
      vkBeginCommandBuffer(slot.command_buffer, &begin);
      command_buffer_ = slot.command_buffer;

      std::vector<PendingInit> inits = std::move(pending_inits_);
      pending_inits_.clear();
      for (PendingInit& init : inits)
        runInit(init);
    }

    void submitFrame(const FrameData& frame, Stats&) override {
      std::vector<FrameBuffer*> presented;
      std::vector<VkSemaphore> wait_semaphores;
      std::vector<ReadBack> reads;

      for (const ViewData& view : frame.views) {
        FrameBuffer* frame_buffer = nullptr;
        if (frameBufferValid(view.frame_buffer)) {
          frame_buffer = &frame_buffers_[view.frame_buffer];
          if (frame_buffer->swap_chain && !frame_buffer->chain->acquired) {
            VkSemaphore semaphore = acquireSemaphore();
            if (acquire(*frame_buffer->chain, semaphore)) {
              wait_semaphores.push_back(semaphore);
              presented.push_back(frame_buffer);
            }
            else
              frame_buffer = nullptr;
          }
        }
        executeView(frame, view, frame_buffer);
      }

      for (const ReadRequest& request : frame.reads) {
        if (request.texture < textures_.size() && textures_[request.texture].used)
          reads.push_back(readTexture(textures_[request.texture], request.data));
      }

      for (uint16_t idx : frame.screenshots) {
        if (!frameBufferValid(idx))
          continue;
        FrameBuffer& frame_buffer = frame_buffers_[idx];
        if (frame_buffer.swap_chain && frame_buffer.chain->acquired)
          reads.push_back(readSwapChain(idx, *frame_buffer.chain));
        else if (!frame_buffer.swap_chain) {
          ReadBack read = readTexture(textures_[frame_buffer.texture], nullptr);
          read.screenshot = true;
          read.frame_buffer = idx;
          reads.push_back(std::move(read));
        }
      }

      std::vector<VkSemaphore> signal_semaphores;
      for (FrameBuffer* frame_buffer : presented) {
        SwapChain& chain = *frame_buffer->chain;
        transitionSwapImage(chain, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
        signal_semaphores.push_back(chain.render_finished[chain.current_image]);
      }

      vkEndCommandBuffer(command_buffer_);
      std::vector<VkPipelineStageFlags> wait_stages(wait_semaphores.size(), VK_PIPELINE_STAGE_ALL_COMMANDS_BIT);
      VkSubmitInfo submit { VK_STRUCTURE_TYPE_SUBMIT_INFO };
      submit.waitSemaphoreCount = static_cast<uint32_t>(wait_semaphores.size());
      submit.pWaitSemaphores = wait_semaphores.data();
      submit.pWaitDstStageMask = wait_stages.data();
      submit.commandBufferCount = 1;
      submit.pCommandBuffers = &command_buffer_;
      submit.signalSemaphoreCount = static_cast<uint32_t>(signal_semaphores.size());
      submit.pSignalSemaphores = signal_semaphores.data();
      FrameSlot& slot = slots_[slot_index_];
      if (checkResult(vkQueueSubmit(queue_, 1, &submit, slot.fence)) == VK_SUCCESS)
        slot.submitted = true;
      else
        log("Vulkan queue submit failed");

      for (FrameBuffer* frame_buffer : presented) {
        SwapChain& chain = *frame_buffer->chain;
        VkPresentInfoKHR present { VK_STRUCTURE_TYPE_PRESENT_INFO_KHR };
        present.waitSemaphoreCount = 1;
        present.pWaitSemaphores = &chain.render_finished[chain.current_image];
        present.swapchainCount = 1;
        present.pSwapchains = &chain.swap_chain;
        present.pImageIndices = &chain.current_image;
        VkResult result = checkResult(vkQueuePresentKHR(queue_, &present));
        if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR)
          chain.needs_recreate = true;
        else if (result == VK_ERROR_SURFACE_LOST_KHR)
          chain.surface_lost = true;
        chain.acquired = false;
      }

      if (!reads.empty()) {
        if (slot.submitted && checkResult(vkWaitForFences(device_, 1, &slot.fence, VK_TRUE, UINT64_MAX)) == VK_SUCCESS) {
          for (ReadBack& read : reads)
            finishRead(read);
        }
        for (ReadBack& read : reads)
          destroyHostBuffer(read.buffer);
      }
      command_buffer_ = VK_NULL_HANDLE;
    }

  private:
    struct PipelineKey {
      uint16_t program = 0;
      uint32_t layout_hash = 0;
      uint32_t state = 0;
      VkFormat format = VK_FORMAT_UNDEFINED;

      bool operator==(const PipelineKey& other) const {
        return program == other.program && layout_hash == other.layout_hash && state == other.state &&
               format == other.format;
      }
    };

    struct PipelineKeyHash {
      size_t operator()(const PipelineKey& key) const {
        uint64_t value = key.program | (static_cast<uint64_t>(key.state) << 16);
        return std::hash<uint64_t>()(value) ^ (std::hash<uint64_t>()(key.layout_hash) * 31) ^
               (static_cast<size_t>(key.format) * 131);
      }
    };

    VkResult checkResult(VkResult result) {
      if (result == VK_ERROR_DEVICE_LOST && !device_lost_) {
        device_lost_ = true;
        log("Vulkan device lost");
      }
      return result;
    }

    void log(const char* message) const {
      if (init_.log_callback)
        init_.log_callback(message);
      else
        std::fprintf(stderr, "visage vulkan: %s\n", message);
    }

    static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT severity,
                                                        VkDebugUtilsMessageTypeFlagsEXT,
                                                        const VkDebugUtilsMessengerCallbackDataEXT* data,
                                                        void* user_data) {
      if (severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT)
        static_cast<VulkanBackend*>(user_data)->log(data->pMessage);
      return VK_FALSE;
    }

    bool loadLibrary() {
      library_ = dlopen("libvulkan.so.1", RTLD_NOW | RTLD_LOCAL);
      if (library_ == nullptr)
        library_ = dlopen("libvulkan.so", RTLD_NOW | RTLD_LOCAL);
      if (library_ == nullptr)
        return false;

      vkGetInstanceProcAddr = reinterpret_cast<PFN_vkGetInstanceProcAddr>(dlsym(library_, "vkGetInstanceProcAddr"));
      if (vkGetInstanceProcAddr == nullptr)
        return false;

#define VISAGE_VK_LOAD(name)                                                                  \
  name = reinterpret_cast<PFN_##name>(vkGetInstanceProcAddr(VK_NULL_HANDLE, #name)); \
  if (name == nullptr)                                                                        \
    return false;
      VISAGE_VK_GLOBAL_FUNCTIONS(VISAGE_VK_LOAD)
#undef VISAGE_VK_LOAD
      return true;
    }

    bool createInstance() {
      uint32_t count = 0;
      vkEnumerateInstanceExtensionProperties(nullptr, &count, nullptr);
      std::vector<VkExtensionProperties> extensions(count);
      vkEnumerateInstanceExtensionProperties(nullptr, &count, extensions.data());
      auto has_extension = [&](const char* name) {
        return std::any_of(extensions.begin(), extensions.end(),
                           [name](const VkExtensionProperties& e) { return std::strcmp(e.extensionName, name) == 0; });
      };

      std::vector<const char*> enabled;
      if (!has_extension(VK_KHR_SURFACE_EXTENSION_NAME) || !has_extension(VK_KHR_XLIB_SURFACE_EXTENSION_NAME))
        return false;
      enabled.push_back(VK_KHR_SURFACE_EXTENSION_NAME);
      enabled.push_back(VK_KHR_XLIB_SURFACE_EXTENSION_NAME);

      const char* validation_env = std::getenv("VISAGE_VULKAN_VALIDATION");
      bool validation = init_.debug || (validation_env && validation_env[0] == '1');
      std::vector<const char*> layers;
      if (validation) {
        uint32_t layer_count = 0;
        vkEnumerateInstanceLayerProperties(&layer_count, nullptr);
        std::vector<VkLayerProperties> available(layer_count);
        vkEnumerateInstanceLayerProperties(&layer_count, available.data());
        bool has_layer = std::any_of(available.begin(), available.end(), [](const VkLayerProperties& layer) {
          return std::strcmp(layer.layerName, "VK_LAYER_KHRONOS_validation") == 0;
        });
        if (has_layer)
          layers.push_back("VK_LAYER_KHRONOS_validation");
        if (has_extension(VK_EXT_DEBUG_UTILS_EXTENSION_NAME))
          enabled.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
        else
          validation = false;
      }

      VkApplicationInfo app { VK_STRUCTURE_TYPE_APPLICATION_INFO };
      app.pApplicationName = "visage";
      app.pEngineName = "visage";
      app.apiVersion = VK_API_VERSION_1_1;

      VkInstanceCreateInfo info { VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO };
      info.pApplicationInfo = &app;
      info.enabledExtensionCount = static_cast<uint32_t>(enabled.size());
      info.ppEnabledExtensionNames = enabled.data();
      info.enabledLayerCount = static_cast<uint32_t>(layers.size());
      info.ppEnabledLayerNames = layers.data();
      if (vkCreateInstance(&info, nullptr, &instance_) != VK_SUCCESS)
        return false;

#define VISAGE_VK_LOAD(name)                                                             \
  name = reinterpret_cast<PFN_##name>(vkGetInstanceProcAddr(instance_, #name)); \
  if (name == nullptr)                                                                   \
    return false;
      VISAGE_VK_INSTANCE_FUNCTIONS(VISAGE_VK_LOAD)
#undef VISAGE_VK_LOAD

      if (validation) {
#define VISAGE_VK_LOAD(name) name = reinterpret_cast<PFN_##name>(vkGetInstanceProcAddr(instance_, #name));
        VISAGE_VK_DEBUG_FUNCTIONS(VISAGE_VK_LOAD)
#undef VISAGE_VK_LOAD
        if (vkCreateDebugUtilsMessengerEXT) {
          VkDebugUtilsMessengerCreateInfoEXT messenger { VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT };
          messenger.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                                      VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
          messenger.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                                  VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                                  VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
          messenger.pfnUserCallback = debugCallback;
          messenger.pUserData = this;
          vkCreateDebugUtilsMessengerEXT(instance_, &messenger, nullptr, &debug_messenger_);
        }
      }
      return true;
    }

    bool selectDevice() {
      uint32_t count = 0;
      vkEnumeratePhysicalDevices(instance_, &count, nullptr);
      std::vector<VkPhysicalDevice> devices(count);
      vkEnumeratePhysicalDevices(instance_, &count, devices.data());

      int best_score = -1;
      for (VkPhysicalDevice device : devices) {
        VkPhysicalDeviceProperties properties;
        vkGetPhysicalDeviceProperties(device, &properties);
        if (VK_VERSION_MINOR(properties.apiVersion) < 1 && VK_VERSION_MAJOR(properties.apiVersion) <= 1)
          continue;

        uint32_t extension_count = 0;
        vkEnumerateDeviceExtensionProperties(device, nullptr, &extension_count, nullptr);
        std::vector<VkExtensionProperties> extensions(extension_count);
        vkEnumerateDeviceExtensionProperties(device, nullptr, &extension_count, extensions.data());
        bool has_swap_chain = std::any_of(extensions.begin(), extensions.end(), [](const VkExtensionProperties& e) {
          return std::strcmp(e.extensionName, VK_KHR_SWAPCHAIN_EXTENSION_NAME) == 0;
        });
        if (!has_swap_chain)
          continue;

        uint32_t family_count = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(device, &family_count, nullptr);
        std::vector<VkQueueFamilyProperties> families(family_count);
        vkGetPhysicalDeviceQueueFamilyProperties(device, &family_count, families.data());
        uint32_t family = UINT32_MAX;
        bool presents = false;
        for (uint32_t i = 0; i < family_count && !presents; ++i) {
          VkQueueFlags needed = VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT;
          if ((families[i].queueFlags & needed) != needed)
            continue;

          presents = canPresent(device, i);
          if (family == UINT32_MAX || presents)
            family = i;
        }
        if (family == UINT32_MAX)
          continue;

        bool cpu = properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_CPU;
        int device_score = (presents ? 2 : 0) + (cpu ? 0 : 1);
        if (device_score > best_score) {
          best_score = device_score;
          physical_device_ = device;
          queue_family_ = family;
          properties_ = properties;
        }
      }
      if (physical_device_ == VK_NULL_HANDLE)
        return false;

      vkGetPhysicalDeviceMemoryProperties(physical_device_, &memory_properties_);
      return true;
    }

    bool canPresent(VkPhysicalDevice device, uint32_t family) const {
      ::Display* display = static_cast<::Display*>(init_.display);
      if (display == nullptr)
        return true;

      VisualID visual = DefaultVisual(display, DefaultScreen(display))->visualid;
      return vkGetPhysicalDeviceXlibPresentationSupportKHR(device, family, display, visual);
    }

    bool createDevice() {
      float priority = 1.0f;
      VkDeviceQueueCreateInfo queue_info { VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO };
      queue_info.queueFamilyIndex = queue_family_;
      queue_info.queueCount = 1;
      queue_info.pQueuePriorities = &priority;

      const char* extensions[] = { VK_KHR_SWAPCHAIN_EXTENSION_NAME };
      VkDeviceCreateInfo info { VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO };
      info.queueCreateInfoCount = 1;
      info.pQueueCreateInfos = &queue_info;
      info.enabledExtensionCount = 1;
      info.ppEnabledExtensionNames = extensions;
      if (vkCreateDevice(physical_device_, &info, nullptr, &device_) != VK_SUCCESS)
        return false;

#define VISAGE_VK_LOAD(name)                                                         \
  name = reinterpret_cast<PFN_##name>(vkGetDeviceProcAddr(device_, #name)); \
  if (name == nullptr)                                                               \
    return false;
      VISAGE_VK_DEVICE_FUNCTIONS(VISAGE_VK_LOAD)
#undef VISAGE_VK_LOAD

      vkGetDeviceQueue(device_, queue_family_, 0, &queue_);
      return true;
    }

    void createCommonObjects() {
      std::vector<VkDescriptorSetLayoutBinding> bindings;
      bindings.push_back({ 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1, VK_SHADER_STAGE_VERTEX_BIT, nullptr });
      bindings.push_back({ 1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr });
      for (int i = 0; i < kMaxTextureStages; ++i) {
        bindings.push_back({ static_cast<uint32_t>(kResourceBindingOffset + i),
                             VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                             VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, nullptr });
      }
      VkDescriptorSetLayoutCreateInfo set_info { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
      set_info.bindingCount = static_cast<uint32_t>(bindings.size());
      set_info.pBindings = bindings.data();
      vkCreateDescriptorSetLayout(device_, &set_info, nullptr, &graphics_set_layout_);

      VkPipelineLayoutCreateInfo layout_info { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
      layout_info.setLayoutCount = 1;
      layout_info.pSetLayouts = &graphics_set_layout_;
      vkCreatePipelineLayout(device_, &layout_info, nullptr, &graphics_pipeline_layout_);

      for (FrameSlot& slot : slots_) {
        VkCommandPoolCreateInfo pool_info { VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
        pool_info.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
        pool_info.queueFamilyIndex = queue_family_;
        vkCreateCommandPool(device_, &pool_info, nullptr, &slot.command_pool);

        VkCommandBufferAllocateInfo allocate_info { VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
        allocate_info.commandPool = slot.command_pool;
        allocate_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocate_info.commandBufferCount = 1;
        vkAllocateCommandBuffers(device_, &allocate_info, &slot.command_buffer);

        VkFenceCreateInfo fence_info { VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
        vkCreateFence(device_, &fence_info, nullptr, &slot.fence);
      }
    }

    void fillCaps(Caps& caps) {
      caps.renderer = RendererType::Vulkan;
      caps.origin_bottom_left = false;
      caps.compute = true;
      caps.swap_chain = true;
      caps.max_texture_size = static_cast<int>(properties_.limits.maxImageDimension2D);
      for (int i = 0; i < static_cast<int>(TextureFormat::Count); ++i) {
        VkFormatProperties properties;
        vkGetPhysicalDeviceFormatProperties(physical_device_, formatInfo(static_cast<TextureFormat>(i)).format,
                                            &properties);
        VkFormatFeatureFlags features = properties.optimalTilingFeatures;
        uint32_t flags = 0;
        if (features & VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT)
          flags |= kFormatTexture;
        if (features & VK_FORMAT_FEATURE_COLOR_ATTACHMENT_BIT)
          flags |= kFormatRenderTarget;
        if (features & VK_FORMAT_FEATURE_COLOR_ATTACHMENT_BLEND_BIT)
          flags |= kFormatBlend;
        if (features & VK_FORMAT_FEATURE_STORAGE_IMAGE_BIT)
          flags |= kFormatStorage;
        caps.formats[i] = flags;
      }
    }

    bool allocate(const VkMemoryRequirements& requirements, VkMemoryPropertyFlags properties,
                  Allocation& allocation) {
      uint32_t type = UINT32_MAX;
      for (uint32_t i = 0; i < memory_properties_.memoryTypeCount; ++i) {
        if ((requirements.memoryTypeBits & (1u << i)) &&
            (memory_properties_.memoryTypes[i].propertyFlags & properties) == properties) {
          type = i;
          break;
        }
      }
      if (type == UINT32_MAX && (properties & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)) {
        for (uint32_t i = 0; i < memory_properties_.memoryTypeCount && type == UINT32_MAX; ++i) {
          if (requirements.memoryTypeBits & (1u << i))
            type = i;
        }
      }
      if (type == UINT32_MAX || num_allocations_ >= properties_.limits.maxMemoryAllocationCount)
        return false;

      VkMemoryAllocateInfo info { VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
      info.allocationSize = requirements.size;
      info.memoryTypeIndex = type;
      if (vkAllocateMemory(device_, &info, nullptr, &allocation.memory) != VK_SUCCESS)
        return false;
      num_allocations_++;

      if (properties & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) {
        if (vkMapMemory(device_, allocation.memory, 0, VK_WHOLE_SIZE, 0, &allocation.mapped) != VK_SUCCESS) {
          freeMemory(allocation.memory);
          allocation = Allocation();
          return false;
        }
      }
      return true;
    }

    void freeMemory(VkDeviceMemory memory) {
      if (memory == VK_NULL_HANDLE)
        return;
      vkFreeMemory(device_, memory, nullptr);
      num_allocations_--;
    }

    bool createHostBuffer(VkDeviceSize size, VkBufferUsageFlags usage, HostBuffer& result) {
      result = HostBuffer();
      VkBufferCreateInfo info { VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
      info.size = size;
      info.usage = usage;
      info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
      if (vkCreateBuffer(device_, &info, nullptr, &result.buffer) != VK_SUCCESS)
        return false;

      VkMemoryRequirements requirements;
      vkGetBufferMemoryRequirements(device_, result.buffer, &requirements);
      VkMemoryPropertyFlags flags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
      if (!allocate(requirements, flags, result.allocation) ||
          vkBindBufferMemory(device_, result.buffer, result.allocation.memory, 0) != VK_SUCCESS) {
        destroyHostBuffer(result);
        return false;
      }
      result.size = size;
      return true;
    }

    void destroyHostBuffer(HostBuffer& buffer) {
      if (buffer.buffer)
        vkDestroyBuffer(device_, buffer.buffer, nullptr);
      freeMemory(buffer.allocation.memory);
      buffer = HostBuffer();
    }

    void resetRing(FrameSlot& slot) {
      if (slot.ring_chunks.size() > 1) {
        VkDeviceSize total = 0;
        for (HostBuffer& chunk : slot.ring_chunks) {
          total += chunk.size;
          destroyHostBuffer(chunk);
        }
        slot.ring_chunks.clear();
        HostBuffer chunk;
        if (createHostBuffer(total, ringUsage(), chunk))
          slot.ring_chunks.push_back(chunk);
      }
      slot.ring_chunk = 0;
      slot.ring_offset = 0;
    }

    static VkBufferUsageFlags ringUsage() {
      return VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
    }

    RingAllocation ringAllocate(VkDeviceSize size, VkDeviceSize alignment) {
      FrameSlot& slot = slots_[slot_index_];
      alignment = std::max<VkDeviceSize>(alignment, 16);
      while (true) {
        if (slot.ring_chunk < slot.ring_chunks.size()) {
          HostBuffer& chunk = slot.ring_chunks[slot.ring_chunk];
          VkDeviceSize offset = (slot.ring_offset + alignment - 1) & ~(alignment - 1);
          if (offset + size <= chunk.size) {
            slot.ring_offset = offset + size;
            return { chunk.buffer, offset, static_cast<uint8_t*>(chunk.allocation.mapped) + offset };
          }
          if (slot.ring_chunk + 1 < slot.ring_chunks.size()) {
            slot.ring_chunk++;
            slot.ring_offset = 0;
            continue;
          }
        }

        VkDeviceSize chunk_size = kRingChunkSize;
        if (!slot.ring_chunks.empty())
          chunk_size = std::max(chunk_size, slot.ring_chunks.back().size * 2);
        chunk_size = std::max(chunk_size, size + alignment);
        HostBuffer chunk;
        if (!createHostBuffer(chunk_size, ringUsage(), chunk)) {
          log("Failed to allocate a staging buffer");
          return {};
        }
        slot.ring_chunks.push_back(chunk);
        slot.ring_chunk = slot.ring_chunks.size() - 1;
        slot.ring_offset = 0;
      }
    }

    void deferRelease(std::function<void()> release) {
      slots_[slot_index_].releases.push_back(std::move(release));
    }

    void runReleases(FrameSlot& slot) {
      std::vector<std::function<void()>> releases = std::move(slot.releases);
      slot.releases.clear();
      for (auto& release : releases)
        release();
    }

    void destroyTextureNow(Texture& texture) {
      if (texture.view)
        vkDestroyImageView(device_, texture.view, nullptr);
      if (texture.image)
        vkDestroyImage(device_, texture.image, nullptr);
      freeMemory(texture.allocation.memory);
      texture = Texture();
    }

    void destroyBufferNow(Buffer& buffer) {
      if (buffer.buffer)
        vkDestroyBuffer(device_, buffer.buffer, nullptr);
      freeMemory(buffer.allocation.memory);
      buffer = Buffer();
    }

    void destroyProgramObjects(Program& program) {
      if (program.compute_pipeline)
        vkDestroyPipeline(device_, program.compute_pipeline, nullptr);
      if (program.compute) {
        vkDestroyPipelineLayout(device_, program.pipeline_layout, nullptr);
        vkDestroyDescriptorSetLayout(device_, program.set_layout, nullptr);
      }
      program = Program();
    }

    void destroyProgramNow(uint16_t idx) {
      for (auto it = pipelines_.begin(); it != pipelines_.end();) {
        if (it->first.program == idx) {
          vkDestroyPipeline(device_, it->second, nullptr);
          it = pipelines_.erase(it);
        }
        else
          ++it;
      }
      destroyProgramObjects(programs_[idx]);
    }

    void destroySwapChainObjects(SwapChain& chain) {
      for (VkFramebuffer framebuffer : chain.framebuffers)
        vkDestroyFramebuffer(device_, framebuffer, nullptr);
      for (VkImageView view : chain.views)
        vkDestroyImageView(device_, view, nullptr);
      for (VkSemaphore semaphore : chain.render_finished)
        vkDestroySemaphore(device_, semaphore, nullptr);
      chain.framebuffers.clear();
      chain.views.clear();
      chain.render_finished.clear();
      chain.images.clear();
      chain.layouts.clear();
    }

    void destroyFrameBufferNow(uint16_t idx) {
      FrameBuffer& frame_buffer = frame_buffers_[idx];
      if (frame_buffer.chain) {
        SwapChain& chain = *frame_buffer.chain;
        destroySwapChainObjects(chain);
        if (chain.swap_chain)
          vkDestroySwapchainKHR(device_, chain.swap_chain, nullptr);
        if (chain.surface)
          vkDestroySurfaceKHR(instance_, chain.surface, nullptr);
      }
      if (frame_buffer.framebuffer)
        vkDestroyFramebuffer(device_, frame_buffer.framebuffer, nullptr);
      frame_buffer = FrameBuffer();
    }

    bool buildSwapChain(SwapChain& chain) {
      VkSurfaceCapabilitiesKHR capabilities;
      if (vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physical_device_, chain.surface, &capabilities) != VK_SUCCESS)
        return false;

      VkExtent2D extent = capabilities.currentExtent;
      if (extent.width == UINT32_MAX) {
        extent.width = std::clamp<uint32_t>(std::max(1, chain.width), capabilities.minImageExtent.width,
                                            capabilities.maxImageExtent.width);
        extent.height = std::clamp<uint32_t>(std::max(1, chain.height), capabilities.minImageExtent.height,
                                             capabilities.maxImageExtent.height);
      }
      if (extent.width == 0 || extent.height == 0)
        return false;

      uint32_t format_count = 0;
      vkGetPhysicalDeviceSurfaceFormatsKHR(physical_device_, chain.surface, &format_count, nullptr);
      std::vector<VkSurfaceFormatKHR> formats(format_count);
      vkGetPhysicalDeviceSurfaceFormatsKHR(physical_device_, chain.surface, &format_count, formats.data());
      if (formats.empty())
        return false;

      VkSurfaceFormatKHR surface_format = formats[0];
      for (VkFormat preferred : { VK_FORMAT_B8G8R8A8_UNORM, VK_FORMAT_R8G8B8A8_UNORM }) {
        auto found = std::find_if(formats.begin(), formats.end(), [preferred](const VkSurfaceFormatKHR& format) {
          return format.format == preferred && format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;
        });
        if (found != formats.end()) {
          surface_format = *found;
          break;
        }
      }
      if (chain.format != VK_FORMAT_UNDEFINED && chain.format != surface_format.format)
        return false;

      uint32_t mode_count = 0;
      vkGetPhysicalDeviceSurfacePresentModesKHR(physical_device_, chain.surface, &mode_count, nullptr);
      std::vector<VkPresentModeKHR> modes(mode_count);
      vkGetPhysicalDeviceSurfacePresentModesKHR(physical_device_, chain.surface, &mode_count, modes.data());
      VkPresentModeKHR present_mode = VK_PRESENT_MODE_FIFO_KHR;
      if (!init_.vsync) {
        for (VkPresentModeKHR mode : { VK_PRESENT_MODE_MAILBOX_KHR, VK_PRESENT_MODE_IMMEDIATE_KHR }) {
          if (std::find(modes.begin(), modes.end(), mode) != modes.end()) {
            present_mode = mode;
            break;
          }
        }
      }

      uint32_t image_count = std::max<uint32_t>(capabilities.minImageCount, 2);
      if (capabilities.maxImageCount)
        image_count = std::min(image_count, capabilities.maxImageCount);

      VkCompositeAlphaFlagBitsKHR composite = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
      if (!(capabilities.supportedCompositeAlpha & composite)) {
        for (VkCompositeAlphaFlagBitsKHR flag : { VK_COMPOSITE_ALPHA_INHERIT_BIT_KHR,
                                                  VK_COMPOSITE_ALPHA_PRE_MULTIPLIED_BIT_KHR,
                                                  VK_COMPOSITE_ALPHA_POST_MULTIPLIED_BIT_KHR }) {
          if (capabilities.supportedCompositeAlpha & flag) {
            composite = flag;
            break;
          }
        }
      }

      VkSwapchainCreateInfoKHR info { VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR };
      info.surface = chain.surface;
      info.minImageCount = image_count;
      info.imageFormat = surface_format.format;
      info.imageColorSpace = surface_format.colorSpace;
      info.imageExtent = extent;
      info.imageArrayLayers = 1;
      info.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
      if (capabilities.supportedUsageFlags & VK_IMAGE_USAGE_TRANSFER_SRC_BIT)
        info.imageUsage |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
      info.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
      info.preTransform = (capabilities.supportedTransforms & VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR) ?
                              VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR :
                              capabilities.currentTransform;
      info.compositeAlpha = composite;
      info.presentMode = present_mode;
      info.clipped = VK_TRUE;
      info.oldSwapchain = chain.swap_chain;

      VkSwapchainKHR swap_chain = VK_NULL_HANDLE;
      VkResult result = vkCreateSwapchainKHR(device_, &info, nullptr, &swap_chain);
      destroySwapChainObjects(chain);
      if (chain.swap_chain)
        vkDestroySwapchainKHR(device_, chain.swap_chain, nullptr);
      chain.swap_chain = VK_NULL_HANDLE;
      if (result != VK_SUCCESS)
        return false;

      chain.swap_chain = swap_chain;
      chain.format = surface_format.format;
      chain.extent = extent;

      uint32_t count = 0;
      vkGetSwapchainImagesKHR(device_, swap_chain, &count, nullptr);
      chain.images.resize(count);
      vkGetSwapchainImagesKHR(device_, swap_chain, &count, chain.images.data());
      chain.layouts.assign(count, VK_IMAGE_LAYOUT_UNDEFINED);

      VkRenderPass render_pass = renderPass(chain.format);
      for (VkImage image : chain.images) {
        VkImageViewCreateInfo view_info { VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        view_info.image = image;
        view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
        view_info.format = chain.format;
        view_info.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
        VkImageView view = VK_NULL_HANDLE;
        vkCreateImageView(device_, &view_info, nullptr, &view);
        chain.views.push_back(view);

        VkFramebufferCreateInfo framebuffer_info { VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO };
        framebuffer_info.renderPass = render_pass;
        framebuffer_info.attachmentCount = 1;
        framebuffer_info.pAttachments = &view;
        framebuffer_info.width = extent.width;
        framebuffer_info.height = extent.height;
        framebuffer_info.layers = 1;
        VkFramebuffer framebuffer = VK_NULL_HANDLE;
        vkCreateFramebuffer(device_, &framebuffer_info, nullptr, &framebuffer);
        chain.framebuffers.push_back(framebuffer);

        VkSemaphoreCreateInfo semaphore_info { VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
        VkSemaphore semaphore = VK_NULL_HANDLE;
        vkCreateSemaphore(device_, &semaphore_info, nullptr, &semaphore);
        chain.render_finished.push_back(semaphore);
      }
      chain.needs_recreate = false;
      return true;
    }

    bool createSurface(SwapChain& chain) {
      VkXlibSurfaceCreateInfoKHR surface_info { VK_STRUCTURE_TYPE_XLIB_SURFACE_CREATE_INFO_KHR };
      surface_info.dpy = static_cast<::Display*>(chain.display);
      surface_info.window = static_cast<::Window>(reinterpret_cast<uintptr_t>(chain.window_handle));
      if (vkCreateXlibSurfaceKHR(instance_, &surface_info, nullptr, &chain.surface) != VK_SUCCESS)
        return false;

      VkBool32 supported = VK_FALSE;
      vkGetPhysicalDeviceSurfaceSupportKHR(physical_device_, queue_family_, chain.surface, &supported);
      if (!supported) {
        vkDestroySurfaceKHR(instance_, chain.surface, nullptr);
        chain.surface = VK_NULL_HANDLE;
        return false;
      }
      return true;
    }

    bool recreateSurface(SwapChain& chain) {
      vkDeviceWaitIdle(device_);
      destroySwapChainObjects(chain);
      if (chain.swap_chain)
        vkDestroySwapchainKHR(device_, chain.swap_chain, nullptr);
      chain.swap_chain = VK_NULL_HANDLE;
      if (chain.surface)
        vkDestroySurfaceKHR(instance_, chain.surface, nullptr);
      chain.surface = VK_NULL_HANDLE;
      chain.surface_lost = false;
      return createSurface(chain) && buildSwapChain(chain);
    }

    bool recreateSwapChain(SwapChain& chain) {
      vkDeviceWaitIdle(device_);
      for (FrameSlot& slot : slots_) {
        if (slot.submitted && &slot != &slots_[slot_index_]) {
          vkWaitForFences(device_, 1, &slot.fence, VK_TRUE, UINT64_MAX);
          slot.submitted = false;
          runReleases(slot);
        }
      }
      return buildSwapChain(chain);
    }

    VkSemaphore acquireSemaphore() {
      FrameSlot& slot = slots_[slot_index_];
      if (slot.acquire_semaphores_used == slot.acquire_semaphores.size()) {
        VkSemaphoreCreateInfo info { VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
        VkSemaphore semaphore = VK_NULL_HANDLE;
        vkCreateSemaphore(device_, &info, nullptr, &semaphore);
        slot.acquire_semaphores.push_back(semaphore);
      }
      return slot.acquire_semaphores[slot.acquire_semaphores_used++];
    }

    bool acquire(SwapChain& chain, VkSemaphore semaphore) {
      for (int attempt = 0; attempt < 2; ++attempt) {
        if (chain.surface_lost && !recreateSurface(chain))
          return false;
        if ((chain.needs_recreate || chain.swap_chain == VK_NULL_HANDLE) && !recreateSwapChain(chain))
          return false;

        VkResult result = checkResult(vkAcquireNextImageKHR(device_, chain.swap_chain, kAcquireTimeoutNs, semaphore,
                                                            VK_NULL_HANDLE, &chain.current_image));
        if (result == VK_SUCCESS || result == VK_SUBOPTIMAL_KHR) {
          if (result == VK_SUBOPTIMAL_KHR)
            chain.needs_recreate = true;
          chain.acquired = true;
          chain.layouts[chain.current_image] = VK_IMAGE_LAYOUT_UNDEFINED;
          return true;
        }
        if (result == VK_ERROR_SURFACE_LOST_KHR)
          chain.surface_lost = true;
        else if (result == VK_ERROR_OUT_OF_DATE_KHR)
          chain.needs_recreate = true;
        else
          return false;
      }
      return false;
    }

    VkRenderPass renderPass(VkFormat format) {
      auto found = render_passes_.find(format);
      if (found != render_passes_.end())
        return found->second;

      VkAttachmentDescription attachment {};
      attachment.format = format;
      attachment.samples = VK_SAMPLE_COUNT_1_BIT;
      attachment.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
      attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
      attachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
      attachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
      attachment.initialLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
      attachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

      VkAttachmentReference reference { 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };
      VkSubpassDescription subpass {};
      subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
      subpass.colorAttachmentCount = 1;
      subpass.pColorAttachments = &reference;

      VkRenderPassCreateInfo info { VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO };
      info.attachmentCount = 1;
      info.pAttachments = &attachment;
      info.subpassCount = 1;
      info.pSubpasses = &subpass;
      VkRenderPass render_pass = VK_NULL_HANDLE;
      if (vkCreateRenderPass(device_, &info, nullptr, &render_pass) != VK_SUCCESS)
        return VK_NULL_HANDLE;
      render_passes_[format] = render_pass;
      return render_pass;
    }

    VkSampler sampler(uint32_t flags) {
      flags &= kSamplerMask;
      auto found = samplers_.find(flags);
      if (found != samplers_.end())
        return found->second;

      auto address = [](bool clamp, bool mirror) {
        if (clamp)
          return VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        if (mirror)
          return VK_SAMPLER_ADDRESS_MODE_MIRRORED_REPEAT;
        return VK_SAMPLER_ADDRESS_MODE_REPEAT;
      };

      VkSamplerCreateInfo info { VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
      info.magFilter = (flags & kSamplerMagPoint) ? VK_FILTER_NEAREST : VK_FILTER_LINEAR;
      info.minFilter = (flags & kSamplerMinPoint) ? VK_FILTER_NEAREST : VK_FILTER_LINEAR;
      info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
      info.addressModeU = address(flags & kSamplerUClamp, flags & kSamplerUMirror);
      info.addressModeV = address(flags & kSamplerVClamp, flags & kSamplerVMirror);
      info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
      info.maxLod = 0.0f;
      VkSampler result = VK_NULL_HANDLE;
      vkCreateSampler(device_, &info, nullptr, &result);
      samplers_[flags] = result;
      return result;
    }

    void barrier() {
      VkMemoryBarrier memory { VK_STRUCTURE_TYPE_MEMORY_BARRIER };
      memory.srcAccessMask = VK_ACCESS_MEMORY_WRITE_BIT;
      memory.dstAccessMask = VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT;
      vkCmdPipelineBarrier(command_buffer_, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
                           VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 1, &memory, 0, nullptr, 0, nullptr);
    }

    void transitionImage(VkImage image, VkImageLayout& current, VkImageLayout layout) {
      if (current == layout)
        return;

      VkImageMemoryBarrier image_barrier { VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER };
      image_barrier.srcAccessMask = current == VK_IMAGE_LAYOUT_UNDEFINED ? 0 : VK_ACCESS_MEMORY_WRITE_BIT;
      image_barrier.dstAccessMask = VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT;
      image_barrier.oldLayout = current;
      image_barrier.newLayout = layout;
      image_barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
      image_barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
      image_barrier.image = image;
      image_barrier.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
      vkCmdPipelineBarrier(command_buffer_, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
                           VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr, 0, nullptr, 1, &image_barrier);
      current = layout;
    }

    void transition(Texture& texture, VkImageLayout layout) {
      transitionImage(texture.image, texture.layout, layout);
    }

    void transitionSwapImage(SwapChain& chain, VkImageLayout layout) {
      transitionImage(chain.images[chain.current_image], chain.layouts[chain.current_image], layout);
    }

    void copyToImage(Texture& texture, const RingAllocation& staging, int x, int y, int width, int height) {
      transition(texture, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
      VkBufferImageCopy region {};
      region.bufferOffset = staging.offset;
      region.imageSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
      region.imageOffset = { x, y, 0 };
      region.imageExtent = { static_cast<uint32_t>(width), static_cast<uint32_t>(height), 1 };
      vkCmdCopyBufferToImage(command_buffer_, staging.buffer, texture.image,
                             VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
      transition(texture, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    }

    void copyToBuffer(Buffer& buffer, const RingAllocation& staging, uint32_t offset, uint32_t size) {
      barrier();
      VkBufferCopy region { staging.offset, offset, size };
      vkCmdCopyBuffer(command_buffer_, staging.buffer, buffer.buffer, 1, &region);
      barrier();
    }

    void runInit(PendingInit& init) {
      if (init.texture) {
        if (init.idx >= textures_.size() || !textures_[init.idx].used)
          return;
        Texture& texture = textures_[init.idx];
        if (init.data.empty()) {
          transition(texture, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
          VkClearColorValue clear {};
          VkImageSubresourceRange range { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
          vkCmdClearColorImage(command_buffer_, texture.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                               &clear, 1, &range);
          transition(texture, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        }
        else
          updateTexture(init.idx, 0, 0, texture.desc.width, texture.desc.height, init.data.data(), init.pitch);
        return;
      }

      if (init.idx >= buffers_.size() || !buffers_[init.idx].used)
        return;
      Buffer& buffer = buffers_[init.idx];
      if (init.data.empty()) {
        barrier();
        vkCmdFillBuffer(command_buffer_, buffer.buffer, 0, buffer.size, 0);
        barrier();
      }
      else
        updateBuffer(init.idx, 0, init.data.data(), static_cast<uint32_t>(init.data.size()));
    }

    bool frameBufferValid(uint16_t idx) const {
      return idx < frame_buffers_.size() && frame_buffers_[idx].used;
    }

    bool textureValid(uint16_t idx) const { return idx < textures_.size() && textures_[idx].used; }

    VkPipeline graphicsPipeline(uint16_t program_idx, const Buffer* vertex_buffer, const RenderState& state,
                                VkFormat format) {
      PipelineKey key;
      key.program = program_idx;
      key.layout_hash = vertex_buffer ? vertex_buffer->desc.layout.hash() : 0;
      key.state = state.key();
      key.format = format;
      auto found = pipelines_.find(key);
      if (found != pipelines_.end())
        return found->second;

      Program& program = programs_[program_idx];
      VkPipelineShaderStageCreateInfo stages[2] {};
      stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
      stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
      stages[0].module = shaders_[program.vertex].module;
      stages[0].pName = "main";
      stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
      stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
      stages[1].module = shaders_[program.fragment].module;
      stages[1].pName = "main";

      VkVertexInputBindingDescription binding {};
      std::vector<VkVertexInputAttributeDescription> attributes;
      VkPipelineVertexInputStateCreateInfo vertex_input { VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO };
      if (vertex_buffer && !program.attributes.empty()) {
        const VertexLayout& layout = vertex_buffer->desc.layout;
        binding.binding = 0;
        binding.stride = layout.stride();
        binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
        for (const ShaderAttribute& attribute : program.attributes) {
          static constexpr VkFormat kFormats[] = { VK_FORMAT_R32_SFLOAT, VK_FORMAT_R32_SFLOAT,
                                                   VK_FORMAT_R32G32_SFLOAT, VK_FORMAT_R32G32B32_SFLOAT,
                                                   VK_FORMAT_R32G32B32A32_SFLOAT };
          int count = layout.has(attribute.attrib) ? layout.count(attribute.attrib) : 1;
          uint32_t offset = layout.has(attribute.attrib) ? layout.offset(attribute.attrib) : 0;
          attributes.push_back({ static_cast<uint32_t>(attribute.attrib), 0, kFormats[std::clamp(count, 1, 4)], offset });
        }
        vertex_input.vertexBindingDescriptionCount = 1;
        vertex_input.pVertexBindingDescriptions = &binding;
        vertex_input.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributes.size());
        vertex_input.pVertexAttributeDescriptions = attributes.data();
      }

      VkPipelineInputAssemblyStateCreateInfo assembly { VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO };
      assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

      VkPipelineViewportStateCreateInfo viewport { VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO };
      viewport.viewportCount = 1;
      viewport.scissorCount = 1;

      VkPipelineRasterizationStateCreateInfo raster { VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
      raster.polygonMode = VK_POLYGON_MODE_FILL;
      raster.cullMode = VK_CULL_MODE_NONE;
      raster.frontFace = VK_FRONT_FACE_CLOCKWISE;
      raster.lineWidth = 1.0f;

      VkPipelineMultisampleStateCreateInfo multisample { VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO };
      multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

      VkPipelineDepthStencilStateCreateInfo depth { VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO };

      VkPipelineColorBlendAttachmentState blend {};
      blend.blendEnable = state.blend ? VK_TRUE : VK_FALSE;
      blend.srcColorBlendFactor = blendFactor(state.src_rgb);
      blend.dstColorBlendFactor = blendFactor(state.dst_rgb);
      blend.colorBlendOp = blendOp(state.op_rgb);
      blend.srcAlphaBlendFactor = blendFactor(state.src_alpha);
      blend.dstAlphaBlendFactor = blendFactor(state.dst_alpha);
      blend.alphaBlendOp = blendOp(state.op_alpha);
      blend.colorWriteMask = ((state.write_mask & kWriteR) ? VK_COLOR_COMPONENT_R_BIT : 0) |
                             ((state.write_mask & kWriteG) ? VK_COLOR_COMPONENT_G_BIT : 0) |
                             ((state.write_mask & kWriteB) ? VK_COLOR_COMPONENT_B_BIT : 0) |
                             ((state.write_mask & kWriteA) ? VK_COLOR_COMPONENT_A_BIT : 0);
      VkPipelineColorBlendStateCreateInfo color_blend { VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO };
      color_blend.attachmentCount = 1;
      color_blend.pAttachments = &blend;

      VkDynamicState dynamic_states[] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
      VkPipelineDynamicStateCreateInfo dynamic { VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO };
      dynamic.dynamicStateCount = 2;
      dynamic.pDynamicStates = dynamic_states;

      VkGraphicsPipelineCreateInfo info { VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
      info.stageCount = 2;
      info.pStages = stages;
      info.pVertexInputState = &vertex_input;
      info.pInputAssemblyState = &assembly;
      info.pViewportState = &viewport;
      info.pRasterizationState = &raster;
      info.pMultisampleState = &multisample;
      info.pDepthStencilState = &depth;
      info.pColorBlendState = &color_blend;
      info.pDynamicState = &dynamic;
      info.layout = graphics_pipeline_layout_;
      info.renderPass = renderPass(format);
      info.subpass = 0;

      VkPipeline pipeline = VK_NULL_HANDLE;
      if (vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &info, nullptr, &pipeline) != VK_SUCCESS) {
        log("Failed to create a graphics pipeline");
        pipeline = VK_NULL_HANDLE;
      }
      pipelines_[key] = pipeline;
      return pipeline;
    }

    VkDescriptorSet allocateDescriptorSet(VkDescriptorSetLayout layout) {
      FrameSlot& slot = slots_[slot_index_];
      while (true) {
        if (slot.descriptor_pool_index == slot.descriptor_pools.size()) {
          VkDescriptorPoolSize sizes[] = {
            { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, kDescriptorPoolSets * 2 },
            { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, kDescriptorPoolSets * 4 },
            { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, kDescriptorPoolSets },
            { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, kDescriptorPoolSets },
          };
          VkDescriptorPoolCreateInfo info { VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
          info.maxSets = kDescriptorPoolSets;
          info.poolSizeCount = 4;
          info.pPoolSizes = sizes;
          VkDescriptorPool pool = VK_NULL_HANDLE;
          if (vkCreateDescriptorPool(device_, &info, nullptr, &pool) != VK_SUCCESS)
            return VK_NULL_HANDLE;
          slot.descriptor_pools.push_back(pool);
        }

        VkDescriptorSetAllocateInfo info { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
        info.descriptorPool = slot.descriptor_pools[slot.descriptor_pool_index];
        info.descriptorSetCount = 1;
        info.pSetLayouts = &layout;
        VkDescriptorSet set = VK_NULL_HANDLE;
        VkResult result = vkAllocateDescriptorSets(device_, &info, &set);
        if (result == VK_SUCCESS)
          return set;
        if (result != VK_ERROR_OUT_OF_POOL_MEMORY && result != VK_ERROR_FRAGMENTED_POOL)
          return VK_NULL_HANDLE;
        slot.descriptor_pool_index++;
      }
    }

    VkDeviceSize uniformAlignment() const {
      return std::max<VkDeviceSize>(16, properties_.limits.minUniformBufferOffsetAlignment);
    }

    RingAllocation uploadUniforms(const FrameData& frame, uint32_t offset, uint32_t size) {
      if (size == 0)
        return {};
      RingAllocation allocation = ringAllocate(size, uniformAlignment());
      if (allocation.data)
        std::memcpy(allocation.data, frame.uniform_data.data() + offset, size);
      return allocation;
    }

    VkBuffer placeholderUniformBuffer() {
      if (placeholder_uniforms_.buffer == VK_NULL_HANDLE)
        createHostBuffer(256, ringUsage(), placeholder_uniforms_);
      return placeholder_uniforms_.buffer;
    }

    bool bindDrawDescriptors(const FrameData& frame, const DrawItem& draw) {
      RingAllocation vertex_uniforms = uploadUniforms(frame, draw.vertex_uniform_offset, draw.vertex_uniform_size);
      RingAllocation fragment_uniforms = uploadUniforms(frame, draw.fragment_uniform_offset,
                                                        draw.fragment_uniform_size);

      DescriptorKey key;
      key.vertex_buffer = vertex_uniforms.buffer ? vertex_uniforms.buffer : placeholderUniformBuffer();
      key.fragment_buffer = fragment_uniforms.buffer ? fragment_uniforms.buffer : placeholderUniformBuffer();
      key.vertex_size = std::max<uint32_t>(16, draw.vertex_uniform_size);
      key.fragment_size = std::max<uint32_t>(16, draw.fragment_uniform_size);
      for (int i = 0; i < kMaxTextureStages; ++i) {
        if (textureValid(draw.textures[i].texture))
          key.textures[i] = draw.textures[i];
      }

      FrameSlot& slot = slots_[slot_index_];
      VkDescriptorSet set = VK_NULL_HANDLE;
      auto found = slot.descriptor_cache.find(key);
      if (found != slot.descriptor_cache.end())
        set = found->second;
      else {
        set = allocateDescriptorSet(graphics_set_layout_);
        if (set == VK_NULL_HANDLE)
          return false;

        VkDescriptorBufferInfo buffers[2] = { { key.vertex_buffer, 0, key.vertex_size },
                                              { key.fragment_buffer, 0, key.fragment_size } };
        VkDescriptorImageInfo images[kMaxTextureStages] {};
        std::vector<VkWriteDescriptorSet> writes;
        for (uint32_t i = 0; i < 2; ++i) {
          VkWriteDescriptorSet write { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
          write.dstSet = set;
          write.dstBinding = i;
          write.descriptorCount = 1;
          write.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC;
          write.pBufferInfo = &buffers[i];
          writes.push_back(write);
        }
        for (int i = 0; i < kMaxTextureStages; ++i) {
          if (key.textures[i].texture == kInvalidHandle)
            continue;
          images[i].sampler = sampler(key.textures[i].sampler_flags);
          images[i].imageView = textures_[key.textures[i].texture].view;
          images[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
          VkWriteDescriptorSet write { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
          write.dstSet = set;
          write.dstBinding = kResourceBindingOffset + i;
          write.descriptorCount = 1;
          write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
          write.pImageInfo = &images[i];
          writes.push_back(write);
        }
        vkUpdateDescriptorSets(device_, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
        slot.descriptor_cache[key] = set;
      }

      uint32_t offsets[2] = { static_cast<uint32_t>(vertex_uniforms.offset),
                              static_cast<uint32_t>(fragment_uniforms.offset) };
      vkCmdBindDescriptorSets(command_buffer_, VK_PIPELINE_BIND_POINT_GRAPHICS, graphics_pipeline_layout_, 0, 1,
                              &set, 2, offsets);
      return true;
    }

    void prepareDrawTextures(const FrameData& frame, const ViewData& view, size_t start, size_t end) {
      for (size_t i = start; i < end; ++i) {
        const DrawItem& draw = frame.draws[view.items[i].index];
        for (const TextureBinding& binding : draw.textures) {
          if (textureValid(binding.texture))
            transition(textures_[binding.texture], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        }
      }
    }

    void executeDraws(const FrameData& frame, const ViewData& view, size_t start, size_t end,
                      FrameBuffer& frame_buffer) {
      prepareDrawTextures(frame, view, start, end);

      VkFramebuffer framebuffer = VK_NULL_HANDLE;
      VkExtent2D extent {};
      VkFormat format = frame_buffer.format;
      if (frame_buffer.swap_chain) {
        SwapChain& chain = *frame_buffer.chain;
        transitionSwapImage(chain, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
        framebuffer = chain.framebuffers[chain.current_image];
        extent = chain.extent;
      }
      else {
        Texture& texture = textures_[frame_buffer.texture];
        transition(texture, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
        framebuffer = frame_buffer.framebuffer;
        extent = { static_cast<uint32_t>(texture.desc.width), static_cast<uint32_t>(texture.desc.height) };
      }

      VkRenderPassBeginInfo begin { VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO };
      begin.renderPass = renderPass(format);
      begin.framebuffer = framebuffer;
      begin.renderArea = { { 0, 0 }, extent };
      vkCmdBeginRenderPass(command_buffer_, &begin, VK_SUBPASS_CONTENTS_INLINE);

      int x = std::clamp(view.x, 0, static_cast<int>(extent.width));
      int y = std::clamp(view.y, 0, static_cast<int>(extent.height));
      int width = std::clamp(view.width, 0, static_cast<int>(extent.width) - x);
      int height = std::clamp(view.height, 0, static_cast<int>(extent.height) - y);
      if (width > 0 && height > 0) {
        VkViewport viewport { static_cast<float>(view.x), static_cast<float>(view.y + view.height),
                              static_cast<float>(view.width), -static_cast<float>(view.height), 0.0f, 1.0f };
        VkRect2D scissor { { x, y }, { static_cast<uint32_t>(width), static_cast<uint32_t>(height) } };
        vkCmdSetViewport(command_buffer_, 0, 1, &viewport);
        vkCmdSetScissor(command_buffer_, 0, 1, &scissor);

        VkPipeline bound_pipeline = VK_NULL_HANDLE;
        for (size_t i = start; i < end; ++i) {
          const DrawItem& draw = frame.draws[view.items[i].index];
          if (draw.program >= programs_.size() || !programs_[draw.program].used)
            continue;

          const Buffer* vertex_buffer = nullptr;
          if (draw.vertex_buffer != kInvalidHandle) {
            if (draw.vertex_buffer >= buffers_.size() || !buffers_[draw.vertex_buffer].used)
              continue;
            vertex_buffer = &buffers_[draw.vertex_buffer];
          }

          VkPipeline pipeline = graphicsPipeline(draw.program, vertex_buffer, draw.state, format);
          if (pipeline == VK_NULL_HANDLE)
            continue;
          if (pipeline != bound_pipeline) {
            vkCmdBindPipeline(command_buffer_, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
            bound_pipeline = pipeline;
          }
          if (!bindDrawDescriptors(frame, draw))
            continue;

          if (vertex_buffer) {
            VkDeviceSize offset = 0;
            vkCmdBindVertexBuffers(command_buffer_, 0, 1, &vertex_buffer->buffer, &offset);
          }

          if (draw.index_buffer != kInvalidHandle && draw.index_buffer < buffers_.size() &&
              buffers_[draw.index_buffer].used) {
            const Buffer& index_buffer = buffers_[draw.index_buffer];
            VkIndexType type = (index_buffer.desc.flags & kBufferIndex32) ? VK_INDEX_TYPE_UINT32 :
                                                                            VK_INDEX_TYPE_UINT16;
            vkCmdBindIndexBuffer(command_buffer_, index_buffer.buffer, 0, type);
            vkCmdDrawIndexed(command_buffer_, draw.index_count, 1, draw.index_start,
                             static_cast<int32_t>(draw.vertex_start), 0);
          }
          else
            vkCmdDraw(command_buffer_, draw.vertex_count, 1, draw.vertex_start, 0);
        }
      }
      vkCmdEndRenderPass(command_buffer_);
    }

    void executeCompute(const FrameData& frame, const ComputeItem& item) {
      if (item.program >= programs_.size() || !programs_[item.program].used || !programs_[item.program].compute)
        return;
      Program& program = programs_[item.program];

      for (const ShaderStorage& storage : program.storage) {
        const ComputeBinding& binding = item.bindings[storage.stage];
        if (storage.image && textureValid(binding.handle))
          transition(textures_[binding.handle], VK_IMAGE_LAYOUT_GENERAL);
      }
      for (const ShaderSampler& shader_sampler : program.samplers) {
        const ComputeBinding& binding = item.bindings[shader_sampler.stage];
        if (binding.image && textureValid(binding.handle))
          transition(textures_[binding.handle], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
      }

      VkDescriptorSet set = allocateDescriptorSet(program.set_layout);
      if (set == VK_NULL_HANDLE)
        return;

      RingAllocation uniforms = uploadUniforms(frame, item.uniform_offset, item.uniform_size);
      std::vector<VkWriteDescriptorSet> writes;
      std::vector<VkDescriptorBufferInfo> buffer_infos;
      std::vector<VkDescriptorImageInfo> image_infos;
      buffer_infos.reserve(kMaxComputeBindings + 1);
      image_infos.reserve(kMaxComputeBindings * 2);

      if (program.compute_uniforms) {
        buffer_infos.push_back({ uniforms.buffer ? uniforms.buffer : placeholderUniformBuffer(), 0,
                                 std::max<VkDeviceSize>(16, item.uniform_size) });
        VkWriteDescriptorSet write { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        write.dstSet = set;
        write.dstBinding = 0;
        write.descriptorCount = 1;
        write.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC;
        write.pBufferInfo = &buffer_infos.back();
        writes.push_back(write);
      }

      for (const ShaderStorage& storage : program.storage) {
        const ComputeBinding& binding = item.bindings[storage.stage];
        VkWriteDescriptorSet write { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        write.dstSet = set;
        write.dstBinding = kResourceBindingOffset + storage.stage;
        write.descriptorCount = 1;
        if (storage.image) {
          if (!textureValid(binding.handle))
            return;
          image_infos.push_back({ VK_NULL_HANDLE, textures_[binding.handle].view, VK_IMAGE_LAYOUT_GENERAL });
          write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
          write.pImageInfo = &image_infos.back();
        }
        else {
          if (binding.handle >= buffers_.size() || !buffers_[binding.handle].used)
            return;
          buffer_infos.push_back({ buffers_[binding.handle].buffer, 0, VK_WHOLE_SIZE });
          write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
          write.pBufferInfo = &buffer_infos.back();
        }
        writes.push_back(write);
      }

      for (const ShaderSampler& shader_sampler : program.samplers) {
        const ComputeBinding& binding = item.bindings[shader_sampler.stage];
        if (!textureValid(binding.handle))
          return;
        image_infos.push_back({ sampler(textures_[binding.handle].desc.flags), textures_[binding.handle].view,
                                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL });
        VkWriteDescriptorSet write { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        write.dstSet = set;
        write.dstBinding = kResourceBindingOffset + shader_sampler.stage;
        write.descriptorCount = 1;
        write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        write.pImageInfo = &image_infos.back();
        writes.push_back(write);
      }

      vkUpdateDescriptorSets(device_, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
      vkCmdBindPipeline(command_buffer_, VK_PIPELINE_BIND_POINT_COMPUTE, program.compute_pipeline);
      uint32_t offset = static_cast<uint32_t>(uniforms.offset);
      vkCmdBindDescriptorSets(command_buffer_, VK_PIPELINE_BIND_POINT_COMPUTE, program.pipeline_layout, 0, 1, &set,
                              program.compute_uniforms ? 1 : 0, &offset);
      vkCmdDispatch(command_buffer_, item.groups[0], item.groups[1], item.groups[2]);
      barrier();
    }

    void executeBlit(const BlitItem& item) {
      if (!textureValid(item.source) || !textureValid(item.destination))
        return;
      Texture& source = textures_[item.source];
      Texture& destination = textures_[item.destination];
      transition(source, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
      transition(destination, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
      VkImageCopy region {};
      region.srcSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
      region.srcOffset = { item.source_x, item.source_y, 0 };
      region.dstSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
      region.dstOffset = { item.dest_x, item.dest_y, 0 };
      region.extent = { static_cast<uint32_t>(item.width), static_cast<uint32_t>(item.height), 1 };
      vkCmdCopyImage(command_buffer_, source.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, destination.image,
                     VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
      transition(destination, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    }

    void executeView(const FrameData& frame, const ViewData& view, FrameBuffer* frame_buffer) {
      size_t i = 0;
      while (i < view.items.size()) {
        const ViewItem& item = view.items[i];
        if (item.type == ViewItem::Draw) {
          size_t end = i;
          while (end < view.items.size() && view.items[end].type == ViewItem::Draw)
            ++end;
          if (frame_buffer)
            executeDraws(frame, view, i, end, *frame_buffer);
          i = end;
        }
        else {
          if (item.type == ViewItem::Compute)
            executeCompute(frame, frame.computes[item.index]);
          else
            executeBlit(frame.blits[item.index]);
          ++i;
        }
      }
    }

    ReadBack readImage(VkImage image, VkImageLayout& layout, int width, int height, uint32_t bytes) {
      ReadBack read;
      read.row_bytes = width * bytes;
      read.rows = height;
      read.width = width;
      read.height = height;
      if (!createHostBuffer(static_cast<VkDeviceSize>(read.row_bytes) * height, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                            read.buffer))
        return read;

      transitionImage(image, layout, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
      VkBufferImageCopy region {};
      region.imageSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
      region.imageExtent = { static_cast<uint32_t>(width), static_cast<uint32_t>(height), 1 };
      vkCmdCopyImageToBuffer(command_buffer_, image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, read.buffer.buffer, 1,
                             &region);
      barrier();
      return read;
    }

    ReadBack readTexture(Texture& texture, void* destination) {
      ReadBack read = readImage(texture.image, texture.layout, texture.desc.width, texture.desc.height,
                                texture.bytes);
      read.destination = destination;
      read.bgra = texture.desc.format == TextureFormat::BGRA8;
      transition(texture, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
      return read;
    }

    ReadBack readSwapChain(uint16_t idx, SwapChain& chain) {
      ReadBack read = readImage(chain.images[chain.current_image], chain.layouts[chain.current_image],
                                static_cast<int>(chain.extent.width), static_cast<int>(chain.extent.height), 4);
      read.screenshot = true;
      read.frame_buffer = idx;
      read.bgra = chain.format == VK_FORMAT_B8G8R8A8_UNORM;
      return read;
    }

    void finishRead(ReadBack& read) {
      if (read.buffer.allocation.mapped == nullptr)
        return;
      auto data = static_cast<const uint8_t*>(read.buffer.allocation.mapped);
      if (read.destination)
        std::memcpy(read.destination, data, static_cast<size_t>(read.row_bytes) * read.rows);
      if (read.screenshot && init_.screenshot_callback)
        init_.screenshot_callback({ read.frame_buffer }, data, read.width, read.height, read.row_bytes, read.bgra);
    }

    Init init_;
    Caps caps_;
    void* library_ = nullptr;
    VkInstance instance_ = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT debug_messenger_ = VK_NULL_HANDLE;
    VkPhysicalDevice physical_device_ = VK_NULL_HANDLE;
    VkPhysicalDeviceProperties properties_ {};
    VkPhysicalDeviceMemoryProperties memory_properties_ {};
    uint32_t queue_family_ = 0;
    uint32_t num_allocations_ = 0;
    bool device_lost_ = false;
    VkDevice device_ = VK_NULL_HANDLE;
    VkQueue queue_ = VK_NULL_HANDLE;
    VkDescriptorSetLayout graphics_set_layout_ = VK_NULL_HANDLE;
    VkPipelineLayout graphics_pipeline_layout_ = VK_NULL_HANDLE;
    HostBuffer placeholder_uniforms_;

    FrameSlot slots_[kFramesInFlight];
    int slot_index_ = 0;
    VkCommandBuffer command_buffer_ = VK_NULL_HANDLE;

    std::vector<Texture> textures_;
    std::vector<Buffer> buffers_;
    std::vector<Shader> shaders_;
    std::vector<Program> programs_;
    std::vector<FrameBuffer> frame_buffers_;
    std::vector<PendingInit> pending_inits_;

    std::unordered_map<PipelineKey, VkPipeline, PipelineKeyHash> pipelines_;
    std::unordered_map<uint32_t, VkSampler> samplers_;
    std::unordered_map<VkFormat, VkRenderPass> render_passes_;
  };

  std::unique_ptr<Backend> createVulkanBackend() {
    return std::make_unique<VulkanBackend>();
  }
}
