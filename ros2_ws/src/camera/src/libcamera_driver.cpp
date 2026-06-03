#include "camera/libcamera_driver.hpp"
#include "camera/image_converter.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <iostream>
#include <sys/mman.h>
#include <vector>

#include <libcamera/control_ids.h>

namespace
{
LibcameraDriver::MappedPlane map_plane(const libcamera::FrameBuffer::Plane &plane)
{
    const size_t offset = static_cast<size_t>(plane.offset);
    const size_t length = static_cast<size_t>(plane.length);
    LibcameraDriver::MappedPlane mapped;

    mapped.mapped_length = offset + length;
    mapped.base = mmap(nullptr, mapped.mapped_length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
    if (mapped.base == MAP_FAILED) {
        mapped.base = nullptr;
        mapped.mapped_length = 0;
        return mapped;
    }

    mapped.data = static_cast<const uint8_t*>(mapped.base) + offset;
    return mapped;
}
}

LibcameraDriver::LibcameraDriver()
: initialized_(false),
  running_(false),
  width_(640),
  height_(480),
  fps_(30),
  y_stride_(640),
  uv_stride_(320),
  stream_(nullptr)
{
}

LibcameraDriver::~LibcameraDriver()
{
    stop();
}

bool LibcameraDriver::initialize(int width, int height, int fps)
{
    width_ = width;
    height_ = height;
    fps_ = std::max(1, fps);

    camera_manager_ = std::make_unique<libcamera::CameraManager>();
    if (camera_manager_->start() != 0) {
        std::cerr << "CameraManager start fehlgeschlagen\n";
        return false;
    }

    if (camera_manager_->cameras().empty()) {
        std::cerr << "Keine Kamera gefunden\n";
        return false;
    }

    camera_ = camera_manager_->cameras()[0];

    if (camera_->acquire() != 0) {
        std::cerr << "camera acquire fehlgeschlagen\n";
        return false;
    }

    std::vector<libcamera::StreamRole> roles = { libcamera::StreamRole::Viewfinder };
    config_ = camera_->generateConfiguration(roles);
    if (!config_ || config_->size() == 0) {
        std::cerr << "generateConfiguration fehlgeschlagen\n";
        return false;
    }

    libcamera::StreamConfiguration &cfg = config_->at(0);
    cfg.size.width = width_;
    cfg.size.height = height_;
    cfg.pixelFormat = libcamera::formats::YUV420;
    cfg.bufferCount = 2;

    libcamera::CameraConfiguration::Status validation = config_->validate();
    if (validation == libcamera::CameraConfiguration::Invalid) {
        std::cerr << "Konfiguration ist ungueltig\n";
        return false;
    }

    width_ = static_cast<int>(cfg.size.width);
    height_ = static_cast<int>(cfg.size.height);
    y_stride_ = static_cast<int>(cfg.stride);
    if (y_stride_ <= 0) {
        y_stride_ = width_;
    }
    uv_stride_ = y_stride_ / 2;

    if (camera_->configure(config_.get()) != 0) {
        std::cerr << "camera configure fehlgeschlagen\n";
        return false;
    }

    stream_ = cfg.stream();
    if (stream_ == nullptr) {
        std::cerr << "Kein Stream verfuegbar\n";
        return false;
    }

    allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);

    if (allocator_->allocate(stream_) < 0) {
        std::cerr << "Buffer allocation fehlgeschlagen\n";
        return false;
    }

    const auto &buffers = allocator_->buffers(stream_);
    if (buffers.empty()) {
        std::cerr << "Keine Buffer alloziert\n";
        return false;
    }

    if (!map_buffers()) {
        std::cerr << "Buffer mapping fehlgeschlagen\n";
        return false;
    }

    requests_.clear();
    for (const std::unique_ptr<libcamera::FrameBuffer> &buffer : buffers) {
        std::unique_ptr<libcamera::Request> request = camera_->createRequest();
        if (!request) {
            std::cerr << "createRequest fehlgeschlagen\n";
            return false;
        }

        if (request->addBuffer(stream_, buffer.get()) < 0) {
            std::cerr << "addBuffer fehlgeschlagen\n";
            return false;
        }

        requests_.push_back(std::move(request));
    }

    camera_->requestCompleted.connect(this, &LibcameraDriver::request_complete);

    initialized_ = true;
    return true;
}

bool LibcameraDriver::start()
{
    if (!initialized_) {
        return false;
    }

    libcamera::ControlList controls(camera_->controls());
    const int64_t frame_duration_us = 1000000 / std::max(1, fps_);
    const std::array<int64_t, 2> frame_duration_limits{
        frame_duration_us,
        frame_duration_us
    };
    controls.set(libcamera::controls::FrameDurationLimits,
        libcamera::Span<const int64_t, 2>(frame_duration_limits));

    if (camera_->start(&controls) != 0) {
        std::cerr << "camera start fehlgeschlagen\n";
        return false;
    }

    for (auto &request : requests_) {
        if (camera_->queueRequest(request.get()) < 0) {
            std::cerr << "queueRequest fehlgeschlagen\n";
            return false;
        }
    }

    running_ = true;
    return true;
}

bool LibcameraDriver::capture_frame(std::vector<uint8_t>& data, uint64_t& timestamp_ns)
{
    if (!running_) {
        return false;
    }

    libcamera::Request* request = nullptr;

    {
        std::unique_lock<std::mutex> lock(completed_mutex_);
        completed_cv_.wait(lock, [this]() { return !completed_requests_.empty(); });
        request = completed_requests_.front();
        completed_requests_.pop();
    }

    if (request == nullptr) {
        return false;
    }

    const auto &buffer_map = request->buffers();
    auto it = buffer_map.find(stream_);
    if (it == buffer_map.end()) {
        request->reuse(libcamera::Request::ReuseBuffers);
        camera_->queueRequest(request);
        return false;
    }

    libcamera::FrameBuffer* buffer = it->second;
    const auto &planes = buffer->planes();

    if (planes.size() < 3) {
        request->reuse(libcamera::Request::ReuseBuffers);
        camera_->queueRequest(request);
        return false;
    }

    const auto mapped_it = mapped_buffers_.find(buffer);
    if (mapped_it == mapped_buffers_.end() || mapped_it->second.size() < 3) {
        request->reuse(libcamera::Request::ReuseBuffers);
        camera_->queueRequest(request);
        return false;
    }

    const auto &mapped_planes = mapped_it->second;
    ImageConverter::yuv420_to_bgr(
        mapped_planes[0].data,
        mapped_planes[1].data,
        mapped_planes[2].data,
        width_,
        height_,
        y_stride_,
        uv_stride_,
        data
    );

    timestamp_ns = 0;
    if (buffer->metadata().timestamp) {
        timestamp_ns = buffer->metadata().timestamp;
    }

    request->reuse(libcamera::Request::ReuseBuffers);
    if (camera_->queueRequest(request) < 0) {
        return false;
    }

    return true;
}

int LibcameraDriver::width() const
{
    return width_;
}

int LibcameraDriver::height() const
{
    return height_;
}

void LibcameraDriver::stop()
{
    if (camera_) {
        if (running_) {
            camera_->stop();
            running_ = false;
        }

        camera_->requestCompleted.disconnect(this, &LibcameraDriver::request_complete);
        camera_->release();
        camera_.reset();
    }

    unmap_buffers();
    allocator_.reset();
    config_.reset();

    if (camera_manager_) {
        camera_manager_->stop();
        camera_manager_.reset();
    }

    initialized_ = false;
}

bool LibcameraDriver::map_buffers()
{
    unmap_buffers();

    const auto &buffers = allocator_->buffers(stream_);
    for (const auto &buffer : buffers) {
        std::vector<MappedPlane> mapped_planes;
        mapped_planes.reserve(buffer->planes().size());

        for (const auto &plane : buffer->planes()) {
            MappedPlane mapped = map_plane(plane);
            if (mapped.base == nullptr || mapped.data == nullptr) {
                unmap_buffers();
                return false;
            }

            mapped_planes.push_back(mapped);
        }

        mapped_buffers_[buffer.get()] = std::move(mapped_planes);
    }

    return true;
}

void LibcameraDriver::unmap_buffers()
{
    for (auto &[buffer, planes] : mapped_buffers_) {
        (void)buffer;
        for (auto &plane : planes) {
            if (plane.base != nullptr) {
                munmap(plane.base, plane.mapped_length);
            }
            plane = {};
        }
    }

    mapped_buffers_.clear();
}

void LibcameraDriver::request_complete(libcamera::Request *request)
{
    if (request->status() == libcamera::Request::RequestCancelled) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(completed_mutex_);
        completed_requests_.push(request);
    }

    completed_cv_.notify_one();
}
