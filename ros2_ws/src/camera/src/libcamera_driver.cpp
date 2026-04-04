#include "camera/libcamera_driver.hpp"
#include "camera/image_converter.hpp"

#include <iostream>
#include <sys/mman.h>
#include <vector>

LibcameraDriver::LibcameraDriver()
: initialized_(false),
  running_(false),
  width_(640),
  height_(480),
  fps_(30),
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
    fps_ = fps;

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

    libcamera::CameraConfiguration::Status validation = config_->validate();
    if (validation == libcamera::CameraConfiguration::Invalid) {
        std::cerr << "Konfiguration ist ungueltig\n";
        return false;
    }

    width_ = static_cast<int>(cfg.size.width);
    height_ = static_cast<int>(cfg.size.height);

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

    if (camera_->start() != 0) {
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

    void* y_mem = mmap(nullptr, planes[0].length, PROT_READ, MAP_SHARED, planes[0].fd.get(), 0);
    void* u_mem = mmap(nullptr, planes[1].length, PROT_READ, MAP_SHARED, planes[1].fd.get(), 0);
    void* v_mem = mmap(nullptr, planes[2].length, PROT_READ, MAP_SHARED, planes[2].fd.get(), 0);

    if (y_mem == MAP_FAILED || u_mem == MAP_FAILED || v_mem == MAP_FAILED) {
        if (y_mem != MAP_FAILED) {
            munmap(y_mem, planes[0].length);
        }
        if (u_mem != MAP_FAILED) {
            munmap(u_mem, planes[1].length);
        }
        if (v_mem != MAP_FAILED) {
            munmap(v_mem, planes[2].length);
        }

        request->reuse(libcamera::Request::ReuseBuffers);
        camera_->queueRequest(request);
        return false;
    }

    const uint8_t* y_plane = static_cast<const uint8_t*>(y_mem);
    const uint8_t* u_plane = static_cast<const uint8_t*>(u_mem);
    const uint8_t* v_plane = static_cast<const uint8_t*>(v_mem);

    const int y_stride = width_;
    const int uv_stride = width_ / 2;

    ImageConverter::yuv420_to_rgb(
        y_plane,
        u_plane,
        v_plane,
        width_,
        height_,
        y_stride,
        uv_stride,
        data
    );

    munmap(y_mem, planes[0].length);
    munmap(u_mem, planes[1].length);
    munmap(v_mem, planes[2].length);

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

    allocator_.reset();
    config_.reset();

    if (camera_manager_) {
        camera_manager_->stop();
        camera_manager_.reset();
    }

    initialized_ = false;
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