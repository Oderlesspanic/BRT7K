#pragma once

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>

#include <libcamera/libcamera.h>

class LibcameraDriver
{
public:
    LibcameraDriver();
    ~LibcameraDriver();

    bool initialize(int width, int height, int fps);
    bool start();
    bool capture_frame(std::vector<uint8_t>& data, uint64_t& timestamp_ns);
    void stop();
    int width() const;
    int height() const;

private:
    void request_complete(libcamera::Request *request);

    bool initialized_;
    bool running_;

    int width_;
    int height_;
    int fps_;
    int y_stride_;
    int uv_stride_;

    std::unique_ptr<libcamera::CameraManager> camera_manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    libcamera::Stream* stream_;

    std::vector<std::unique_ptr<libcamera::Request>> requests_;

    std::mutex completed_mutex_;
    std::condition_variable completed_cv_;
    std::queue<libcamera::Request*> completed_requests_;
};
