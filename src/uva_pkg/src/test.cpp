#include "test.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <im2d.hpp>
#include <rga.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/core/core.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <string>
#include <utility>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <unistd.h>
#include "image_drawing.h"
#include "image_utils.h"


#include "yolov5.h"

namespace
{
constexpr std::uint32_t kRgaRgbStrideAlignment = 16;
constexpr const char * kDefaultCaptureDevice = "/dev/video9";
constexpr const char * kPreviewWindowName = "uva_rgb_preview";

bool file_exists(const std::string & path)
{
    return !path.empty() && access(path.c_str(), F_OK) == 0;
}

std::string join_path(const std::string & base, const std::string & relative)
{
    if (base.empty()) {
        return relative;
    }

    if (base.back() == '/') {
        return base + relative;
    }

    return base + "/" + relative;
}

std::string resolve_default_asset_path(const std::string & relative_path)
{
    std::vector<std::string> candidates;

    try {
        candidates.push_back(join_path(
            ament_index_cpp::get_package_share_directory("uva_pkg"),
            relative_path));
    } catch (const std::exception &) {
        // Fall back to source-tree and legacy container locations below.
    }

#ifdef UVA_PKG_SOURCE_DIR
    candidates.push_back(join_path(UVA_PKG_SOURCE_DIR, relative_path));
#endif
    candidates.push_back(join_path("/app", relative_path));

    for (const auto & candidate : candidates) {
        if (file_exists(candidate)) {
            return candidate;
        }
    }

    return candidates.empty() ? relative_path : candidates.front();
}
}  // namespace

namespace test_space
{
Test::Test(rclcpp::Node::SharedPtr node)
: node_(std::move(node))
{
    publisher_ = node_->create_publisher<std_msgs::msg::String>("topic", 10);
    save_test_publisher_ = node_->create_publisher<std_msgs::msg::String>("save_test", 10);
    image_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>("rgb_image", 10);
    std::memset(&rknn_app_ctx_, 0, sizeof(rknn_app_ctx_));

    model_path_ = node_->declare_parameter<std::string>(
        "model_path",
        resolve_default_asset_path("model/best.rknn"));
    label_path_ = node_->declare_parameter<std::string>(
        "label_path",
        resolve_default_asset_path("model/apple.txt"));
    capture_device_ = node_->declare_parameter<std::string>(
        "capture_device",
        kDefaultCaptureDevice);
    enable_preview_ = node_->declare_parameter<bool>("enable_preview", true);

    RCLCPP_INFO(
        node_->get_logger(),
        "Test pipeline initialized. model=%s label=%s capture_device=%s preview=%s",
        model_path_.c_str(),
        label_path_.c_str(),
        capture_device_.c_str(),
        enable_preview_ ? "true" : "false");

    initialize_detection();
}

Test::~Test()
{
    stop();
    shutdown_detection();
    RCLCPP_INFO(node_->get_logger(), "Test pipeline destroyed.");
}

void Test::start()
{
    if (running_.exchange(true)) {
        return;
    }

    if (!capture_initialize()) {
        running_ = false;
        return;
    }
    initialize_frame_pool(8);

    capture_thread_ = std::thread(&Test::capture_loop, this);
    detect_thread_ = std::thread(&Test::detect_loop, this);
    save_thread_ = std::thread(&Test::save_loop, this);

    RCLCPP_INFO(node_->get_logger(), "Capture, detect, and save threads started.");
}

void Test::stop()
{
    if (!running_.exchange(false)) {
        return;
    }

    frame_pool_cv_.notify_all();
    detect_cv_.notify_all();
    save_cv_.notify_all();

    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    if (detect_thread_.joinable()) {
        detect_thread_.join();
    }
    if (save_thread_.joinable()) {
        save_thread_.join();
    }

    capture_cleanup();
    if (enable_preview_) {
        try {
            cv::destroyWindow(kPreviewWindowName);
        } catch (const cv::Exception &) {
            // Ignore GUI teardown errors when no display backend is available.
        }
    }

    RCLCPP_INFO(node_->get_logger(), "Worker threads stopped.");
}

void Test::initialize_frame_pool(std::size_t frame_count)
{
    std::lock_guard<std::mutex> lock(frame_pool_mutex_);

    std::queue<FramePtr> empty_queue;
    std::swap(free_frames_, empty_queue);

    frame_storage_.clear();
    frame_storage_.reserve(frame_count);

    for (std::size_t i = 0; i < frame_count; ++i) {
        auto frame = std::make_shared<Frame>();
        frame->width  = capture_width_;
        frame->height = capture_height_;
        frame->stride = capture_stride_;
        frame->pixel_format = capture_pixel_format_;
        frame->data_size = frame_bytes_;
        frame->data.resize(frame_bytes_);
        frame->converted_stride = converted_stride_pixels_;
        frame->converted_pixel_format = converted_pixel_format_;
        frame->converted_data_size = converted_frame_bytes_;
        frame->converted_data.resize(converted_frame_bytes_);
        frame_storage_.push_back(frame);
        free_frames_.push(frame);
    }

    next_frame_id_ = 0;
}

Test::FramePtr Test::acquire_frame()
{
    std::unique_lock<std::mutex> lock(frame_pool_mutex_);

    frame_pool_cv_.wait(lock, [this]() {
        return !running_ || !free_frames_.empty();
    });

    if (free_frames_.empty()) {
        return nullptr;
    }

    auto frame = free_frames_.front();
    free_frames_.pop();
    return frame;
}

void Test::recycle_frame(const FramePtr & frame)
{
    if (!frame) {
        return;
    }

    std::lock_guard<std::mutex> lock(frame_pool_mutex_);
    free_frames_.push(frame);
    frame_pool_cv_.notify_one();
}

void Test::dispatch_frame(const FramePtr & frame)
{
    if (!frame) {
        return;
    }

    bool notify_detect = false;
    bool notify_save = false;

    frame->ref_count.store(0);

    {
        std::lock_guard<std::mutex> lock(detect_mutex_);
        frame->ref_count.fetch_add(1);
        detect_queue_.push(frame);
        notify_detect = true; 
    }

    {
        std::lock_guard<std::mutex> lock(save_mutex_);
        frame->ref_count.fetch_add(1);
        save_queue_.push(frame);
        notify_save = true;
    }



    if (notify_detect) {
        detect_cv_.notify_one();
    }
    if (notify_save) {
        save_cv_.notify_one();
    }
}

Test::FramePtr Test::wait_and_pop_frame(
    std::queue<FramePtr> & queue,
    std::mutex & queue_mutex,
    std::condition_variable & queue_cv)
{
    std::unique_lock<std::mutex> lock(queue_mutex);

    queue_cv.wait(lock, [this, &queue]() {
        return !running_ || !queue.empty();
    });

    if (queue.empty()) {
        return nullptr;
    }

    auto frame = queue.front();
    queue.pop();
    return frame;
}

void Test::release_frame_reference(const FramePtr & frame)
{
    if (!frame) {
        return;
    }

    if (frame->ref_count.fetch_sub(1) == 1) {
        recycle_frame(frame);
    }
}

std::uint32_t Test::align_up(std::uint32_t value, std::uint32_t alignment)
{
    if (alignment == 0U) {
        return value;
    }
    return ((value + alignment - 1U) / alignment) * alignment;
}

void Test::configure_rga_output_layout()
{
    converted_width_ = capture_width_;
    converted_height_ = capture_height_;
    converted_stride_pixels_ = align_up(capture_width_, kRgaRgbStrideAlignment);
    converted_row_bytes_ = converted_stride_pixels_ * 3U;
    converted_pixel_format_ = RK_FORMAT_RGB_888;
    converted_frame_bytes_ = static_cast<std::size_t>(converted_row_bytes_) * converted_height_;
}

bool Test::capture_initialize()
{
    m_fd = open(capture_device_.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (m_fd < 0) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Failed to open %s: %s",
            capture_device_.c_str(),
            std::strerror(errno));
        return false;
    }

    struct v4l2_format fmt;
    std::memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = capture_width_;
    fmt.fmt.pix.height = capture_height_;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(m_fd, VIDIOC_S_FMT, &fmt) < 0) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "VIDIOC_S_FMT failed on %s: %s",
            capture_device_.c_str(),
            std::strerror(errno));
        capture_cleanup();
        return false;
    }

    capture_width_ = fmt.fmt.pix.width;
    capture_height_ = fmt.fmt.pix.height;
    capture_stride_ = fmt.fmt.pix.bytesperline != 0 ? fmt.fmt.pix.bytesperline : fmt.fmt.pix.width * 2U;
    capture_pixel_format_ = fmt.fmt.pix.pixelformat;
    frame_bytes_ = fmt.fmt.pix.sizeimage != 0 ? fmt.fmt.pix.sizeimage : capture_stride_ * capture_height_;

    if (capture_pixel_format_ != V4L2_PIX_FMT_YUYV) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Camera did not accept YUYV capture format on %s.",
            capture_device_.c_str());
        capture_cleanup();
        return false;
    }

    configure_rga_output_layout();
    struct v4l2_requestbuffers req;
    std::memset(&req, 0, sizeof(req));
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(m_fd, VIDIOC_REQBUFS, &req) < 0) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "VIDIOC_REQBUFS failed: %s",
            std::strerror(errno));
        capture_cleanup();
        return false;
    }

    m_buffer_count = static_cast<int>(req.count);
    m_buffers = new Buffer[m_buffer_count];

    for (int i = 0; i < m_buffer_count; ++i) {
        struct v4l2_buffer buf;
        std::memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = static_cast<unsigned int>(i);

        if (ioctl(m_fd, VIDIOC_QUERYBUF, &buf) < 0) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "VIDIOC_QUERYBUF failed: %s",
                std::strerror(errno));
            capture_cleanup();
            return false;
        }

        m_buffers[i].length = buf.length;
        m_buffers[i].start = mmap(
            nullptr,
            buf.length,
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            m_fd,
            buf.m.offset);

        if (m_buffers[i].start == MAP_FAILED) {
            m_buffers[i].start = nullptr;
            RCLCPP_ERROR(node_->get_logger(), "mmap failed: %s", std::strerror(errno));
            capture_cleanup();
            return false;
        }

        if (ioctl(m_fd, VIDIOC_QBUF, &buf) < 0) {
            RCLCPP_ERROR(node_->get_logger(), "VIDIOC_QBUF failed: %s", std::strerror(errno));
            capture_cleanup();
            return false;
        }
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(m_fd, VIDIOC_STREAMON, &type) < 0) {
        RCLCPP_ERROR(node_->get_logger(), "VIDIOC_STREAMON failed: %s", std::strerror(errno));
        capture_cleanup();
        return false;
    }

    capture_ready_ = true;
    RCLCPP_INFO(
        node_->get_logger(),
        "V4L2 capture ready on %s: %ux%u stride=%u bytes=%zu format=YUYV, RGA RGB stride=%u row_bytes=%u",
        capture_device_.c_str(),
        capture_width_,
        capture_height_,
        capture_stride_,
        frame_bytes_,
        converted_stride_pixels_,
        converted_row_bytes_);
    return true;
}

void Test::capture_loop()
{
    while (running_ && rclcpp::ok()) {
        auto frame = acquire_frame();
        if (!frame) {
            continue;
        }

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(m_fd, &fds);
        struct timeval tv = {2, 0};

        const int ready = select(m_fd + 1, &fds, nullptr, nullptr, &tv);
        if (ready < 0) {
            if (errno == EINTR) {
                recycle_frame(frame);
                continue;
            }
            RCLCPP_ERROR(node_->get_logger(), "select failed: %s", std::strerror(errno));
            recycle_frame(frame);
            continue;
        }
        if (ready == 0) {
            RCLCPP_WARN(node_->get_logger(), "Waiting for camera frame timed out.");
            recycle_frame(frame);
            continue;
        }

        struct v4l2_buffer buf;
        std::memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(m_fd, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EAGAIN) {
                recycle_frame(frame);
                continue;
            }
            RCLCPP_ERROR(node_->get_logger(), "VIDIOC_DQBUF failed: %s", std::strerror(errno));
            recycle_frame(frame);
            continue;
        }

        frame->frame_id = ++next_frame_id_;
        frame->stamp = node_->get_clock()->now();
        frame->width = capture_width_;
        frame->height = capture_height_;
        frame->stride = capture_stride_;
        frame->pixel_format = capture_pixel_format_;
        frame->data_size = std::min<std::size_t>(buf.bytesused, frame->data.size());
        frame->converted_stride = converted_stride_pixels_;
        frame->converted_pixel_format = converted_pixel_format_;
        frame->converted_data_size = converted_frame_bytes_;
        frame->ref_count.store(0);
        std::memcpy(frame->data.data(), m_buffers[buf.index].start, frame->data_size);

        if (ioctl(m_fd, VIDIOC_QBUF, &buf) < 0) {
            RCLCPP_ERROR(node_->get_logger(), "VIDIOC_QBUF failed: %s", std::strerror(errno));
            recycle_frame(frame);
            continue;
        }

        dispatch_frame(frame);

        if (frame->frame_id % 30 == 0) {
            publish_status("capture frame " + std::to_string(frame->frame_id));
        }
    }
}

void Test::capture_cleanup()
{
    if (!capture_ready_) {
        if (m_fd >= 0) {
            close(m_fd);
            m_fd = -1;
        }
        return;
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(m_fd, VIDIOC_STREAMOFF, &type);

    for (int i = 0; i < m_buffer_count; ++i) {
        if (m_buffers != nullptr && m_buffers[i].start != nullptr) {
            munmap(m_buffers[i].start, m_buffers[i].length);
        }
    }
    delete[] m_buffers;
    m_buffers = nullptr;
    m_buffer_count = 0;

    if (m_fd >= 0) {
        close(m_fd);
        m_fd = -1;
    }

    capture_ready_ = false;
}

bool Test::convert_frame_to_rgb(const FramePtr & frame)
{
    if (!frame || frame->data_size == 0U || frame->converted_data.empty()) {
        return false;
    }

    if ((frame->stride % 2U) != 0U) {
        RCLCPP_ERROR(node_->get_logger(), "Unexpected YUYV stride=%u, cannot derive RGA pixel stride.", frame->stride);
        return false;
    }

    rga_buffer_t src = wrapbuffer_virtualaddr(
        frame->data.data(),
        static_cast<int>(frame->width),
        static_cast<int>(frame->height),
        RK_FORMAT_YUYV_422,
        static_cast<int>(frame->stride / 2U),
        static_cast<int>(frame->height));

    rga_buffer_t dst = wrapbuffer_virtualaddr(
        frame->converted_data.data(),
        static_cast<int>(converted_width_),
        static_cast<int>(converted_height_),
        RK_FORMAT_RGB_888,
        static_cast<int>(frame->converted_stride),
        static_cast<int>(converted_height_));

    const IM_STATUS status = imcvtcolor(
        src,
        dst,
        RK_FORMAT_YUYV_422,
        RK_FORMAT_RGB_888,
        IM_YUV_TO_RGB_BT601_LIMIT);

    if (status != IM_STATUS_SUCCESS) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "imcvtcolor(YUYV->RGB888) failed for frame %llu, status=%d",
            static_cast<unsigned long long>(frame->frame_id),
            static_cast<int>(status));
        return false;
    }

    frame->converted_pixel_format = converted_pixel_format_;
    frame->converted_data_size = converted_frame_bytes_;
    return true;
}

void Test::detect_loop()
{
    while (running_ && rclcpp::ok()) {
        auto frame = wait_and_pop_frame(detect_queue_, detect_mutex_, detect_cv_);
        if (!frame) {
            continue;
        }

        if (frame->pixel_format != V4L2_PIX_FMT_YUYV || frame->data_size == 0U) {
            RCLCPP_WARN(node_->get_logger(), "Unsupported frame format for RGA conversion.");
            release_frame_reference(frame);
            continue;
        }

        if (!convert_frame_to_rgb(frame)) {
            release_frame_reference(frame);
            continue;
        }


        cv::Mat rgb_frame(
                    static_cast<int>(frame->height),
                    static_cast<int>(frame->width),
                    CV_8UC3,
                    frame->converted_data.data(),
                    static_cast<std::size_t>(frame->converted_stride) * 3U);
        //使用RGB  MAT类型图像进行目标检测
        image_buffer_t src_image;
        memset(&src_image, 0, sizeof(image_buffer_t));
        src_image.width = rgb_frame.cols;
        src_image.height = rgb_frame.rows;
        src_image.width_stride = static_cast<int>(frame->converted_stride);
        src_image.height_stride = rgb_frame.rows;
        src_image.format = (rgb_frame.channels() == 1) ? IMAGE_FORMAT_GRAY8 : IMAGE_FORMAT_RGB888;
        src_image.virt_addr = rgb_frame.data;
        src_image.size = static_cast<int>(frame->converted_data_size);
        src_image.fd = -1;

        if (detection_ready_) {
            object_detect_result_list od_results;
            const int ret = inference_yolov5_model(&rknn_app_ctx_, &src_image, &od_results);
            if (ret != 0) {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "inference_yolov5_model failed on frame %llu: ret=%d",
                    static_cast<unsigned long long>(frame->frame_id),
                    ret);
                release_frame_reference(frame);
                continue;
            }

            char text[256];
            for (int i = 0; i < od_results.count; i++) {
                object_detect_result *det_result = &(od_results.results[i]);
                const int x1 = det_result->box.left;
                const int y1 = det_result->box.top;
                const int x2 = det_result->box.right;
                const int y2 = det_result->box.bottom;

                draw_rectangle(&src_image, x1, y1, x2 - x1, y2 - y1, COLOR_BLUE, 3);
                snprintf(
                    text,
                    sizeof(text),
                    "%s %.1f%%",
                    coco_cls_to_name(det_result->cls_id),
                    det_result->prop * 100);
                draw_text(&src_image, text, x1, y1 - 20, COLOR_RED, 10);
            }
        }

        if (enable_preview_) {
            try {
                cv::Mat bgr_frame;
                cv::cvtColor(rgb_frame, bgr_frame, cv::COLOR_RGB2BGR);
                cv::imshow(kPreviewWindowName, bgr_frame);
                cv::waitKey(1);
            } catch (const cv::Exception & ex) {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Preview display failed: %s. Preview has been disabled.",
                    ex.what());
                enable_preview_ = false;
            }
        }

        // publish_rgb_frame(frame);  // 发布时仍有卡顿，先保留调用点。

        if (frame->frame_id % 60 == 0) {
            RCLCPP_INFO(
                node_->get_logger(),
                "RGA converted frame %llu to RGB888 (%ux%u, stride=%u pixels)",
                static_cast<unsigned long long>(frame->frame_id),
                frame->width,
                frame->height,
                frame->converted_stride);
        }

        release_frame_reference(frame);
    }
}

void Test::save_loop()
{
    while (running_ && rclcpp::ok()) {
        auto frame = wait_and_pop_frame(save_queue_, save_mutex_, save_cv_);
        if (!frame) {
            continue;
        }

        if (frame->frame_id % 60 == 0) {
            RCLCPP_INFO(
                node_->get_logger(),
                "Save thread received frame %llu (placeholder, no save logic yet)",
                static_cast<unsigned long long>(frame->frame_id));
            auto message = std_msgs::msg::String();
            message.data = std::to_string(frame->frame_id) + " frame received in save thread (placeholder)";
            save_test_publisher_->publish(message);
        }

        release_frame_reference(frame);
    }
}

void Test::publish_status(const std::string & text)
{
    auto message = std_msgs::msg::String();
    message.data = text;
    publisher_->publish(message);
}

void Test::publish_rgb_frame(const FramePtr & frame)
{
    if (!frame || frame->converted_data.empty()) {
        return;
    }
    sensor_msgs::msg::Image message;
    message.header.stamp = frame->stamp;
    message.header.frame_id = "camera";
    message.height = frame->height;
    message.width = frame->width;
    message.encoding = "rgb8";
    message.is_bigendian = false;
    message.step = frame->converted_stride * 3U;
    message.data = frame->converted_data;

    image_publisher_->publish(std::move(message));
}

bool Test::initialize_detection()
{
    if (!file_exists(model_path_)) {
        RCLCPP_ERROR(node_->get_logger(), "RKNN model file not found: %s", model_path_.c_str());
        return false;
    }

    if (!file_exists(label_path_)) {
        RCLCPP_ERROR(node_->get_logger(), "Label file not found: %s", label_path_.c_str());
        return false;
    }

    const int post_process_ret = init_post_process(label_path_.c_str());
    if (post_process_ret != 0) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "init_post_process failed: ret=%d label_path=%s",
            post_process_ret,
            label_path_.c_str());
        return false;
    }
    post_process_ready_ = true;

    const int model_ret = init_yolov5_model(model_path_.c_str(), &rknn_app_ctx_);
    if (model_ret != 0) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "init_yolov5_model failed: ret=%d model_path=%s",
            model_ret,
            model_path_.c_str());
        shutdown_detection();
        return false;
    }

    detection_ready_ = true;
    return true;
}

void Test::shutdown_detection()
{
    if (detection_ready_ || rknn_app_ctx_.rknn_ctx != 0 || rknn_app_ctx_.input_attrs != nullptr ||
        rknn_app_ctx_.output_attrs != nullptr)
    {
        release_yolov5_model(&rknn_app_ctx_);
    }

    if (post_process_ready_) {
        deinit_post_process();
    }

    detection_ready_ = false;
    post_process_ready_ = false;
    std::memset(&rknn_app_ctx_, 0, sizeof(rknn_app_ctx_));
}
}  // namespace test_space
