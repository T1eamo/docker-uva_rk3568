#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

namespace test_space
{
class Test
{
private:
    // Frame 表示帧池中的一个帧对象。
    // 这里除了基础元信息，还保存一帧原始 YUYV 图像数据，以及 RGA 转换后的 RGB 数据。
    struct Frame
    {
        // 递增帧号，方便日志和调试时确认帧流转顺序。
        std::uint64_t frame_id{0};

        // 采集时间戳，用来给后续同步、延迟统计或消息发布时间打基础。
        rclcpp::Time stamp{0, 0, RCL_ROS_TIME};

        // 引用计数。
        // 采集线程分发给几个下游消费者，这里就记几个引用；所有消费者释放完后归还帧池。
        std::atomic<int> ref_count{0};

        // 原始采集帧的宽高、步长、像素格式和有效数据长度。
        std::uint32_t width{0};
        std::uint32_t height{0};
        std::uint32_t stride{0};
        std::uint32_t pixel_format{0};
        std::size_t data_size{0};

        // 真正的原始图像字节缓冲区。
        std::vector<std::uint8_t> data;

        // RGA 转换后的 RGB888 缓冲区及其布局信息。
        std::uint32_t converted_stride{0};
        std::uint32_t converted_pixel_format{0};
        std::size_t converted_data_size{0};
        std::vector<std::uint8_t> converted_data;

    };

    using FramePtr = std::shared_ptr<Frame>;

    // 初始化固定数量的帧对象，放入空闲池。
    void initialize_frame_pool(std::size_t frame_count);

    // 从空闲池中取出一个可写帧。
    FramePtr acquire_frame();

    // 将已处理完的帧放回空闲池。
    void recycle_frame(const FramePtr & frame);

    // 将采集到的一帧分发给检测线程和保存线程。
    void dispatch_frame(const FramePtr & frame);

    // 从指定队列中阻塞等待一帧。
    FramePtr wait_and_pop_frame(
        std::queue<FramePtr> & queue,
        std::mutex & queue_mutex,
        std::condition_variable & queue_cv);

    // 某个消费者用完一帧后释放一次引用。
    void release_frame_reference(const FramePtr & frame);

    // 采集线程：负责从 v4l2 获取原始 YUYV 帧，并分发给检测/保存两个下游消费者。
    void capture_loop();

    // 打开 v4l2 设备、设置格式、申请并 mmap 内核缓冲区。
    bool capture_initialize();

    // 关闭 v4l2 设备并释放 mmap 资源。
    void capture_cleanup();

    // 检测线程：负责把 YUYV 转成 RGB，并为后续检测逻辑准备数据。
    void detect_loop();

    // 保存线程骨架：后面接编码、落盘等逻辑时从这里继续扩展。
    void save_loop();

    // 根据当前采集参数计算 RGA 输出缓冲布局。
    void configure_rga_output_layout();

    // 使用 RGA 将一帧 YUYV 转换为 RGB888。
    bool convert_frame_to_rgb(const FramePtr & frame);

    // 向上按指定对齐值取整。
    static std::uint32_t align_up(std::uint32_t value, std::uint32_t alignment);

    // 发布简单状态字符串，当前主要用于演示和调试。
    void publish_status(const std::string & text);

    // 发布 RGA 转换后的 RGB888 图像。
    void publish_rgb_frame(const FramePtr & frame);

    // 外部传入的 ROS 节点句柄，用于日志、时间戳、publisher 等 ROS 能力。
    rclcpp::Node::SharedPtr node_;

    // 示例 publisher，后面可以替换成更正式的状态、检测结果或调试消息发布器。
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr save_test_publisher_;

    // 发布 RGA 转换后的 RGB 图像。
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

    // 全局运行标志，所有工作线程都根据它决定是否继续循环。
    std::atomic<bool> running_{false};

    // 当前演示启动采集线程、检测线程和保存线程骨架。
    std::thread capture_thread_;
    std::thread detect_thread_;
    std::thread save_thread_;

    // 帧池相关同步对象和空闲帧队列。
    std::mutex frame_pool_mutex_;
    std::condition_variable frame_pool_cv_;
    std::queue<FramePtr> free_frames_;

    // 检测消费队列及其同步对象。
    std::mutex detect_mutex_;
    std::condition_variable detect_cv_;
    std::queue<FramePtr> detect_queue_;

    // 保存消费队列及其同步对象，当前先保留框架。
    std::mutex save_mutex_;
    std::condition_variable save_cv_;
    std::queue<FramePtr> save_queue_;

    // 实际持有所有帧对象，避免帧池中的 shared_ptr 没有归属。
    std::vector<FramePtr> frame_storage_;

    // 生成递增帧号。
    std::atomic<std::uint64_t> next_frame_id_{0};

    // v4l2 设备信息和采集参数。
    int m_fd{-1};
    struct Buffer
    {
        void * start{nullptr};
        std::size_t length{0};
    };
    Buffer * m_buffers{nullptr};
    int m_buffer_count{0};
    bool capture_ready_{false};

    std::uint32_t capture_width_{640};
    std::uint32_t capture_height_{480};
    std::uint32_t capture_stride_{0};
    std::uint32_t capture_pixel_format_{0};
    std::size_t frame_bytes_{0};

    // RGA 输出 RGB888 的布局信息。
    std::uint32_t converted_width_{0};
    std::uint32_t converted_height_{0};
    std::uint32_t converted_stride_pixels_{0};
    std::uint32_t converted_row_bytes_{0};
    std::uint32_t converted_pixel_format_{0};
    std::size_t   converted_frame_bytes_{0};

public:
    explicit Test(rclcpp::Node::SharedPtr node);
    ~Test();

    void start();
    void stop();
};
}  // namespace test_space
