#pragma once
#include <chrono>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

namespace qd {
namespace Device {

    /**
     * @brief 相机图像与时间戳数据
     */
    struct CameraData {
        cv::Mat img;
        std::chrono::steady_clock::time_point timestamp;
    };

    /**
     * @brief 设备抽象接口
     */
    class Device {
    public:
        /**
         * @brief 虚析构，保证派生类正确释放
         */
        virtual ~Device() = default;
        /**
         * @brief 获取一帧图像（阻塞或非阻塞由实现决定）
         * @return cv::Mat 图像
         */
        virtual cv::Mat get_image() = 0;
        /**
         * @brief 读取图像与时间戳
         * @param img 输出图像
         * @param timestamp 输出时间戳
         */
        virtual void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) = 0;
        /**
         * @brief 当前数据源是否已经耗尽
         * @return true 仅对有限离线数据源返回 true
         */
        virtual bool is_exhausted() const { return false; }
        // virtual void read(cv::Mat&, std::chrono::steady_clock::time_point&)=0;
    };

} // namespace Device
} // namespace qd
