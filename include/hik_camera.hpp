#pragma once
#include "CameraParams.h"
#include "MvCameraControl.h"
#include "device.hpp"
#include "thread_safe_queue.hpp"
#include <atomic>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <yaml-cpp/yaml.h>

namespace qd::Device {

/**
 * @brief 海康工业相机设备封装
 */
class Hik_Camera: public Device {
public:
    /**
     * @brief 构造并初始化海康相机
     * @param config_path YAML 配置文件路径
     */
    Hik_Camera(const std::string& config_path);
    /**
     * @brief 析构并关闭相机
     */
    ~Hik_Camera();
    /**
     * @brief 获取一帧图像
     * @return cv::Mat 图像
     */
    cv::Mat get_image() override;
    /**
     * @brief 读取图像与时间戳
     * @param img 输出图像
     * @param timestamp 输出时间戳
     */
    void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) override;

private:
    void* camera_handle_ { nullptr };
    int nRet = MV_OK;
    cv::ColorConversionCodes color_code_;
    double exposure_time_;
    cv::Mat image;
    tools::ThreadSafeQueue<CameraData> queue_;
    std::thread daemon_thread_;
    std::atomic<bool> running_ { true };
};

} // namespace qd::Device
