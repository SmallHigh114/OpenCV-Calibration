#pragma once
#include "device.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <yaml-cpp/yaml.h>

namespace qd::Device {

/**
 * @brief 从图片序列读取图像的设备
 */
class Image_Reader: public Device {
public:
    /**
     * @brief 构造并加载图片列表
     * @param config_path YAML 配置文件路径
     */
    Image_Reader(const std::string& config_path);
    /**
     * @brief 析构并释放资源
     */
    ~Image_Reader();
    /**
     * @brief 获取下一张图片
     * @return cv::Mat 图像
     */
    cv::Mat get_image() override;
    /**
     * @brief 读取图像与时间戳（图片输入不包含时间戳）
     * @param img 输出图像
     * @param timestamp 输出时间戳
     */
    void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) override;
    /**
     * @brief 离线图片序列是否已经读完
     */
    bool is_exhausted() const override;

private:
    cv::VideoCapture cap;
    cv::Mat image;
    std::vector<cv::String> filenames;
    int index;
    bool exhausted_ { false };
    bool reported_exhausted_ { false };
};

} // namespace qd::Device
