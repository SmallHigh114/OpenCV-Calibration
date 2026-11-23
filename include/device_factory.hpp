#pragma once
#include "device.hpp"
#include <memory>
#include <string>

namespace qd::app {

/**
 * @brief 设备创建结果，包含设备实例与展示等待时间
 */
struct DeviceContext {
    std::unique_ptr<qd::Device::Device> device;
    int wait_time = 1;
};

/**
 * @brief 根据配置文件创建相机/图片输入设备
 *
 * @param config_path YAML 配置文件路径
 * @return DeviceContext 设备实例与推荐的等待时间
 */
DeviceContext create_device(const std::string& config_path);

} // namespace qd::app
