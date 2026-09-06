#include "device_factory.hpp"
#include "hik_camera.hpp"
#include "image_reader.hpp"
#include "uvc_camera.hpp"
#include <yaml-cpp/yaml.h>

namespace qd::app {

DeviceContext create_device(const std::string& config_path) {
    DeviceContext ctx;
    auto yaml = YAML::LoadFile(config_path);

    auto device_type = yaml["device"].as<std::string>();
    if (device_type == "HIK") {
        ctx.device = std::make_unique<qd::Device::Hik_Camera>(config_path);
    } else if (device_type == "UVC") {
        ctx.device = std::make_unique<qd::Device::UVC_Camera>(config_path);
    } else if (device_type == "IMG") {
        ctx.wait_time = 0;
        ctx.device = std::make_unique<qd::Device::Image_Reader>(config_path);
    }

    return ctx;
}

} // namespace qd::app
