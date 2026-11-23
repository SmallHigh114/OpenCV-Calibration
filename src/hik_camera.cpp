#include "hik_camera.hpp"
#include <opencv2/highgui.hpp>
#include <string>

namespace qd::Device {

Hik_Camera::Hik_Camera(const std::string& config_path): queue_(1) {
    auto version = MV_CC_GetSDKVersion();
    std::cout << "HIK SDK version: " << std::hex << version << std::dec << std::endl;

    auto yaml = YAML::LoadFile(config_path);
    exposure_time_ = yaml["HIK"]["exposure_time"].as<double>();
    // 枚举设备
    MV_CC_DEVICE_INFO_LIST device_list;
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (nRet == MV_OK)
        std::cout << std::hex << "Found camera count = " << device_list.nDeviceNum << std::dec
                  << std::endl;
    // 等待设备连接
    while (device_list.nDeviceNum == 0) {
        std::cout << "No camera found!" << std::endl;
        std::cout << std::hex << "Enum state: " << nRet << std::dec << std::endl;
        cv::waitKey(1000);
        nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    }

    // 打开设备，创建打开句柄
    MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
    nRet = MV_CC_OpenDevice(camera_handle_);

    // 配置参数
    MV_CC_SetBoolValue(
        camera_handle_,
        "AcquisitionFrameRateEnable",
        yaml["HIK"]["enable_frame_rate"].as<bool>()
    );
    MV_CC_SetFloatValue(
        camera_handle_,
        "AcquisitionFrameRate",
        yaml["HIK"]["frame_rate"].as<double>()
    );
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", yaml["HIK"]["exposure_time"].as<double>());
    MV_CC_SetFloatValue(camera_handle_, "Gain", yaml["HIK"]["gain"].as<double>());
    MV_CC_SetEnumValueByString(
        camera_handle_,
        "ADCBitDepth",
        yaml["HIK"]["adc_bit_depth"].as<std::string>().c_str()
    );
    MV_CC_SetEnumValueByString(
        camera_handle_,
        "PixelFormat",
        yaml["HIK"]["pixel_format"].as<std::string>().c_str()
    );

    // 获得图像格式
    auto pixel_type = yaml["HIK"]["pixel_format"].as<std::string>();
    const static std::unordered_map<std::string, cv::ColorConversionCodes> type_map = {
        { "BayerGR8", cv::COLOR_BayerGR2RGB },
        { "BayerRG8", cv::COLOR_BayerRG2RGB },
        { "BayerGB8", cv::COLOR_BayerGB2RGB },
        { "BayerBG8", cv::COLOR_BayerBG2RGB }
    };
    this->color_code_ = type_map.at(pixel_type);

    // 开始采集图像
    nRet = MV_CC_StartGrabbing(camera_handle_);
    if (nRet != MV_OK) {
        std::cout << "MV_CC_StartGrabbing failed: " << nRet << std::endl;
    }

    daemon_thread_ = std::thread([this]() {
        while (running_) {
            MV_FRAME_OUT raw;
            nRet = MV_CC_GetImageBuffer(camera_handle_, &raw, 1000);
            // 打上时间戳
            auto timestamp = std::chrono::steady_clock::now();
            std::chrono::duration<double, std::micro> half_exposure(exposure_time_ / 2.0);
            timestamp = timestamp
                + std::chrono::duration_cast<std::chrono::steady_clock::duration>(half_exposure);

            if (nRet != MV_OK) {
                std::cout << std::hex << "No image data: " << nRet << std::dec << std::endl;
                continue;
            }
            if (!running_) {
                if (nRet == MV_OK) {
                    MV_CC_FreeImageBuffer(camera_handle_, &raw);
                }
                break;
            }
            cv::Mat img(
                cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight),
                CV_8U,
                raw.pBufAddr
            );

            // unsigned char *pDataForRGB = (unsigned char*)malloc(raw.stFrameInfo.nExtendWidth *
            // raw.stFrameInfo.nExtendHeight * 4 + 2048);
            MV_CC_PIXEL_CONVERT_PARAM cvt_param;
            cvt_param.nWidth = raw.stFrameInfo.nWidth;
            cvt_param.nHeight = raw.stFrameInfo.nHeight;

            cvt_param.pSrcData = raw.pBufAddr;
            cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
            cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;

            cvt_param.pDstBuffer = img.data;
            // cvt_param.pDstBuffer = pDataForRGB;
            cvt_param.nDstBufferSize =
                raw.stFrameInfo.nExtendWidth * raw.stFrameInfo.nExtendHeight * 4 + 2048;
            cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
            // nRet = MV_CC_ConvertPixelType(camera_handle_, &cvt_param);
            // cv::Mat dst_image(
            //     cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight),
            //     CV_8UC3,
            //     pDataForRGB
            // );
            cv::Mat dst_image;
            cv::cvtColor(img, dst_image, this->color_code_);
            img = dst_image;
            queue_.push({ img, timestamp });

            // 将pFrame内的数据指针权限进行释放
            MV_CC_FreeImageBuffer(camera_handle_, &raw);
        }
    });
}

Hik_Camera::~Hik_Camera() {
    running_ = false;
    if (camera_handle_) {
        MV_CC_StopGrabbing(camera_handle_);
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(&camera_handle_);
    }
    if (daemon_thread_.joinable())
        daemon_thread_.join();
}

cv::Mat Hik_Camera::get_image() {
    CameraData data;
    queue_.pop(data);
    return data.img;
}

void Hik_Camera::read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) {
    CameraData data;
    queue_.pop(data);

    img = data.img;
    timestamp = data.timestamp;
}

} // namespace qd::Device
