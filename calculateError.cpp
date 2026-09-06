#include "calibrate.hpp"
#include "device.hpp"
#include "device_factory.hpp"
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include "web_viewer.hpp"

using namespace std;
using namespace cv;
using namespace qd;

const std::string keys =
    "{help h usage ? |                          | 输出命令行参数说明}"
    "{config-path c  | config/calibration.yaml | yaml配置文件路径 }";
/**
 * @brief 重投影误差验证程序入口
 * @param argc 参数数量
 * @param argv 参数列表
 * @return int 退出码
 */
int main(int argc, char* argv[]) {
    // 读取命令行参数
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }

    auto config_path = cli.get<std::string>("config-path");

    // 初始化设备
    auto device_ctx = qd::app::create_device(config_path);
    auto device = std::move(device_ctx.device);
    int wait_time = device_ctx.wait_time; // 用于图片显示延迟
    // 初始化标定类
    auto calibrate_ = qd::calibrate::Calibrate(config_path);

    // namedWindow("重投影误差");
    std::chrono::steady_clock::time_point timestamp;
    int count = 0;
        qd::WebViewer viewer(8080);
    viewer.namedWindow("重投影误差");
    while (true) {
        // 获取图像和串口数据
        Mat img;
        device->read(img, timestamp);

        // 检查图像
        if (img.empty()) {
            cout << "image is empty" << endl;
            if (count++ > 10) {
                break;
            } // 11次没获取到图像，退出
            continue;
        }

        calibrate_.display_error(img);
        // imshow("重投影误差", img);
        viewer.imshow("重投影误差", img);
        int key = viewer.waitKey(wait_time);

        // waitKey(wait_time);
    }

    return 0;
}
