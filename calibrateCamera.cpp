#include "calibrate.hpp"
#include "device.hpp"
#include "device_factory.hpp"
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "web_viewer.hpp"

using namespace std;
using namespace cv;
using namespace qd;

const std::string keys =
    "{help h usage ? |                          | 输出命令行参数说明}"
    "{config-path c  | config/calibration.yaml | yaml配置文件路径 }";

/**
 * @brief 相机标定程序入口
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

    // namedWindow("相机标定");
    qd::WebViewer viewer(8080);
    viewer.namedWindow("相机标定");
    std::chrono::steady_clock::time_point timestamp;
    Eigen::Quaterniond q;
    std::cout << "开始标定，操作说明：\n"
              << "  's'   - 手动采集当前帧\n"
              << "  'a'   - 切换自动采集模式（直接移植自 ROS image_pipeline/camera_calibration）\n"
              << "  'c'   - 开始计算标定参数\n"
              << "  'ESC' - 退出\n"
              << "  自动采集策略：基于棋盘归一化参数 (X/Y/Size/Skew) 去重并统计覆盖度" << std::endl;

    while (true) {
        Mat img;
        device->read(img, timestamp);

        if (img.empty()) {
            if (device->is_exhausted()) {
                std::cout << "离线图像已读取完毕，开始执行标定。" << std::endl;
                cv::destroyAllWindows();
                return calibrate_.calibrate_camera() ? 0 : 1;
            }

            cout << "image is empty" << endl;
            continue;
        }

        // int key = waitKey(wait_time);
        int key = viewer.waitKey(wait_time);
        if (key == 'c') {
            if (calibrate_.calibrate_camera()) {
                cv::destroyAllWindows();
                break;
            } else {
                std::cout << "请继续采集有效的标定图像后再次按 'c'。" << std::endl;
            }
        } else if (key == 'a') {
            calibrate_.set_auto_collect(!calibrate_.is_auto_collect_enabled());
        } else if (key == 27) {
            break;
        }

        calibrate_.collect_camera(img, key == 's');

        // imshow("相机标定", img);
        viewer.imshow("相机标定", img);
    }

    device.reset();
    std::cout << "标定完成，程序退出" << std::endl;
    return 0;
}
