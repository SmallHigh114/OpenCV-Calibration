#include "calibrate.hpp"
#include "device.hpp"
#include "device_factory.hpp"
#include "serial_driver.hpp"
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include "web_viewer.hpp"

using namespace std;
using namespace cv;
using namespace qd;

const std::string keys =
    "{help h usage ? |                          | 输出命令行参数说明}"
    "{config-path c  | config/calibration.yaml | yaml配置文件路径 }"
    "{load-data l    |                          | 从文件夹加载已保存的手眼标定数据 }"
    "{data-path d    | ./handeye_calib_data     | 手眼标定数据文件夹路径 }";

/**
 * @brief 手眼标定程序入口
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
    bool load_data = cli.has("load-data");
    auto data_path = cli.get<std::string>("data-path");

    // 初始化标定类
    auto calibrate_ = qd::calibrate::Calibrate(config_path);

    // 如果指定了从文件夹加载数据，则直接加载并标定
    if (load_data) {
        std::cout << "从文件夹加载手眼标定数据: " << data_path << std::endl;
        if (calibrate_.load_handeye_data_from_folder(data_path)) {
            std::cout << "开始计算手眼标定参数..." << std::endl;
            calibrate_.calibrate_handeye();
            std::cout << "标定完成，程序退出" << std::endl;
            return 0;
        } else {
            std::cerr << "加载数据失败，程序退出" << std::endl;
            return 1;
        }
    }

    // 初始化设备
    auto device_ctx = qd::app::create_device(config_path);
    auto device = std::move(device_ctx.device);
    // 手眼标定串口
    std::unique_ptr<Serial_driver> protocol_ = std::make_unique<Serial_driver>(config_path);
    
    // namedWindow("手眼标定");
    std::chrono::steady_clock::time_point timestamp;
    Eigen::Quaterniond q;
    std::cout << "开始标定，按 'c' 键开始计算标定参数，按 's' 键采集数据，按 'ESC' 键退出"
              << std::endl;
    qd::WebViewer viewer(8080);
    viewer.namedWindow("手眼标定");
    while (true) {
        // 获取图像和串口数据
        Mat img;
        device->read(img, timestamp);
        q = protocol_->read(timestamp);

        // 检查图像
        if (img.empty()) {
            cout << "image is empty" << endl;
            continue;
        }


        // 处理键盘输入
        // int key = waitKey(10);
        int key = viewer.waitKey(10);

        if (key == 'c') {
            calibrate_.calibrate_handeye();
            cv::destroyAllWindows();
            protocol_.reset();
            device.reset();
            break;
        } // 标定
        else if (key == 27)
        {
            break;
        }

        bool enable_collect = (key == 's');
        calibrate_.collect_handeye(img, q, enable_collect);
        calibrate_.show_collected_corners(img);
        calibrate_.display_rpy(img, q); // 可视化角度

        // imshow("手眼标定", img);
        viewer.imshow("手眼标定", img);

    }

    std::cout << "标定完成，程序退出" << std::endl;
    return 0;
}
