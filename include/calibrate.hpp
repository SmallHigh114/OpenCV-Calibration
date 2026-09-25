#pragma once
// eigen
#include <Eigen/Dense>
// fmt
#include <fmt/core.h>
#include <fmt/format.h>
// c++
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
// opencv
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
// yaml-cpp
#include <utility>
#include <yaml-cpp/yaml.h>

// 项目内：ROS 风格自动采集器
#include "auto_collector.hpp"

#define IN
#define OUT
#define MINI_DISTANCE_PIX 20.0 // 标定板角点最小距离像素px值

namespace qd::calibrate {

using namespace cv;
using namespace std;

// 参数类
/**
 * @brief 标定板类型
 */
enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
/**
 * @brief 标定流程状态
 */
enum Mode { Calibrating, Calibrated, Undistorting };
/**
 * @brief 标定参数读取与解析
 */
struct Paramer {
    /**
     * @brief 从配置文件读取标定板参数
     * @param config_path YAML 配置文件路径
     */
    Paramer(const std::string& config_path) {
        auto yaml = YAML::LoadFile(config_path);

        // 判断标定板类型
        auto val = yaml["pattern"].as<std::string>();
        if (val == "circles")
            pattern = CIRCLES_GRID;
        else if (val == "acircles")
            pattern = ASYMMETRIC_CIRCLES_GRID;
        else if (val == "chessboard")
            pattern = CHESSBOARD;

        // 标定板尺寸
        boardSize.height = yaml["pattern_rows"].as<int>();
        boardSize.width = yaml["pattern_cols"].as<int>();

        squareSize = yaml["square_size"].as<float>();
        grid_width = squareSize * (boardSize.width - 1);
    }

    Pattern pattern; // 标定板类型
    cv::Size boardSize; // 标定板内角点个数
    float squareSize; // 标定板方格边长
    float grid_width; // 标定板宽度
};

/**
 * @brief 相机标定与手眼标定流程
 */
class Calibrate {
public:
    Calibrate(const std::string& config_path);

    /**
     * @brief 收集相机标定数据
     * 
     * @param img 原始图像
     * @param enable_collect 是否收集数据 
     * @return true 
     * @return false 
     */
    bool collect_camera(Mat& img, bool enable_collect = false);

    /**
    @brief 收集手眼标定数据
    @param enable_collect 是否收集数据
    */
    void collect_handeye(Mat& img, const Eigen::Quaterniond& q, IN bool enable_collect = false);

    /**
    @brief 对采集到的数据进行相机标定
    */
    bool calibrate_camera();
    /**
    @brief 对收集到的数据进行手眼标定
    */
    void calibrate_handeye();

    /**
    @brief 可视化输入的云台欧拉角
    */
    bool display_rpy(cv::Mat& img, const Eigen::Quaterniond& q);

    /**
    * @brief 显示重投影误差
    * @param img 输入图像
    */
    void display_error(cv::Mat& img);

    /**
    * @brief 显示已采集的标定板位置，用于手眼标定确认收集情况
    * 
    * @param img 
    */
    void show_collected_corners(cv::Mat& img);

    // 手眼标定验证相关方法
    /**
    * @brief 从YAML文件加载手眼标定结果
    * @param handeye_yaml_path 手眼标定结果YAML文件路径
    * @return 是否成功加载
    */
    bool load_handeye_calibration(const std::string& handeye_yaml_path);
    /**
    * @brief 验证手眼标定准确性
    * @param img 输入图像
    * @param gimbal_quaternion 云台四元数（用于对比）
    */
    void validate_handeye(cv::Mat& img, const Eigen::Quaterniond& gimbal_quaternion);
    /**
    * @brief 重置验证统计信息
    */
    void reset_validation_stats(); // 重置验证统计信息

    /**
    * @brief 从文件夹加载手眼标定数据
    * @param folder_path 数据文件夹路径
    * @return 是否成功加载
    */
    bool load_handeye_data_from_folder(const std::string& folder_path); // 从文件夹加载手眼标定数据

    /**
     * @brief 将云台模块发来的四元数（IMU 系）换算为云台→世界旋转矩阵
     *
     * 与 sp_vision_25 的 Solver::set_R_gimbal2world 约定一致：
     *   R_gimbal2world = R_gimbal2imubodyᵀ · R_imubody2imuabs · R_gimbal2imubody
     * IMU 安装系与云台系重合时 R_gimbal2imubody 为单位阵，此换算退化为直接取 q。
     *
     * @param q 云台模块四元数（wxyz，IMU 体→IMU 绝对系）
     * @return 云台→世界旋转矩阵
     */
    Eigen::Matrix3d gimbal2world(const Eigen::Quaterniond& q) const;

    /**
     * @brief 切换自动采集模式（直接移植自 ROS camera_calibration）
     * @param enable 是否启用
     */
    void set_auto_collect(bool enable);

    /**
     * @brief 查询当前自动采集状态
     */
    bool is_auto_collect_enabled() const;

private:
    struct RpyRange {
        bool valid { false };
        Eigen::Vector3d min_ypr_deg { Eigen::Vector3d::Zero() };
        Eigen::Vector3d max_ypr_deg { Eigen::Vector3d::Zero() };
    };

    /**
    @brief 获取标定板角点
    */
    bool collect_camera(
        IN Mat& img,
        OUT std::vector<Point2f>& pixel_points,
        OUT vector<Point3f>& object_points
    );

    /**
     * @brief 计算标定板感兴趣区域内的拉普拉斯方差，作为清晰度指标
     *
     * 该指标 ROS 原版没有，仅作为对运动模糊 / 高曝光拖影的额外保护，
     * 默认开启。配置中 \c auto_collect_sharpness_threshold <= 0 时关闭。
     *
     * @param img        原始图像
     * @param corners    检测到的角点
     * @return 拉普拉斯方差，值越大越清晰
     */
    double compute_sharpness(const cv::Mat& img, const std::vector<cv::Point2f>& corners) const;

    /**
    @brief 输入 2D 标定角点获得标定板坐标系点位
    */
    vector<Point3f> calcChessboardCorners(std::vector<cv::Point2f>& pixel_points);

    /**
    @brief 查找 2D 标定角点
    */
    bool find_Chessboard(const cv::Mat& img, std::vector<cv::Point2f>& pixel_points);

    /**
        @brief 计算重投影误差
        @param object_points 3D 物体点
        @param pixel_points 2D 像素点
        @return 重投影误差
    */
    double calculate_reprojection_error(
        const std::vector<cv::Point2f>& pixel_points,
        const std::vector<cv::Point2f>& projected_points
    );

    // 保存和加载标定数据
    /**
    * @brief 保存相机标定图片
    * @param img 要保存的图像
    * @param index 图片索引
    */
    void save_camera_image(const cv::Mat& img, int index); // 保存相机标定图片
    /**
    * @brief 保存手眼标定数据（图片和姿态信息）
    * @param img 要保存的图像
    * @param q 云台四元数
    * @param index 数据索引
    */
    void save_handeye_data(
        const cv::Mat& img,
        const Eigen::Quaterniond& q,
        int index
    ); // 保存手眼标定数据（图片+姿态）

    /**
    * @brief 保存相机标定结果到 YAML 文件
    *
    * @param image_size 图像大小 (cv::Size(width, height))
    * @param camera_matrix 相机内参矩阵 (3x3)
    * @param dist_coeffs 畸变系数 (1xN，通常5个或8个)
    * @param filename 输出的YAML文件路径
    */
    void saveCalibrationYAML(
        const cv::Size& image_size,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs,
        const std::string& filename
    );
    /**
    @brief 输出手眼标定数据
    */
    void print_yaml(
        const cv::Mat& R_camera2gimbal,
        const cv::Mat& t_camera2gimbal,
        const Eigen::Vector3d& rpy
    );
    /**
    @brief 输出手眼标定数据（含标定板位姿信息）
    */
    void print_yaml(
        const cv::Mat& t_camera2gimbal,
        const Eigen::Vector3d& rpy,
        double board_distance,
        const Eigen::Vector3d& board_ypr,
        const RpyRange& handeye_rpy_range
    );
    /**
    * @brief 保存手眼标定结果到YAML文件
    * @param R_camera2gimbal 相机到云台的旋转矩阵
    * @param t_camera2gimbal 相机到云台的平移向量
    * @param rpy 相机同理想情况的偏角
    * @param handeye_rpy_range 参与标定的下位机 RPY 姿态角范围
    * @param filename 输出的YAML文件路径
    */
    void saveHandEyeCalibrationYAML(
        const cv::Mat& R_camera2gimbal,
        const cv::Mat& t_camera2gimbal,
        const Eigen::Vector3d& rpy,
        const RpyRange& handeye_rpy_range,
        const std::string& filename
    );

    /**
     * @brief 统计参与手眼标定的下位机 RPY 姿态角范围
     */
    RpyRange calculate_handeye_rpy_range() const;

    /**
     * @brief 计算标定板角点的最小距离
     * 
     * @param pixel_points 标定板角点
     * @return std::pair<bool, double> bool：是否满足最小距离要求，double：最小距离像素px值
     */
    std::pair<bool, double> calculate_coners_min_distance(IN std::vector<Point2f>& pixel_points);

public:
    Paramer paramer;

private:
    Size img_size;

    // 标定用数据
    std::vector<std::vector<cv::Point3f>> obj_points;
    std::vector<std::vector<cv::Point2f>> img_points;

    // 参数
    cv::Matx33d camera_matrix;
    cv::Mat distort_coeffs;
    int calibrateCamera_flags_ = cv::CALIB_FIX_K3;
    // IMU 安装系到云台系的固定旋转（行主序，配置项 R_gimbal2imubody）
    Eigen::Matrix3d R_gimbal2imubody_ { Eigen::Matrix3d::Identity() };

    std::vector<cv::Mat> rvecs, tvecs;
    // 手眼标定用数据（calibrateRobotWorldHandEye 需要 world2gimbal）
    std::vector<cv::Mat> R_world2gimbal_list, t_world2gimbal_list;
    std::vector<Eigen::Vector3d> handeye_ypr_deg_list_;
    // 手眼标定结果（用于验证）
    cv::Mat R_camera2gimbal;
    cv::Mat t_camera2gimbal;
    bool handeye_loaded = false;
    // 验证用历史数据（用于计算位置一致性）
    std::vector<cv::Mat> world_positions_history; // 存储标定板在世界坐标系下的位置历史

    cv::TickMeter tm; // 延迟计时器
    int collected_count = 0; // 已采集的标定图像数量

    // 数据保存路径
    std::string camera_calib_save_path; // 相机标定图片保存路径
    std::string handeye_calib_save_path; // 手眼标定数据保存路径

    // ---- 自动采集（直接移植自 ROS image_pipeline/camera_calibration）----
    /// 自动采集器：负责 ROS 风格的特征参数 / 去重 / 覆盖度统计 / UI 进度条
    std::unique_ptr<AutoCollector> auto_collector_;
    /// 上一帧角点缓存，供 AutoCollector 做静止性检查
    std::vector<cv::Point2f> last_frame_corners_;
    /// 拉普拉斯方差阈值（<= 0 时关闭清晰度检查）
    double auto_collect_sharpness_threshold_ = 0.0;
};

/**
 * @brief 归一化角度到 $(-\pi, \pi]$
 * @param angle 输入角度（弧度）
 * @return double 归一化后的角度
 */
static double limit_rad(double angle) {
    while (angle > CV_PI)
        angle -= 2 * CV_PI;
    while (angle <= -CV_PI)
        angle += 2 * CV_PI;
    return angle;
}

/**
 * @brief 四元数转欧拉角
 * @param q 四元数
 * @param axis0 轴序0
 * @param axis1 轴序1
 * @param axis2 轴序2
 * @param extrinsic 是否为外旋
 * @return Eigen::Vector3d 欧拉角（弧度）
 */
static Eigen::Vector3d
eulers(Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic = false) {
    if (!extrinsic)
        std::swap(axis0, axis2);

    auto i = axis0, j = axis1, k = axis2;
    auto is_proper = (i == k);
    if (is_proper)
        k = 3 - i - j;
    auto sign = (i - j) * (j - k) * (k - i) / 2;

    double a, b, c, d;
    Eigen::Vector4d xyzw = q.coeffs();
    if (is_proper) {
        a = xyzw[3];
        b = xyzw[i];
        c = xyzw[j];
        d = xyzw[k] * sign;
    } else {
        a = xyzw[3] - xyzw[j];
        b = xyzw[i] + xyzw[k] * sign;
        c = xyzw[j] + xyzw[3];
        d = xyzw[k] * sign - xyzw[i];
    }

    Eigen::Vector3d eulers;
    auto n2 = a * a + b * b + c * c + d * d;
    eulers[1] = std::acos(2 * (a * a + b * b) / n2 - 1);

    auto half_sum = std::atan2(b, a);
    auto half_diff = std::atan2(-d, c);

    auto eps = 1e-7;
    auto safe1 = std::abs(eulers[1]) >= eps;
    auto safe2 = std::abs(eulers[1] - CV_PI) >= eps;
    auto safe = safe1 && safe2;
    if (safe) {
        eulers[0] = half_sum + half_diff;
        eulers[2] = half_sum - half_diff;
    } else {
        if (!extrinsic) {
            eulers[0] = 0;
            if (!safe1)
                eulers[2] = 2 * half_sum;
            if (!safe2)
                eulers[2] = -2 * half_diff;
        } else {
            eulers[2] = 0;
            if (!safe1)
                eulers[0] = 2 * half_sum;
            if (!safe2)
                eulers[0] = 2 * half_diff;
        }
    }

    for (int i = 0; i < 3; i++)
        eulers[i] = limit_rad(eulers[i]);

    if (!is_proper) {
        eulers[2] *= sign;
        eulers[1] -= CV_PI / 2;
    }

    if (!extrinsic)
        std::swap(eulers[0], eulers[2]);

    return eulers;
}

/**
 * @brief 旋转矩阵转欧拉角
 * @param R 旋转矩阵
 * @param axis0 轴序0
 * @param axis1 轴序1
 * @param axis2 轴序2
 * @param extrinsic 是否为外旋
 * @return Eigen::Vector3d 欧拉角（弧度）
 */
static Eigen::Vector3d
eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic = false) {
    Eigen::Quaterniond q(R);
    return eulers(q, axis0, axis1, axis2, extrinsic);
}

} // namespace qd::calibrate
