#include "thread_safe_queue.hpp"
#include "uart_transporter.hpp"
#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <fmt/core.h>
#include <memory>
#include <thread>
#include <vector>
#include <yaml-cpp/yaml.h>

// IMU 帧格式（与 serial/ 模块 ReceiveData 一致，共 17 字节）：
//   [0]      header     int8
//   [1..2]   fire_v     int16 小端
//   [3..6]   imu_yaw    int32 小端
//   [7..10]  imu_pitch  int32 小端
//   [11..14] imu_roll   int32 小端
//   [15..16] crc16      前 15 字节的 CRC16 校验，低字节在前
#define frame_length 17
/**
 * @brief IMU 姿态数据
 */
struct IMUData {
    Eigen::Quaterniond q;
    double roll;
    double pitch;
    double yaw;
    std::chrono::steady_clock::time_point timestamp;
};

/**
 * @brief 串口 IMU 数据读取与插值
 */
class Serial_driver {
public:
    /**
     * @brief 构造并打开串口
     * @param config_path YAML 配置文件路径
     */
    Serial_driver(const std::string& config_path);
    /**
     * @brief 析构并停止读取线程
     */
    ~Serial_driver();
    /**
     * @brief 根据目标时间戳读取姿态（线性时间插值）
     * @param timestamp 目标时间戳
     * @return Eigen::Quaterniond 四元数姿态
     */
    Eigen::Quaterniond read(std::chrono::steady_clock::time_point timestamp);
    /**
     * @brief 欧拉角转四元数（角度制）
     * @param roll 横滚角（度）
     * @param pitch 俯仰角（度）
     * @param yaw 偏航角（度）
     * @return Eigen::Quaterniond 归一化四元数
     */
    Eigen::Quaterniond rpyToQuat(double roll, double pitch, double yaw);

private:
    IMUData data_ahead_;
    IMUData data_behind_;

    uint8_t tmp_buffer_[frame_length];
    // 累积缓冲：数据按帧解析，CRC 失步时逐字节滑动重新同步
    std::vector<uint8_t> rx_buffer_;

    std::unique_ptr<UartTransporter> uart_transporter;
    tools::ThreadSafeQueue<IMUData> queue_;
    std::thread daemon_thread_;
    std::atomic<bool> running_ { true };
};