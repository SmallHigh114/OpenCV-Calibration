#include "thread_safe_queue.hpp"
#include "uart_transporter.hpp"
#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// 云台通讯协议（与 io/gimbal 模块一致）：
// 云台 → 视觉（GimbalToVision），__packed 共 43 字节：
//   [0..1]   head[2]       帧头 'S','P'
//   [2]      mode          uint8（0: 空闲, 1: 自瞄, 2: 小符, 3: 大符）
//   [3..18]  q[4]          float 四元数，wxyz 顺序
//   [19..22] yaw           float
//   [23..26] yaw_vel       float
//   [27..30] pitch         float
//   [31..34] pitch_vel     float
//   [35..38] bullet_speed  float
//   [39..40] bullet_count  uint16
//   [41..42] crc16         uint16，前 41 字节的 CRC16，高字节在前（大端）
struct __attribute__((packed)) GimbalToVision {
    uint8_t head[2] = { 'S', 'P' };
    uint8_t mode; // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
    float q[4]; // wxyz 顺序
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;    
    float bullet_speed;
    uint16_t bullet_count; // 子弹累计发送次数
    uint16_t crc16;
};

static_assert(sizeof(GimbalToVision) == 43);

// 视觉 → 云台（VisionToGimbal），__packed 共 29 字节：
//   [0..1]   head[2]  帧头 'S','P'
//   [2]      mode     uint8（0: 不控制, 1: 控制云台但不开火, 2: 控制云台且开火）
//   [3..26]  yaw / yaw_vel / yaw_acc / pitch / pitch_vel / pitch_acc  float
//   [27..28] crc16    uint16，前 27 字节的 CRC16
struct __attribute__((packed)) VisionToGimbal {
    uint8_t head[2] = { 'S', 'P' };
    uint8_t mode; // 0: 不控制, 1: 控制云台但不开火, 2: 控制云台且开火
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint16_t crc16;
};

static_assert(sizeof(VisionToGimbal) == 29);

/**
 * @brief 云台姿态数据
 */
struct IMUData {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
};

/**
 * @brief 串口云台通讯与姿态插值（协议见 GimbalToVision / VisionToGimbal）
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
     * @brief 根据目标时间戳读取姿态（四元数线性时间插值）
     * @param timestamp 目标时间戳
     * @return Eigen::Quaterniond 四元数姿态
     */
    Eigen::Quaterniond read(std::chrono::steady_clock::time_point timestamp);
    /**
     * @brief 向云台发送控制指令
     * @param control 是否控制云台
     * @param fire 是否开火（仅在 control 为 true 时有效）
     * @param yaw yaw 轴角度
     * @param yaw_vel yaw 轴角速度
     * @param yaw_acc yaw 轴角加速度
     * @param pitch pitch 轴角度
     * @param pitch_vel pitch 轴角速度
     * @param pitch_acc pitch 轴角加速度
     */
    void send(
        bool control,
        bool fire,
        float yaw,
        float yaw_vel,
        float yaw_acc,
        float pitch,
        float pitch_vel,
        float pitch_acc
    );

private:
    uint8_t tmp_buffer_[256];
    // 累积缓冲：数据按帧解析，帧头/CRC 失步时逐字节滑动重新同步
    std::vector<uint8_t> rx_buffer_;

    std::unique_ptr<UartTransporter> uart_transporter;
    tools::ThreadSafeQueue<IMUData> queue_;
    std::thread daemon_thread_;
    std::atomic<bool> running_ { true };
};
