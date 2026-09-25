#include "serial_driver.hpp"
#include "uart_transporter.hpp"
#include <cstring>
#include <fmt/core.h>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace {
// CRC16 查找表（与 io/gimbal 模块 tools/crc 一致，初值 0xFFFF）
const uint16_t k_crc16_table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
    0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
    0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
    0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
    0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
    0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
    0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
    0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
    0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
    0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
    0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
    0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

uint16_t crc16_calc(const uint8_t* msg, uint32_t len) {
    uint16_t crc = 0xffff;
    while (len--) {
        uint8_t ch = *msg++;
        crc = static_cast<uint16_t>(crc >> 8) ^ k_crc16_table[(crc ^ ch) & 0x00ff];
    }
    return crc;
}
} // namespace

Serial_driver::Serial_driver(const std::string& config_path): queue_(5000) {
    auto yaml = YAML::LoadFile(config_path);

    auto port_name = yaml["Serial"]["port_name"].as<std::string>();
    auto baud_rate = yaml["Serial"]["baud_rate"].as<int>();

    // debug
    fmt::print("import port_name: {} \n", port_name);
    fmt::print("import baud_rate: {} \n", baud_rate);

    uart_transporter = std::make_unique<UartTransporter>(port_name, baud_rate);

    while (!uart_transporter->open()) {
        fmt::print("serial open failed, retrying... \n");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    daemon_thread_ = std::thread([this]() {
        rx_buffer_.reserve(sizeof(GimbalToVision) * 4);
        fmt::print("等待云台数据...\n");
        uint64_t valid_cnt = 0;
        uint64_t crc_fail_cnt = 0;
        auto last_status = std::chrono::steady_clock::now();
        while (running_) {
            int recv_len = uart_transporter->read(tmp_buffer_, sizeof(tmp_buffer_));
            if (recv_len > 0) {
                rx_buffer_.insert(rx_buffer_.end(), tmp_buffer_, tmp_buffer_ + recv_len);

                // 按帧解析：帧头 'S','P' + CRC16 校验，失步时逐字节滑动重新同步
                while (rx_buffer_.size() >= sizeof(GimbalToVision)) {
                    if (rx_buffer_[0] != 'S' || rx_buffer_[1] != 'P') {
                        rx_buffer_.erase(rx_buffer_.begin());
                        continue;
                    }

                    // 线路上 CRC16 高字节在前（大端），CRC 覆盖除末尾 2 字节外的全部内容
                    auto crc_offset = sizeof(GimbalToVision) - sizeof(uint16_t);
                    uint16_t crc_rx = static_cast<uint16_t>(
                        (rx_buffer_[crc_offset + 1] << 8) | rx_buffer_[crc_offset]
                    );
                    if (crc16_calc(rx_buffer_.data(), crc_offset) != crc_rx) {
                        crc_fail_cnt++;
                        rx_buffer_.erase(rx_buffer_.begin());
                        continue;
                    }

                    GimbalToVision frame;
                    memcpy(&frame, rx_buffer_.data(), sizeof(frame));

                    auto timestamp = std::chrono::steady_clock::now();
                    Eigen::Quaterniond q(frame.q[0], frame.q[1], frame.q[2], frame.q[3]); // wxyz
                    queue_.push({ q, timestamp });

                    if (valid_cnt == 0) {
                        fmt::print(
                            "云台首帧已接收, q = [{:.4f}, {:.4f}, {:.4f}, {:.4f}]\n",
                            q.w(),
                            q.x(),
                            q.y(),
                            q.z()
                        );
                    }
                    valid_cnt++;

                    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + sizeof(frame));
                }
            }

            // 每秒打印一次状态：无任何输出 = 串口上没有字节；crc_fail 增长 = 字节乱码/协议不符
            auto now = std::chrono::steady_clock::now();
            if (now - last_status >= std::chrono::seconds(1)) {
                fmt::print(
                    "gimbal serial status: valid={} crc_fail={} buffered={}B\n",
                    valid_cnt,
                    crc_fail_cnt,
                    rx_buffer_.size()
                );
                valid_cnt = 0;
                crc_fail_cnt = 0;
                last_status = now;
            }
        }
    });
    fmt::print("open serial finish \n");
}

Serial_driver::~Serial_driver() {
    running_ = false;
    if (daemon_thread_.joinable())
        daemon_thread_.join();
}

Eigen::Quaterniond Serial_driver::read(std::chrono::steady_clock::time_point timestamp) {
    while (true) {
        IMUData ahead = queue_.pop();
        IMUData behind = queue_.front();

        auto t_ab = std::chrono::duration<double>(behind.timestamp - ahead.timestamp);
        if (t_ab.count() == 0) {
            return ahead.q.normalized();
        }
        auto t_ac = std::chrono::duration<double>(timestamp - ahead.timestamp);
        auto k = t_ac / t_ab;
        Eigen::Quaterniond q_c = ahead.q.slerp(k, behind.q).normalized();

        // 目标早于已有数据：向后外插
        if (timestamp < ahead.timestamp)
            return q_c;
        // 目标晚于该区间：取下一对数据
        if (!(ahead.timestamp < timestamp && timestamp <= behind.timestamp))
            continue;
        return q_c;
    }
}

void Serial_driver::send(
    bool control,
    bool fire,
    float yaw,
    float yaw_vel,
    float yaw_acc,
    float pitch,
    float pitch_vel,
    float pitch_acc
) {
    VisionToGimbal frame {};
    frame.head[0] = 'S';
    frame.head[1] = 'P';
    frame.mode = control ? (fire ? 2 : 1) : 0;
    frame.yaw = yaw;
    frame.yaw_vel = yaw_vel;
    frame.yaw_acc = yaw_acc;
    frame.pitch = pitch;
    frame.pitch_vel = pitch_vel;
    frame.pitch_acc = pitch_acc;
    frame.crc16 =
        crc16_calc(reinterpret_cast<uint8_t*>(&frame), sizeof(frame) - sizeof(frame.crc16));

    uart_transporter->write(reinterpret_cast<uint8_t*>(&frame), sizeof(frame));
}
