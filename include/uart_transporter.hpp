// Copyright (C) 2021 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Additional modifications and features by Chengfu Zou, 2023.
//
// Copyright (C) FYT Vision Group. All rights reserved.

#ifndef SERIAL_DRIVER_UART_TRANSPORTER_HPP_
#define SERIAL_DRIVER_UART_TRANSPORTER_HPP_

// std
#include <string>
// project

// 串口数据传输设备，符合通用传输接口。
class UartTransporter {
public:
    /**
     * @brief 构造串口传输对象
     * @param device_path 设备路径
     * @param speed 波特率
     * @param flow_ctrl 流控方式
     * @param databits 数据位
     * @param stopbits 停止位
     * @param parity 校验位
     */
    UartTransporter(
        const std::string& device_path = "/dev/ttyUSB0",
        int speed = 115200,
        int flow_ctrl = 0,
        int databits = 8,
        int stopbits = 1,
        int parity = 'N'
    ):
        device_path_(device_path),
        speed_(speed),
        flow_ctrl_(flow_ctrl),
        databits_(databits),
        stopbits_(stopbits),
        parity_(parity) {}

    /**
     * @brief 打开串口
     * @return true 成功
     * @return false 失败
     */
    bool open();
    /**
     * @brief 关闭串口
     */
    void close();
    /**
     * @brief 串口是否打开
     * @return true 打开
     * @return false 关闭
     */
    bool isOpen();
    /**
     * @brief 读取串口数据
     * @param buffer 输出缓冲区
     * @param len 读取长度
     * @return int 实际读取长度
     */
    int read(void* buffer, size_t len);
    /**
     * @brief 写入串口数据
     * @param buffer 输入缓冲区
     * @param len 写入长度
     * @return int 实际写入长度
     */
    int write(const void* buffer, size_t len);
    /**
     * @brief 获取最后错误信息
     * @return std::string 错误信息
     */
    std::string errorMessage() {
        return error_message_;
    }

private:
    /**
     * @brief 配置串口参数
     * @param speed 波特率
     * @param flow_ctrl 流控方式
     * @param databits 数据位
     * @param stopbits 停止位
     * @param parity 校验位
     * @return true 成功
     * @return false 失败
     */
    bool setParam(
        int speed = 115200,
        int flow_ctrl = 0,
        int databits = 0,
        int stopbits = 1,
        int parity = 'N'
    );

private:
    // 设备文件描述符
    int fd_ { -1 };
    // 设备状态
    bool is_open_ { false };
    std::string error_message_;
    // 设备参数
    std::string device_path_;
    int speed_;
    int flow_ctrl_;
    int databits_;
    int stopbits_;
    int parity_;
};

#endif // SERIAL_DRIVER_UART_TRANSPORTER_HPP_
