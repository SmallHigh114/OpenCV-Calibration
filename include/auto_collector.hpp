#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace qd::calibrate {

/**
 * @brief ROS \c camera_calibration 风格的自动采集器
 *
 * 直接移植自 ROS 包 \c image_pipeline/camera_calibration 中
 * \c calibrator.py 的核心算法（Willow Garage / OSRF, BSD-3-Clause）。
 *
 * 算法概述：
 *   - 基于检测到的棋盘 4 个外角点，计算一个 4 维归一化特征
 *     <tt>params = [p_x, p_y, p_size, p_skew]</tt>，全部位于
 *     \f$[0, 1]\f$；
 *   - 用与历史样本的 L1 距离阈值过滤"几乎重复"的样本；
 *   - 可选静止性检查（与上一帧角点的平均位移）避免运动模糊；
 *   - 用 <tt>param_ranges = [0.7, 0.7, 0.4, 0.5]</tt> 估计每个维度
 *     的覆盖进度，全部覆盖或样本数达到下限则视为可标定。
 *
 * 本类只负责自动采集策略与覆盖度统计，不持有标定数据本身。
 */
class AutoCollector {
public:
    /**
     * @brief 标定板在画面中的归一化特征参数
     *
     * 维度顺序与 ROS 原版保持一致：X/Y/Size/Skew。
     */
    struct Params {
        double x;    ///< 标定板中心 X 归一化坐标
        double y;    ///< 标定板中心 Y 归一化坐标
        double size; ///< 标定板大小：sqrt(area / image_area)
        double skew; ///< 倾斜程度：min(1, 2 * |π/2 - 角度|)
    };

    /**
     * @brief 自动采集器配置项
     */
    struct Config {
        /// 同一参数与历史样本最小 L1 距离，低于该值视为重复样本（ROS 默认 0.2）
        double param_distance_threshold = 0.2;
        /// 4 维参数的"足够覆盖"阈值（X, Y, Size, Skew，ROS 默认 [0.7, 0.7, 0.4, 0.5]）
        std::array<double, 4> param_ranges { { 0.7, 0.7, 0.4, 0.5 } };
        /// 视为采集完成的最少样本数（即便部分维度未覆盖也允许标定，ROS 默认 40）
        std::size_t goodenough_samples = 40;
        /// 角点平均运动速度阈值（像素/帧）。<= 0 表示禁用静止性检查（ROS 默认 -1）
        double max_chessboard_speed = -1.0;
        /// 两次自动采集的最小时间间隔（毫秒），ROS 原版未定义，这里保留作为限频
        int min_interval_ms = 0;
    };

    /**
     * @brief 4 个维度的覆盖度报告，用于 UI 进度条
     */
    struct Progress {
        std::array<double, 4> min_params { { 0, 0, 0, 0 } }; ///< 各维度历史最小值
        std::array<double, 4> max_params { { 0, 0, 0, 0 } }; ///< 各维度历史最大值
        std::array<double, 4> progress { { 0, 0, 0, 0 } };   ///< 各维度覆盖进度 [0, 1]
        std::size_t sample_count = 0; ///< 已采集样本数
        bool        good_enough  = false; ///< 是否已满足标定条件
    };

    /**
     * @brief 使用默认配置构造自动采集器
     * @param board_size  标定板内角点尺寸（cols x rows，需与 OpenCV findChessboardCorners 输出一致）
     */
    explicit AutoCollector(const cv::Size& board_size);

    /**
     * @brief 构造自动采集器
     * @param board_size  标定板内角点尺寸（cols x rows，需与 OpenCV findChessboardCorners 输出一致）
     * @param config      自动采集策略配置
     */
    AutoCollector(const cv::Size& board_size, const Config& config);

    /**
     * @brief 计算当前帧的归一化特征参数
     * @param[in]  corners    棋盘角点（行优先，长度 = cols * rows）
     * @param[in]  image_size 原图分辨率
     * @param[out] out        计算得到的参数
     * @return 角点数量与标定板尺寸不匹配时返回 false
     */
    bool compute_params(
        const std::vector<cv::Point2f>& corners,
        const cv::Size&                 image_size,
        Params&                         out
    ) const;

    /**
     * @brief 判断当前帧是否值得加入样本库
     *
     * 等价于 ROS \c calibrator.py 中的 \c is_good_sample：
     *   - 历史为空时直接接收；
     *   - 与历史样本的最小 L1 距离 \f$\le\f$ \c param_distance_threshold 则拒绝；
     *   - 启用静止性检查时，平均角点位移过大也拒绝；
     *   - 启用 \c min_interval_ms 时距离上次采集时间不足也拒绝。
     *
     * @param params           当前帧的归一化参数
     * @param corners          当前帧角点
     * @param last_corners     上一帧角点（可空，用于静止性检查）
     * @return true 表示建议采集
     */
    bool is_good_sample(
        const Params&                   params,
        const std::vector<cv::Point2f>& corners,
        const std::vector<cv::Point2f>& last_corners
    ) const;

    /**
     * @brief 把样本登记进历史库并刷新计时器
     *
     * 调用方在真正保存图像后再调用本接口，本类内部仅保留特征参数。
     */
    void add_sample(const Params& params);

    /**
     * @brief 缓存最近一帧角点，用于下一帧的静止性检查
     */
    void update_last_frame(const std::vector<cv::Point2f>& corners);

    /**
     * @brief 计算当前覆盖度报告，等价于 ROS 的 \c compute_goodenough
     */
    Progress compute_progress() const;

    /**
     * @brief 在图像左下角绘制 ROS 风格的覆盖度进度条
     * @param img            待绘制图像（BGR）
     * @param current_params 当前帧参数；nullptr 表示未检测到棋盘
     */
    void draw_progress(cv::Mat& img, const Params* current_params) const;

    /**
     * @brief 清空所有历史样本与覆盖度
     */
    void reset();

    /**
     * @brief 启用 / 停用自动采集
     *
     * 仅切换标志位，不影响已有样本统计；从禁用切换到启用时刻意"借用"过去 10 秒
     * 作为上次采集时间，让用户一打开就能采到第一帧。
     */
    void set_enabled(bool enabled);

    /// @brief 当前是否启用自动采集
    bool enabled() const { return enabled_; }

    /// @brief 获取自动采集配置（只读）
    const Config& config() const { return config_; }

    /// @brief 已经接受的样本数量
    std::size_t sample_count() const { return db_params_.size(); }

private:
    /**
     * @brief 当前两次采集之间是否已经超过 \c min_interval_ms
     */
    bool interval_ready() const;

    cv::Size            board_size_;
    Config              config_;
    bool                enabled_ = false;
    std::vector<Params> db_params_;

    std::chrono::steady_clock::time_point last_collect_time_;
};

} // namespace qd::calibrate
