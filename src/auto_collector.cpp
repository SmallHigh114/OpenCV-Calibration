/**
 * @file auto_collector.cpp
 * @brief ROS \c camera_calibration 自动采集算法的 C++ 移植
 *
 * 算法与命名严格对齐 ROS 包 \c image_pipeline/camera_calibration 中的
 * \c calibrator.py（Willow Garage / OSRF, BSD-3-Clause）。
 */

#include "auto_collector.hpp"

#include <algorithm>
#include <cmath>
#include <fmt/core.h>
#include <numeric>
#include <opencv2/imgproc.hpp>

namespace qd::calibrate {

namespace {

/**
 * @brief 取棋盘外侧的 4 个角点
 *
 * 等价于 ROS 的 \c _get_outside_corners。OpenCV 的 \c findChessboardCorners
 * 输出顺序是行优先（先列后行），因此 4 个外角点的下标可由列数 \c xdim 推得：
 *   - up_left   = corners[0]
 *   - up_right  = corners[xdim - 1]
 *   - down_right= corners[xdim * ydim - 1]
 *   - down_left = corners[xdim * (ydim - 1)]
 *
 * @param corners 全部内角点（数量须等于 \c board.width * \c board.height）
 * @param board   标定板内角点尺寸
 * @return 4 个外角点：(up_left, up_right, down_right, down_left)
 */
std::array<cv::Point2f, 4>
get_outside_corners(const std::vector<cv::Point2f>& corners, const cv::Size& board) {
    const int xdim = board.width;
    const int ydim = board.height;
    return { corners[0],
             corners[static_cast<std::size_t>(xdim - 1)],
             corners[static_cast<std::size_t>(xdim * ydim - 1)],
             corners[static_cast<std::size_t>(xdim * (ydim - 1))] };
}

/**
 * @brief 通过外四角计算棋盘投影面积
 *
 * 使用 ROS 原版的"对角向量叉乘的一半"公式：
 * \f[
 *    \text{area} = \frac{|p_x q_y - p_y q_x|}{2}
 * \f]
 * 其中 \f$p = b + c, q = a + b\f$，
 * \f$a = ur - ul, b = dr - ur, c = dl - dr\f$。
 */
double calculate_area(const std::array<cv::Point2f, 4>& outside) {
    const cv::Point2f a = outside[1] - outside[0];
    const cv::Point2f b = outside[2] - outside[1];
    const cv::Point2f c = outside[3] - outside[2];
    const cv::Point2f p = b + c;
    const cv::Point2f q = a + b;
    return std::abs(p.x * q.y - p.y * q.x) / 2.0;
}

/**
 * @brief 通过 up_left/up_right/down_right 三点计算倾斜度
 *
 * 取与 90° 偏离的 2 倍，截断到 [0, 1]，与 ROS 原版完全一致。
 */
double calculate_skew(const std::array<cv::Point2f, 4>& outside) {
    const cv::Point2f& ul = outside[0];
    const cv::Point2f& ur = outside[1];
    const cv::Point2f& dr = outside[2];

    const cv::Point2f ab = ul - ur;
    const cv::Point2f cb = dr - ur;
    const double dot = ab.x * cb.x + ab.y * cb.y;
    const double n_ab = std::hypot(ab.x, ab.y);
    const double n_cb = std::hypot(cb.x, cb.y);
    if (n_ab == 0.0 || n_cb == 0.0) {
        return 1.0;
    }

    double cos_v = dot / (n_ab * n_cb);
    cos_v = std::clamp(cos_v, -1.0, 1.0);
    const double angle = std::acos(cos_v);
    return std::min(1.0, 2.0 * std::abs(CV_PI / 2.0 - angle));
}

/**
 * @brief 计算两组角点平均位移（像素），用于静止性判定
 *
 * 与 ROS 的 \c is_slow_moving 等价；当两组角点数量不一致时返回正无穷，
 * 让上层将其视为非静止。
 */
double average_corner_motion(
    const std::vector<cv::Point2f>& cur, const std::vector<cv::Point2f>& last
) {
    if (cur.empty() || cur.size() != last.size()) {
        return std::numeric_limits<double>::infinity();
    }
    double sum = 0.0;
    for (std::size_t i = 0; i < cur.size(); ++i) {
        sum += cv::norm(cur[i] - last[i]);
    }
    return sum / static_cast<double>(cur.size());
}

/**
 * @brief 将 ROS \c calibrator.py 的 4 维 L1 距离封装成 lambda
 */
double param_distance(const AutoCollector::Params& a, const AutoCollector::Params& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y) + std::abs(a.size - b.size)
        + std::abs(a.skew - b.skew);
}

} // namespace

AutoCollector::AutoCollector(const cv::Size& board_size):
    AutoCollector(board_size, Config {}) {}

AutoCollector::AutoCollector(const cv::Size& board_size, const Config& config):
    board_size_(board_size), config_(config) {
    // 让"启用即可采"，第一次切换到启用状态前不做时间约束
    last_collect_time_ = std::chrono::steady_clock::now()
        - std::chrono::milliseconds(std::max(config_.min_interval_ms * 4, 1000));
}

bool AutoCollector::compute_params(
    const std::vector<cv::Point2f>& corners,
    const cv::Size&                 image_size,
    Params&                         out
) const {
    const std::size_t expected =
        static_cast<std::size_t>(board_size_.width * board_size_.height);
    if (corners.size() != expected || image_size.width <= 0 || image_size.height <= 0) {
        return false;
    }

    const auto outside = get_outside_corners(corners, board_size_);
    const double area  = calculate_area(outside);
    const double skew  = calculate_skew(outside);
    const double border = std::sqrt(area);

    // 与 ROS 一致：把 X/Y "向内收缩半个棋盘"，避免大棋盘被惩罚
    const double width  = static_cast<double>(image_size.width);
    const double height = static_cast<double>(image_size.height);

    double mean_x = 0.0;
    double mean_y = 0.0;
    for (const auto& c : corners) {
        mean_x += c.x;
        mean_y += c.y;
    }
    mean_x /= static_cast<double>(corners.size());
    mean_y /= static_cast<double>(corners.size());

    const double denom_x = std::max(1.0, width  - border);
    const double denom_y = std::max(1.0, height - border);
    out.x    = std::clamp((mean_x - border / 2.0) / denom_x, 0.0, 1.0);
    out.y    = std::clamp((mean_y - border / 2.0) / denom_y, 0.0, 1.0);
    out.size = std::sqrt(area / (width * height));
    out.skew = skew;
    return true;
}

bool AutoCollector::is_good_sample(
    const Params&                   params,
    const std::vector<cv::Point2f>& corners,
    const std::vector<cv::Point2f>& last_corners
) const {
    if (!interval_ready()) {
        return false;
    }

    if (db_params_.empty()) {
        // 第一帧无需静止性检查（与 ROS 相同语义）
        return true;
    }

    double min_dist = std::numeric_limits<double>::infinity();
    for (const auto& p : db_params_) {
        min_dist = std::min(min_dist, param_distance(params, p));
    }
    if (min_dist <= config_.param_distance_threshold) {
        return false;
    }

    if (config_.max_chessboard_speed > 0.0) {
        const double motion = average_corner_motion(corners, last_corners);
        if (motion > config_.max_chessboard_speed) {
            return false;
        }
    }

    return true;
}

void AutoCollector::add_sample(const Params& params) {
    db_params_.push_back(params);
    last_collect_time_ = std::chrono::steady_clock::now();
}

void AutoCollector::update_last_frame(const std::vector<cv::Point2f>& /*corners*/) {
    // 当前实现不需要保留历史角点（静止性检查由调用方提供 last_corners），
    // 占位接口便于将来拓展，例如内嵌历史角点缓存。
}

AutoCollector::Progress AutoCollector::compute_progress() const {
    Progress report;
    report.sample_count = db_params_.size();
    if (db_params_.empty()) {
        return report;
    }

    std::array<double, 4> mn { { db_params_.front().x,
                                 db_params_.front().y,
                                 db_params_.front().size,
                                 db_params_.front().skew } };
    std::array<double, 4> mx = mn;
    for (const auto& p : db_params_) {
        mn[0] = std::min(mn[0], p.x);
        mn[1] = std::min(mn[1], p.y);
        mn[2] = std::min(mn[2], p.size);
        mn[3] = std::min(mn[3], p.skew);
        mx[0] = std::max(mx[0], p.x);
        mx[1] = std::max(mx[1], p.y);
        mx[2] = std::max(mx[2], p.size);
        mx[3] = std::max(mx[3], p.skew);
    }
    // 与 ROS 完全一致：Size/Skew 不奖励"小值起点"
    mn[2] = 0.0;
    mn[3] = 0.0;

    bool all_full = true;
    for (int i = 0; i < 4; ++i) {
        const double range = std::max(1e-6, config_.param_ranges[i]);
        const double prog  = std::min((mx[i] - mn[i]) / range, 1.0);
        report.min_params[i] = mn[i];
        report.max_params[i] = mx[i];
        report.progress[i]   = prog;
        if (prog < 1.0) {
            all_full = false;
        }
    }
    report.good_enough =
        report.sample_count >= config_.goodenough_samples || all_full;
    return report;
}

void AutoCollector::draw_progress(cv::Mat& img, const Params* current_params) const {
    if (img.empty()) {
        return;
    }

    const Progress report = compute_progress();
    const std::array<const char*, 4> labels { { "X", "Y", "Size", "Skew" } };
    const std::array<double, 4> cur_vals { {
        current_params ? current_params->x    : -1.0,
        current_params ? current_params->y    : -1.0,
        current_params ? current_params->size : -1.0,
        current_params ? current_params->skew : -1.0,
    } };

    // 布局参数（左下角面板）
    const int bar_w   = 200;
    const int bar_h   = 16;
    const int label_w = 56;
    const int count_w = 60;
    const int gap     = 5;
    const int margin  = 10;

    const int title_h = 22;
    const int total_h = title_h + 4 * (bar_h + gap);
    const int total_w = label_w + bar_w + count_w + 8;
    const int x0 = margin;
    const int y0 = img.rows - margin - total_h;

    // 半透明背景
    cv::Rect panel(x0 - 6, y0 - 6, total_w + 12, total_h + 12);
    panel &= cv::Rect(0, 0, img.cols, img.rows);
    if (panel.area() > 0) {
        cv::Mat roi = img(panel);
        cv::Mat dark(roi.size(), roi.type(), cv::Scalar(20, 20, 20));
        cv::addWeighted(roi, 0.35, dark, 0.65, 0, roi);
        cv::rectangle(img, panel, cv::Scalar(120, 120, 120), 1);
    }

    const std::string title = fmt::format(
        "Auto Collect  N={}{}", report.sample_count,
        report.good_enough ? "  [GOOD]" : ""
    );
    cv::putText(
        img, title, { x0, y0 + title_h - 6 }, cv::FONT_HERSHEY_SIMPLEX, 0.55,
        report.good_enough ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 255, 255), 1
    );

    for (int i = 0; i < 4; ++i) {
        const int y = y0 + title_h + i * (bar_h + gap);
        const int bx = x0 + label_w;

        cv::putText(
            img, labels[i], { x0, y + bar_h - 4 }, cv::FONT_HERSHEY_SIMPLEX,
            0.42, { 200, 200, 200 }, 1
        );

        // 进度条背景
        cv::rectangle(img, { bx, y }, { bx + bar_w, y + bar_h }, { 70, 70, 70 }, -1);

        // 已覆盖区段（绿色，从 lo 到 hi）
        const int lo_px = bx + static_cast<int>(report.min_params[i] * bar_w);
        const int hi_px = bx + static_cast<int>(report.max_params[i] * bar_w);
        const cv::Scalar bar_color = report.progress[i] >= 1.0
            ? cv::Scalar(0, 200, 0)
            : cv::Scalar(0, 165, 255);
        if (hi_px > lo_px) {
            cv::rectangle(img, { lo_px, y + 1 }, { hi_px, y + bar_h - 1 }, bar_color, -1);
        }

        // 当前值指示线（蓝色）
        if (cur_vals[i] >= 0.0) {
            int px = bx + static_cast<int>(std::clamp(cur_vals[i], 0.0, 1.0) * bar_w);
            px = std::clamp(px, bx, bx + bar_w - 1);
            cv::line(img, { px, y }, { px, y + bar_h }, { 255, 200, 0 }, 2);
        }

        // 右侧文本：进度百分比
        const std::string text =
            fmt::format("{:>3}%", static_cast<int>(report.progress[i] * 100.0));
        cv::putText(
            img, text, { bx + bar_w + 4, y + bar_h - 4 }, cv::FONT_HERSHEY_SIMPLEX,
            0.42, { 220, 220, 220 }, 1
        );
    }
}

void AutoCollector::reset() {
    db_params_.clear();
    last_collect_time_ = std::chrono::steady_clock::now()
        - std::chrono::milliseconds(std::max(config_.min_interval_ms * 4, 1000));
}

void AutoCollector::set_enabled(bool enabled) {
    if (enabled && !enabled_) {
        // 重新打开时允许立刻采一帧
        last_collect_time_ = std::chrono::steady_clock::now()
            - std::chrono::milliseconds(std::max(config_.min_interval_ms * 4, 1000));
    }
    enabled_ = enabled;
}

bool AutoCollector::interval_ready() const {
    if (config_.min_interval_ms <= 0) {
        return true;
    }
    const auto now     = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                             now - last_collect_time_)
                             .count();
    return elapsed >= config_.min_interval_ms;
}

} // namespace qd::calibrate
