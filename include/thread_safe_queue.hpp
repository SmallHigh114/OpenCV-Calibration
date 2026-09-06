#ifndef TOOLS__THREAD_SAFE_QUEUE_HPP
#define TOOLS__THREAD_SAFE_QUEUE_HPP

#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>

namespace tools {
template<typename T, bool PopWhenFull = false>
class ThreadSafeQueue {
public:
    /**
     * @brief 构造线程安全队列
     * @param max_size 最大容量
     * @param full_handler 队列满时的回调
     */
    ThreadSafeQueue(
        size_t max_size,
        std::function<void(void)> full_handler = [] {}
    ):
        max_size_(max_size),
        full_handler_(full_handler) {}

    /**
     * @brief 入队
     * @param value 待入队元素
     */
    void push(const T& value) {
        std::unique_lock<std::mutex> lock(mutex_);

        if (queue_.size() >= max_size_) {
            if (PopWhenFull) {
                queue_.pop();
            } else {
                full_handler_();
                return;
            }
        }

        queue_.push(value);
        not_empty_condition_.notify_all();
    }

    /**
     * @brief 阻塞式出队到引用
     * @param value 输出元素
     */
    void pop(T& value) {
        std::unique_lock<std::mutex> lock(mutex_);

        not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

        if (queue_.empty()) {
            std::cerr << "Error: Attempt to pop from an empty queue." << std::endl;
            return;
        }

        value = queue_.front();
        queue_.pop();
    }

    /**
     * @brief 阻塞式出队并返回
     * @return T 出队元素
     */
    T pop() {
        std::unique_lock<std::mutex> lock(mutex_);

        not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

        T value = std::move(queue_.front());
        queue_.pop();
        return std::move(value);
    }

    /**
     * @brief 阻塞式获取队首元素（不出队）
     * @return T 队首元素
     */
    T front() {
        std::unique_lock<std::mutex> lock(mutex_);

        not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

        return queue_.front();
    }

    /**
     * @brief 获取队尾元素（不出队）
     * @param value 输出元素
     */
    void back(T& value) {
        std::unique_lock<std::mutex> lock(mutex_);

        if (queue_.empty()) {
            std::cerr << "Error: Attempt to access the back of an empty queue." << std::endl;
            return;
        }

        value = queue_.back();
    }

    /**
     * @brief 判断队列是否为空
     * @return true 空
     * @return false 非空
     */
    bool empty() {
        std::unique_lock<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    /**
     * @brief 清空队列并唤醒等待线程
     */
    void clear() {
        std::unique_lock<std::mutex> lock(mutex_);
        while (!queue_.empty()) {
            queue_.pop();
        }
        not_empty_condition_.notify_all(); // 如果其他线程正在等待队列不为空，这样可以唤醒它们
    }

private:
    std::queue<T> queue_;
    size_t max_size_;
    mutable std::mutex mutex_;
    std::condition_variable not_empty_condition_;
    std::function<void(void)> full_handler_;
};

} // namespace tools

#endif // TOOLS__THREAD_SAFE_QUEUE_HPP