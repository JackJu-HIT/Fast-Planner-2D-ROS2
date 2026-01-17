#ifndef QUADROTOR_MSGS_MESSAGE_TIME_H
#define QUADROTOR_MSGS_MESSAGE_TIME_H

#include <chrono>
#include <iostream>
#include <time.h>  // 显式包含 C 时间头文件
#include <ctime>   // 包含 C++ 时间头文件

namespace quadrotor_msgs
{

class Duration;

class Time {
public:
    using Clock = std::chrono::system_clock; // 可根据需要替换为steady_clock
    using TimePoint = Clock::time_point;

    // 默认构造（初始化为时钟的纪元时间）
    Time() : time_point_(Clock::now()) {}

    // 通过chrono的time_point构造
    explicit Time(const TimePoint& tp) : time_point_(tp) {}

    // 获取当前时间
    static Time now() {
        return Time(Clock::now());
    }

    // 转换为秒（double类型，类似ROS的toSec()）
    double toSec() const {
        auto duration = time_point_.time_since_epoch();
        return std::chrono::duration<double>(duration).count();
    }

    // 时间加法：Time + Duration = Time
    inline  Time operator+(const Duration& d) const;

    // 时间减法：Time - Duration = Time
    inline  Time operator-(const Duration& d) const;

    // 时间差：Time - Time = Duration
    inline Duration operator-(const Time& other) const;

    // 比较运算符
    bool operator<(const Time& other) const { return time_point_ < other.time_point_; }
    bool operator>(const Time& other) const { return time_point_ > other.time_point_; }
    bool operator==(const Time& other) const { return time_point_ == other.time_point_; }

private:
    TimePoint time_point_;
};

class Duration {
public:
    using ChronoDuration = std::chrono::nanoseconds; // 使用纳秒存储以支持高精度

    // 默认构造（0时长）
    Duration() : duration_(0) {}

    // 通过chrono的duration构造
    template<typename Rep, typename Period>
    explicit Duration(const std::chrono::duration<Rep, Period>& d)
        : duration_(std::chrono::duration_cast<ChronoDuration>(d)) {}

    // 通过秒（double类型）构造，类似ROS的Duration
    explicit Duration(double seconds)
        : duration_(std::chrono::duration_cast<ChronoDuration>(
              std::chrono::duration<double>(seconds))) {}

    // 转换为秒
    double toSec() const {
        return std::chrono::duration<double>(duration_).count();
    }

    // 获取内部duration（供Time类使用）
    ChronoDuration get() const { return duration_; }

    // 运算符重载
    Duration operator+(const Duration& other) const {
        return Duration(duration_ + other.duration_);
    }

    Duration operator-(const Duration& other) const {
        return Duration(duration_ - other.duration_);
    }

private:
    ChronoDuration duration_;
};

// 实现Time的运算符
Time Time::operator+(const Duration& d) const {
    return Time(time_point_ + d.get());
}

Time Time::operator-(const Duration& d) const {
    return Time(time_point_ - d.get());
}

Duration Time::operator-(const Time& other) const {
    return Duration(time_point_ - other.time_point_);
}

// int main() {
//     Time t1 = Time::now();
//     // 模拟耗时操作
//     for (volatile int i = 0; i < 1000000; ++i) {}
//     Time t2 = Time::now();

//     Duration delta = t2 - t1;
//     std::cout << "Time elapsed: " << delta.toSec() << " seconds" << std::endl;

//     Duration wait_time(0.5); // 0.5秒
//     Time t3 = t2 + wait_time;
//     std::cout << "t2: " << t2.toSec() << ", t3: " << t3.toSec() << std::endl;

//     return 0;
// }
}

#endif