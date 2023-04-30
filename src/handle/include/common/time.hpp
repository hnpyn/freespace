#ifndef COMMON_LIBS_INCLUDE_COMMON_TIME_HPP_
#define COMMON_LIBS_INCLUDE_COMMON_TIME_HPP_

#include <ros/ros.h>   // ros::Time::now()
#include <sys/time.h>  // gettimeofday
#include <string>
#include <chrono>

namespace lidar {
namespace common {
//----------------------------------- Timer helper class
class Timer
{
private:
    std::chrono::steady_clock::time_point record;
public:
    Timer() { tic(); }
    ~Timer() {}
    void tic() {
        record = std::chrono::steady_clock::now();
    }
    double toc(bool reset = false) {
        auto now = std::chrono::steady_clock::now();
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(now - record).count();
        if (reset) {
            tic();
        }
        return time;
    }
};

class Clock {
 public:
    Clock() { start(); }

    /// \brief Start clock timer.
    void start() {
        gettimeofday(&real_time_start_, NULL);
        cpu_start_ = clock();
    }

    /// \brief Sample clock timer.
    void takeTime() {
        struct timeval end;
        gettimeofday(&end, NULL);
        cpu_time_ms_ = static_cast<double>(clock() - cpu_start_) /
                       CLOCKS_PER_SEC * kSecondsToMiliseconds;

        uint64_t seconds, useconds;

        seconds = end.tv_sec - real_time_start_.tv_sec;
        useconds = end.tv_usec - real_time_start_.tv_usec;
        real_time_ms_ = (seconds * kSecondsToMiliseconds +
                         useconds * kMicrosecondsToMiliseconds) +
                        0.5;
    }

    /// \brief Return elapsed physical time.
    double getRealTime() { return real_time_ms_; }

    /// \brief Return elapsed CPU time.
    double getCPUTime() { return cpu_time_ms_; }

    double takeRealTime() {
        takeTime();
        return getRealTime();
    }

 private:
    struct timeval real_time_start_;
    double real_time_ms_, cpu_time_ms_;
    clock_t cpu_start_;

    static constexpr double kSecondsToMiliseconds = 1000.0;
    static constexpr double kMicrosecondsToMiliseconds = 0.001;
};

static std::string getCurrentTimestampString() {
    uint64_t current_time = ros::Time::now().toSec() * 100;
    return std::to_string(current_time);
}

static std::string  Get_Current_Date()
{
    time_t nowtime;
    nowtime = time(NULL); //获取日历时间
    char tmp[64];
    strftime(tmp,sizeof(tmp),"%Y_%m_%d_%H_%M_%S",localtime(&nowtime));
    return tmp;
}

}  // namespace common
}  // namespace lidar

#endif  // COMMON_LIBS_INCLUDE_COMMON_TIME_HPP_
