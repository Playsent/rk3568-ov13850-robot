#pragma once

#include "robot/config.hpp"
#include "robot/unique_fd.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <sys/types.h>

namespace robot {

class Rtmp {
public:
    Rtmp(const Config& config, std::string fps);
    ~Rtmp();
    void publish(const std::uint8_t* data, std::size_t size, bool idr);
    void finish() noexcept;

private:
    bool start();
    void stop(bool drain) noexcept;
    void retry();

    std::string binary_;
    std::string url_;
    std::string fps_;
    UniqueFd pipe_;
    pid_t child_{-1};
    unsigned failures_{};
    bool started_once_{};
    std::chrono::steady_clock::time_point retry_at_{};
};

}  // namespace robot
