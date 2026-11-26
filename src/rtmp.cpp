#include "robot/rtmp.hpp"

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <iostream>
#include <utility>

#include <fcntl.h>
#include <poll.h>
#include <spawn.h>
#include <sys/wait.h>

extern char** environ;

namespace robot {

Rtmp::Rtmp(const Config& config, std::string fps)
    : binary_(config.ffmpeg), url_(config.rtmp_url), fps_(std::move(fps)) {}

Rtmp::~Rtmp() { stop(false); }

bool Rtmp::start() {
    int fds[2];
    if (::pipe2(fds, O_CLOEXEC) < 0) return false;
    UniqueFd read_end(fds[0]);
    UniqueFd write_end(fds[1]);
    if (read_end.get() == STDIN_FILENO) {
        const int duplicate = ::fcntl(read_end.get(), F_DUPFD_CLOEXEC, STDERR_FILENO + 1);
        if (duplicate < 0) return false;
        read_end.reset(duplicate);
    }
    if (::fcntl(write_end.get(), F_SETFL, O_NONBLOCK) < 0) return false;

    posix_spawn_file_actions_t actions;
    if (::posix_spawn_file_actions_init(&actions) != 0) return false;
    int error = ::posix_spawn_file_actions_adddup2(&actions, read_end.get(), STDIN_FILENO);
    const char* arguments[] = {binary_.c_str(), "-hide_banner", "-loglevel", "warning", "-nostdin",
                               "-fflags", "+genpts", "-f", "h264", "-framerate", fps_.c_str(), "-i", "pipe:0",
                               "-map", "0:v:0", "-c:v", "copy", "-f", "flv", url_.c_str()};
    char* argv[sizeof(arguments) / sizeof(arguments[0]) + 1]{};
    for (std::size_t index = 0; index < sizeof(arguments) / sizeof(arguments[0]); ++index) {
        argv[index] = const_cast<char*>(arguments[index]);
    }
    pid_t child = -1;
    if (error == 0) error = ::posix_spawnp(&child, binary_.c_str(), &actions, nullptr, argv, environ);
    ::posix_spawn_file_actions_destroy(&actions);
    if (error != 0) {
        std::cerr << "cannot start FFmpeg: " << std::strerror(error) << '\n';
        return false;
    }
    pipe_ = std::move(write_end);
    child_ = child;
    started_once_ = true;
    return true;
}

void Rtmp::stop(bool drain) noexcept {
    pipe_.reset();
    if (child_ <= 0) return;
    const auto wait_child = [&]() {
        for (int attempt = 0; attempt < 30; ++attempt) {
            const auto result = ::waitpid(child_, nullptr, WNOHANG);
            if (result == child_ || (result < 0 && errno == ECHILD)) return true;
            ::usleep(10'000);
        }
        return false;
    };
    if (!drain || !wait_child()) {
        ::kill(child_, SIGTERM);
        if (!wait_child()) {
            ::kill(child_, SIGKILL);
            while (::waitpid(child_, nullptr, 0) < 0 && errno == EINTR) {}
        }
    }
    child_ = -1;
}

void Rtmp::retry() {
    stop(false);
    const auto seconds = 1 << std::min(failures_, 4U);
    failures_ = std::min(failures_ + 1, 4U);
    retry_at_ = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
    std::cerr << "RTMP disconnected; retrying on an IDR after " << seconds << " s\n";
}

void Rtmp::publish(const std::uint8_t* data, std::size_t size, bool idr) {
    using Clock = std::chrono::steady_clock;
    if (url_.empty() || size == 0) return;
    if (!pipe_) {
        if (!idr || Clock::now() < retry_at_) return;
        if (!start()) { retry(); return; }
    }
    const auto deadline = Clock::now() + std::chrono::milliseconds(350);
    while (size != 0) {
        const auto written = ::write(pipe_.get(), data, size);
        if (written > 0) {
            data += written;
            size -= static_cast<std::size_t>(written);
            continue;
        }
        if (Clock::now() >= deadline) { retry(); return; }
        if (written < 0 && errno == EINTR) continue;
        if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            pollfd fd{pipe_.get(), POLLOUT, 0};
            const auto wait = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
            const auto ready = ::poll(&fd, 1, std::max(0, static_cast<int>(wait.count())));
            if ((ready > 0 && (fd.revents & POLLOUT)) || (ready < 0 && errno == EINTR)) continue;
        }
        retry();
        return;
    }
    // Keep backoff increasing if FFmpeg repeatedly exits on its initial IDR.
    if (!idr) failures_ = 0;
}

void Rtmp::finish() noexcept {
    stop(true);
    if (!url_.empty() && !started_once_) {
        std::cerr << "FFmpeg did not start an RTMP session\n";
    }
}

}  // namespace robot
