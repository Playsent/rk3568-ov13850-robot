#include "robot/config.hpp"

#include <charconv>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <unordered_map>

namespace robot {
namespace {

std::string trim(const std::string& value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    return value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1);
}

}  // namespace

Config load_config(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot open config: " + path);

    std::unordered_map<std::string, std::string> values;
    std::string line;
    int line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        line = trim(line);
        if (line.empty() || line.front() == '#') continue;
        const auto split = line.find('=');
        if (split == std::string::npos ||
            !values.emplace(trim(line.substr(0, split)), trim(line.substr(split + 1))).second) {
            throw std::runtime_error("invalid or duplicate entry in " + path + ":" +
                                     std::to_string(line_number));
        }
    }
    if (input.bad()) throw std::runtime_error("cannot read config: " + path);

    const auto take = [&](const char* key, const std::string& fallback) {
        const auto item = values.find(key);
        if (item == values.end()) return fallback;
        const auto value = item->second;
        values.erase(item);
        return value;
    };
    const auto integer = [&](const char* key, int fallback, int lo, int hi) {
        const auto value = take(key, std::to_string(fallback));
        int number = 0;
        const auto end = value.data() + value.size();
        const auto result = std::from_chars(value.data(), end, number);
        if (result.ec != std::errc{} || result.ptr != end || number < lo || number > hi) {
            throw std::runtime_error(std::string("invalid integer for ") + key);
        }
        return number;
    };
    const auto probability = [&](const char* key, float fallback) {
        const auto value = take(key, std::to_string(fallback));
        try {
            std::size_t end = 0;
            const auto number = std::stof(value, &end);
            if (end == value.size() && std::isfinite(number) && number > 0 && number <= 1) {
                return number;
            }
        } catch (const std::invalid_argument&) {
        } catch (const std::out_of_range&) {
        }
        throw std::runtime_error(std::string("expected a number in (0, 1] for ") + key);
    };

    Config config;
    config.video_device = take("video_device", config.video_device);
    config.width = integer("width", config.width, 16, 4096);
    config.height = integer("height", config.height, 16, 4096);
    config.fps = integer("fps", config.fps, 1, 60);
    config.buffers = integer("buffers", config.buffers, 4, 32);
    config.frames = integer("frames", config.frames, 0, 2'000'000);
    config.snapshot = take("snapshot", config.snapshot);
    config.model = take("model", config.model);
    config.labels = take("labels", config.labels);
    config.event_log = take("event_log", config.event_log);
    const auto rga = take("use_rga", "false");
    if (rga != "true" && rga != "false") throw std::runtime_error("invalid use_rga");
    config.use_rga = rga == "true";
    config.score_threshold = probability("score_threshold", config.score_threshold);
    config.event_threshold = probability("event_threshold", config.event_threshold);
    config.nms_threshold = probability("nms_threshold", config.nms_threshold);
    config.inference_queue = integer("inference_queue", config.inference_queue, 1, 8);
    config.h264_file = take("h264_file", config.h264_file);
    config.rtmp_url = take("rtmp_url", config.rtmp_url);
    config.ffmpeg = take("ffmpeg", config.ffmpeg);
    config.bitrate = integer("bitrate", config.bitrate, 100'000, 25'000'000);
    config.stream_queue = integer("stream_queue", config.stream_queue, 1, 8);

    if (!values.empty()) throw std::runtime_error("unknown config key: " + values.begin()->first);
    if (config.width % 2 || config.height % 2 || config.video_device.empty()) {
        throw std::runtime_error("NV12 requires even width and height and a video device");
    }
    if (config.use_rga && config.model.empty()) throw std::runtime_error("use_rga requires model");
    if (!config.model.empty() && (config.labels.empty() || config.event_log.empty() ||
                                  config.event_threshold < config.score_threshold)) {
        throw std::runtime_error("inference requires labels, event_log and event_threshold >= score_threshold");
    }
    if (!config.rtmp_url.empty() &&
        ((config.rtmp_url.rfind("rtmp://", 0) != 0 && config.rtmp_url.rfind("rtmps://", 0) != 0) ||
         config.ffmpeg.empty())) {
        throw std::runtime_error("set an rtmp:// or rtmps:// URL and an FFmpeg executable");
    }
    if ((!config.snapshot.empty() && config.snapshot == config.h264_file) ||
        (!config.model.empty() &&
         (config.event_log == config.snapshot || config.event_log == config.h264_file ||
          config.model == config.snapshot || config.model == config.h264_file ||
          config.model == config.event_log || config.labels == config.snapshot ||
          config.labels == config.h264_file || config.labels == config.event_log))) {
        throw std::runtime_error("use separate paths for model, labels and outputs");
    }
    return config;
}

}  // namespace robot
