#pragma once

#include <cstdint>
#include <string>

namespace robot {

struct Config {
    std::string video_device{"auto"};
    int width{1280};
    int height{720};
    int fps{30};
    int buffers{8};
    int frames{120};  // 0 runs until SIGINT or SIGTERM.
    std::string snapshot;

    std::string model;
    std::string labels{"model/coco_80_labels_list.txt"};
    std::string event_log{"inspection_events.ndjson"};
    // use_rga参数表示是否使用RGA进行图像缩放，false表示不使用 true表示使用RGA进行图像缩放.
    bool use_rga{false};
    float score_threshold{0.25F};
    float event_threshold{0.60F};
    float nms_threshold{0.45F};
    int inference_queue{1};

    std::string h264_file;
    std::string rtmp_url;
    std::string ffmpeg{"ffmpeg"};
    int bitrate{2'500'000};
    int stream_queue{2};
};

Config load_config(const std::string& path);

}  // namespace robot
