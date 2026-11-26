#include "robot/detector.hpp"
#include "robot/yolo.hpp"

#include <rknn_api.h>
#ifdef ROBOT_WITH_RGA
#include <im2d.h>
#endif

#include <array>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace robot {
namespace {

void check(int result, const char* operation) {
    if (result != RKNN_SUCC) {
        throw std::runtime_error(std::string(operation) + " failed (" + std::to_string(result) + ")");
    }
}

std::vector<std::string> read_labels(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot read labels: " + path);
    std::vector<std::string> labels;
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!line.empty()) labels.push_back(line);
    }
    if (file.bad() || labels.empty() || labels.size() > 1024) {
        throw std::runtime_error("expected 1 to 1024 labels in " + path);
    }
    return labels;
}

std::string json_label(const std::string& label) {
    std::string value;
    for (const char c : label) {
        if (c == '"' || c == '\\') value.push_back('\\');
        if (static_cast<unsigned char>(c) >= 0x20) value.push_back(c);
    }
    return value;
}

#ifdef ROBOT_WITH_RGA
bool rga_rgb(const Frame& frame, const Letterbox& geometry, std::vector<std::uint8_t>& rgb) {
    const auto& image = frame.image;
    if (frame.dma_fd < 0 || (image.bt709 && image.full_range)) return false;
    rgb.assign(static_cast<std::size_t>(geometry.input_width) * geometry.input_height * 3, 114);
    auto src = wrapbuffer_fd_t(frame.dma_fd, image.width, image.height, image.stride,
                               image.height, RK_FORMAT_YCbCr_420_SP);
    src.color_space_mode = image.bt709 ? IM_YUV_TO_RGB_BT709_LIMIT
                                       : (image.full_range ? IM_YUV_TO_RGB_BT601_FULL
                                                           : IM_YUV_TO_RGB_BT601_LIMIT);
    auto dst = wrapbuffer_virtualaddr_t(rgb.data(), geometry.input_width, geometry.input_height,
                                        geometry.input_width, geometry.input_height, RK_FORMAT_RGB_888);
    const im_rect from{0, 0, image.width, image.height};
    const im_rect to{geometry.x, geometry.y, geometry.width, geometry.height};
    const auto status = improcess(src, dst, {}, from, to, {}, IM_SYNC);
    if (status != IM_STATUS_SUCCESS) {
        std::cerr << "RGA preprocessing failed (" << static_cast<int>(status) << "); using CPU preprocessing\n";
        return false;
    }
    return true;
}
#endif

}  // namespace

struct Detector::Impl {
    const Config config;
    std::vector<std::string> labels;
    rknn_context context{};
    std::ofstream events;
    int input_width{};
    int input_height{};
    std::array<YoloHead, 3> heads{};
    std::vector<std::uint8_t> rgb;
    bool use_rga{};
    bool rga_reported{};

    explicit Impl(const Config& settings)
        : config(settings), labels(read_labels(settings.labels)), use_rga(settings.use_rga) {}
    ~Impl() { if (context) rknn_destroy(context); }

    void init() {
        std::ifstream file(config.model, std::ios::binary | std::ios::ate);
        const auto length = file.tellg();
        if (!file || length <= 0 || length > 512 * 1024 * 1024) {
            throw std::runtime_error("cannot read RK3568 model: " + config.model);
        }
        std::vector<char> model(static_cast<std::size_t>(length));
        file.seekg(0);
        if (!file.read(model.data(), static_cast<std::streamsize>(length))) {
            throw std::runtime_error("incomplete RK3568 model: " + config.model);
        }
        check(rknn_init(&context, model.data(), static_cast<std::uint32_t>(model.size()), 0, nullptr),
              "rknn_init (check RK3568 export and BSP runtime version)");

        rknn_input_output_num count{};
        check(rknn_query(context, RKNN_QUERY_IN_OUT_NUM, &count, sizeof(count)), "RKNN_QUERY_IN_OUT_NUM");
        if (count.n_input != 1 || count.n_output != heads.size()) {
            throw std::runtime_error("expected Rockchip YOLOv5 with one input and three sigmoid heads");
        }
        rknn_tensor_attr input{};
        check(rknn_query(context, RKNN_QUERY_INPUT_ATTR, &input, sizeof(input)), "RKNN_QUERY_INPUT_ATTR");
        const bool nhwc = input.fmt == RKNN_TENSOR_NHWC;
        if (input.n_dims != 4 || input.dims[0] != 1 || input.dims[nhwc ? 3 : 1] != 3 ||
            (!nhwc && input.fmt != RKNN_TENSOR_NCHW)) {
            throw std::runtime_error("expected a batch-1 RGB model input");
        }
        input_height = static_cast<int>(input.dims[nhwc ? 1 : 2]);
        input_width = static_cast<int>(input.dims[nhwc ? 2 : 3]);
        if (input_width != 640 || input_height != 640) {
            throw std::runtime_error("expected a 640x640 Rockchip YOLOv5 model");
        }

        std::array<bool, 3> strides{};
        for (std::uint32_t index = 0; index < heads.size(); ++index) {
            rknn_tensor_attr attr{};
            attr.index = index;
            check(rknn_query(context, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr)), "RKNN_QUERY_OUTPUT_ATTR");
            const bool last = attr.fmt == RKNN_TENSOR_NHWC;
            const auto channels = 3 * (labels.size() + 5);
            const auto height = attr.dims[last ? 1 : 2];
            const auto width = attr.dims[last ? 2 : 3];
            if (attr.n_dims != 4 || attr.dims[0] != 1 || (!last && attr.fmt != RKNN_TENSOR_NCHW) ||
                attr.dims[last ? 3 : 1] != channels || width != height ||
                (width != 80 && width != 40 && width != 20)) {
                throw std::runtime_error("unexpected YOLOv5 output layout; check model and class labels");
            }
            const int level = width == 80 ? 0 : (width == 40 ? 1 : 2);
            if (strides[level]) throw std::runtime_error("duplicate YOLOv5 output stride");
            strides[level] = true;
            heads[index] = {nullptr, channels * width * height, static_cast<int>(width),
                            static_cast<int>(height), last};
        }
        events.open(config.event_log, std::ios::app);
        if (!events) throw std::runtime_error("cannot open event log: " + config.event_log);
    }
};

Detector::Detector(const Config& config) : impl_(std::make_unique<Impl>(config)) { impl_->init(); }
Detector::~Detector() = default;

void Detector::process(FramePtr frame) {
    auto& state = *impl_;
    const auto sequence = frame->sequence;
    const auto geometry = make_letterbox(frame->image.width, frame->image.height,
                                         state.input_width, state.input_height);
#ifdef ROBOT_WITH_RGA
    if (state.use_rga) {
        if (!rga_rgb(*frame, geometry, state.rgb)) {
            state.use_rga = false;
            std::cerr << "RGA unavailable for this buffer or format; CPU preprocessing enabled\n";
        } else if (!state.rga_reported) {
            state.rga_reported = true;
            std::cerr << "Preprocessing: RGA hardware (NV12 DMA-BUF to RGB)\n";
        }
    }
#endif
    if (!state.use_rga) nv12_to_rgb(frame->image, geometry, state.rgb);
    frame.reset();

    rknn_input input{};
    input.buf = state.rgb.data();
    input.size = static_cast<std::uint32_t>(state.rgb.size());
    input.type = RKNN_TENSOR_UINT8;
    input.fmt = RKNN_TENSOR_NHWC;
    check(rknn_inputs_set(state.context, 1, &input), "rknn_inputs_set");
    check(rknn_run(state.context, nullptr), "rknn_run");
    std::array<rknn_output, 3> outputs{};
    constexpr std::uint32_t output_count = 3;
    for (std::uint32_t index = 0; index < output_count; ++index) {
        outputs[index].index = index;
        outputs[index].want_float = 1;
    }
    check(rknn_outputs_get(state.context, output_count, outputs.data(), nullptr), "rknn_outputs_get");
    struct Release {
        rknn_context context;
        rknn_output* outputs;
        ~Release() { rknn_outputs_release(context, output_count, outputs); }
    } release{state.context, outputs.data()};

    auto heads = state.heads;
    for (std::size_t index = 0; index < heads.size(); ++index) {
        if (outputs[index].size != heads[index].elements * sizeof(float)) {
            throw std::runtime_error("RKNN output does not match the packed FP32 tensor size");
        }
        heads[index].data = static_cast<const float*>(outputs[index].buf);
    }
    const auto boxes = decode_yolo(heads, geometry, static_cast<int>(state.labels.size()),
                                   state.config.score_threshold, state.config.nms_threshold);
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const auto time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
    for (const auto& box : boxes) {
        if (box.score < state.config.event_threshold) continue;
        state.events << "{\"time_ms\":" << time_ms << ",\"sequence\":" << sequence
                     << ",\"label\":\"" << json_label(state.labels[box.class_id])
                     << "\",\"confidence\":" << std::fixed << std::setprecision(4) << box.score
                     << ",\"bbox\":[" << box.left << ',' << box.top << ',' << box.right
                     << ',' << box.bottom << "]}\n";
    }
    state.events.flush();
    if (!state.events) throw std::runtime_error("failed writing " + state.config.event_log);
}

}  // namespace robot
