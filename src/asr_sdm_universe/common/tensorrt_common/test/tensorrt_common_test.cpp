#include "tensorrt_common/engine.hpp"
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <chrono>
#include <cmath>

namespace fs = std::filesystem;

namespace {
std::string resolveOnnxPath(const std::string& path) {
    std::error_code ec;
    if (fs::is_regular_file(path, ec)) {
        if (path.size() >= 5 && path.substr(path.size() - 5) == ".onnx") {
            return path;
        }
        throw std::runtime_error("Expected an .onnx file, got: " + path);
    }

    if (!fs::is_directory(path, ec)) {
        throw std::runtime_error("Path is neither a file nor a directory: " + path);
    }

    const fs::path preferred = fs::path(path) / "uie_model.onnx";
    if (fs::is_regular_file(preferred, ec)) {
        return preferred.string();
    }

    for (const auto& entry : fs::directory_iterator(path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".onnx") {
            return entry.path().string();
        }
    }

    throw std::runtime_error("No .onnx file found in directory: " + path);
}

std::string resolveTestImage(const std::string& onnxModelPath, int argc, char* argv[]) {
    std::error_code ec;
    if (argc >= 3) {
        if (!fs::is_regular_file(argv[2], ec)) {
            throw std::runtime_error("Unable to find image at path: " + std::string(argv[2]));
        }
        return argv[2];
    }

    const fs::path packageRoot = fs::path(onnxModelPath).parent_path().parent_path();
    const std::vector<fs::path> candidates = {
        packageRoot / "inputs" / "test_image.png",
        fs::path(onnxModelPath).parent_path() / "test_image.png",
        fs::path("../inputs/test_image.png"),
    };
    for (const auto& candidate : candidates) {
        if (fs::is_regular_file(candidate, ec)) {
            return fs::weakly_canonical(candidate, ec).string();
        }
    }
    throw std::runtime_error(
        "No test image found. Pass one as argv[2], or place inputs/test_image.png in the package.");
}

// Convert NCHW RGB float tensor (optionally in [0,1]) to BGR uint8 image.
cv::Mat chwRgbToBgrU8(const std::vector<float>& data, int channels, int height, int width) {
    if (channels != 3) {
        throw std::runtime_error("Expected 3-channel image output, got channels=" + std::to_string(channels));
    }
    const size_t plane = static_cast<size_t>(height) * static_cast<size_t>(width);
    if (data.size() < plane * 3) {
        throw std::runtime_error("Output tensor too small for HxWxC image");
    }

    // Detect whether values look normalized to [0,1].
    float maxAbs = 0.f;
    for (size_t i = 0; i < std::min(data.size(), plane * 3); ++i) {
        maxAbs = std::max(maxAbs, std::abs(data[i]));
    }
    const bool normalized = maxAbs <= 1.5f;
    const float scale = normalized ? 255.f : 1.f;

    cv::Mat bgr(height, width, CV_8UC3);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const size_t idx = static_cast<size_t>(y) * width + x;
            const float r = data[0 * plane + idx] * scale;
            const float g = data[1 * plane + idx] * scale;
            const float b = data[2 * plane + idx] * scale;
            bgr.at<cv::Vec3b>(y, x) = cv::Vec3b(
                cv::saturate_cast<uchar>(b),
                cv::saturate_cast<uchar>(g),
                cv::saturate_cast<uchar>(r));
        }
    }
    return bgr;
}
}  // namespace

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cout << "Error: Must specify the model path" << std::endl;
        std::cout << "Usage: " << argv[0] << " /path/to/model.onnx [optional_image.png]" << std::endl;
        std::cout << "   or: " << argv[0] << " /path/to/model/directory [optional_image.png]" << std::endl;
        return -1;
    }

    if (argc > 3) {
        std::cout << "Error: Too many arguments provided" << std::endl;
        std::cout << "Usage: " << argv[0] << " /path/to/model.onnx [optional_image.png]" << std::endl;
        return -1;
    }

    std::string onnxModelPath;
    std::string inputImagePath;
    try {
        onnxModelPath = resolveOnnxPath(argv[1]);
        inputImagePath = resolveTestImage(onnxModelPath, argc, argv);
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
        return -1;
    }
    std::cout << "Using ONNX model: " << onnxModelPath << std::endl;
    std::cout << "Using input image: " << inputImagePath << std::endl;

    Options options;
    options.precision = Precision::FP16;
    options.calibrationDataDirectoryPath = "";
    options.optBatchSize = 1;
    options.maxBatchSize = 1;

    Engine engine(options);

    // UIE model: normalize inputs to [0, 1]
    std::array<float, 3> subVals {0.f, 0.f, 0.f};
    std::array<float, 3> divVals {1.f, 1.f, 1.f};
    bool normalize = true;

    bool succ = engine.build(onnxModelPath, subVals, divVals, normalize);
    if (!succ) {
        throw std::runtime_error("Unable to build TRT engine.");
    }

    succ = engine.loadNetwork(onnxModelPath);
    if (!succ) {
        throw std::runtime_error("Unable to load TRT engine.");
    }

    cv::Mat cpuImg = cv::imread(inputImagePath);
    if (cpuImg.empty()) {
        throw std::runtime_error("Unable to read image at path: " + inputImagePath);
    }

    cv::cuda::GpuMat img;
    img.upload(cpuImg);
    cv::cuda::cvtColor(img, img, cv::COLOR_BGR2RGB);

    const auto& inputDims = engine.getInputDims();
    const auto& outputDims = engine.getOutputDims();
    std::vector<std::vector<cv::cuda::GpuMat>> inputs;
    std::vector<cv::cuda::GpuMat> input;
    int netH = 256;
    int netW = 256;
    for (size_t j = 0; j < inputDims.size(); ++j) {
        const auto& dims = inputDims[j];
        netH = dims.d[2] > 0 ? dims.d[2] : 256;
        netW = dims.d[3] > 0 ? dims.d[3] : 256;
        auto resized = Engine::resizeKeepAspectRatioPadRightBottom(
            img, static_cast<size_t>(netH), static_cast<size_t>(netW));
        input.emplace_back(std::move(resized));
    }
    inputs.emplace_back(std::move(input));

    // One timed inference for the saved output
    std::vector<std::vector<std::vector<float>>> featureVectors;
    preciseStopwatch stopwatch;
    if (!engine.runInference(inputs, featureVectors)) {
        throw std::runtime_error("Inference failed.");
    }
    auto elapsedMs = stopwatch.elapsedTime<float, std::chrono::milliseconds>();
    std::cout << "Inference time: " << elapsedMs << " ms" << std::endl;

    if (featureVectors.empty() || featureVectors[0].empty()) {
        throw std::runtime_error("No inference outputs returned.");
    }

    // Output dims are typically [N,C,H,W]
    int outC = 3;
    int outH = netH;
    int outW = netW;
    if (!outputDims.empty() && outputDims[0].nbDims >= 4) {
        outC = outputDims[0].d[1] > 0 ? outputDims[0].d[1] : outC;
        outH = outputDims[0].d[2] > 0 ? outputDims[0].d[2] : outH;
        outW = outputDims[0].d[3] > 0 ? outputDims[0].d[3] : outW;
    }

    const auto& outVec = featureVectors[0][0];
    cv::Mat outBgr = chwRgbToBgrU8(outVec, outC, outH, outW);

    const fs::path outPath =
        fs::path(inputImagePath).parent_path() / (fs::path(inputImagePath).stem().string() + "_out.png");
    if (!cv::imwrite(outPath.string(), outBgr)) {
        throw std::runtime_error("Failed to write output image: " + outPath.string());
    }
    std::cout << "Wrote inference output: " << outPath.string() << std::endl;
    std::cout << "Output shape: [" << outC << ", " << outH << ", " << outW
              << "], values: " << outVec.size() << std::endl;

    // Also write a side-by-side comparison (network input | output)
    cv::Mat netInputBgr;
    {
        cv::cuda::GpuMat netIn = inputs[0][0];
        cv::Mat netInRgb;
        netIn.download(netInRgb);
        cv::cvtColor(netInRgb, netInputBgr, cv::COLOR_RGB2BGR);
    }
    cv::Mat comparison;
    cv::hconcat(netInputBgr, outBgr, comparison);
    const fs::path cmpPath =
        fs::path(inputImagePath).parent_path() / (fs::path(inputImagePath).stem().string() + "_compare.png");
    cv::imwrite(cmpPath.string(), comparison);
    std::cout << "Wrote comparison image: " << cmpPath.string() << std::endl;

    return 0;
}
