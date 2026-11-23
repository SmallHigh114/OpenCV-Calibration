#include "image_reader.hpp"
#include <algorithm>
#include <array>
#include <filesystem>
#include <iostream>

namespace {

std::filesystem::path resolve_images_path(
    const std::string& config_path, const std::string& configured_path
) {
    std::filesystem::path images_path(configured_path);
    if (images_path.is_absolute()) {
        return images_path.lexically_normal();
    }

    const std::filesystem::path config_dir =
        std::filesystem::absolute(std::filesystem::path(config_path)).parent_path();
    return (config_dir / images_path).lexically_normal();
}

void append_glob_matches(
    const std::filesystem::path& images_path,
    const std::string& extension,
    std::vector<cv::String>& filenames
) {
    std::vector<cv::String> matched_files;
    cv::glob((images_path / extension).string(), matched_files);
    filenames.insert(filenames.end(), matched_files.begin(), matched_files.end());
}

} // namespace

namespace qd::Device {

Image_Reader::Image_Reader(const std::string& config_path): index(0) {
    auto yaml = YAML::LoadFile(config_path);
    const auto configured_images_path = yaml["IMG"]["images_path"].as<std::string>();
    const auto images_path = resolve_images_path(config_path, configured_images_path);

    if (!std::filesystem::exists(images_path)) {
        std::cerr << "Image path does not exist: " << images_path << std::endl;
        exhausted_ = true;
        reported_exhausted_ = true;
        return;
    }

    // 获取文件夹中所有图片文件
    for (const auto& extension:
         std::array<std::string, 8> { "*.jpg", "*.JPG", "*.png", "*.PNG",
                                      "*.bmp", "*.BMP", "*.tif", "*.TIF" })
    {
        append_glob_matches(images_path, extension, filenames);
    }

    std::sort(filenames.begin(), filenames.end());
    filenames.erase(std::unique(filenames.begin(), filenames.end()), filenames.end());

    if (filenames.empty()) {
        std::cerr << "No images found in: " << images_path << std::endl;
        exhausted_ = true;
        reported_exhausted_ = true;
        return;
    }

    std::cout << "Loaded " << filenames.size() << " images from " << images_path << std::endl;
}

Image_Reader::~Image_Reader() {
    cap.release();
}

cv::Mat Image_Reader::get_image() {
    while (index < static_cast<int>(filenames.size())) {
        const auto current_file = filenames[index++];
        cv::Mat data = cv::imread(current_file);
        if (!data.empty()) {
            return data;
        }

        std::cerr << "Failed to read image: " << current_file << std::endl;
    }

    exhausted_ = true;
    if (!reported_exhausted_) {
        std::cout << "All images have been read." << std::endl;
        reported_exhausted_ = true;
    }

    return {};
}

void Image_Reader::read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) {
    img = get_image();
    timestamp = std::chrono::steady_clock::now();
}

bool Image_Reader::is_exhausted() const {
    return exhausted_;
}
} // namespace qd::Device
