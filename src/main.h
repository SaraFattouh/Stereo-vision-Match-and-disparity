#pragma once

void StereoEstimation_Naive(
  const int& window_size,
  const int& dmin,
  int height,
  int width,
  cv::Mat& image1, cv::Mat& image2, cv::Mat& naive_disparities, const double& scale);

void Disparity2PointCloud(
  const std::string& output_file,
  int height, int width, cv::Mat& disparities,
  const int& window_size,
  const int& dmin, const double& baseline, const double& focal_length);

void StereoEstimation_Dynamic(
    const int& window_size,
    int height,
    int width,
    cv::Mat& image1, cv::Mat& image2, const double& weight, const std::string& output_file);
