#include <opencv2/opencv.hpp>
#include <iostream>
#include <string> 
#include <fstream>
#include <sstream>
#include "main.h"


int main(int argc, char** argv) {

  ////////////////
  // Parameters //
  ////////////////

  // camera setup parameters
  const double focal_length = 1247;
  const double baseline = 213;

  // stereo estimation parameters
  const int dmin = 67;
  const int window_size = 3;
  const double weight = 500;
  const double scale = 3;

  ///////////////////////////
  // Commandline arguments //
  ///////////////////////////
  int x = argc;
  if (argc < 4) {
    std::cerr << "Usage: " << argv[0] << " IMAGE1 IMAGE2 OUTPUT_FILE OUTPUT_FILE2" << std::endl;
    return 1;
  }

  cv::Mat image1 = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
  cv::Mat image2 = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
  const std::string output_file = argv[3];
  const std::string output2_file = argv[4];

  if (!image1.data) {
    std::cerr << "No image1 data" << std::endl;
    return EXIT_FAILURE;
  }

  if (!image2.data) {
    std::cerr << "No image2 data" << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "------------------ Parameters -------------------" << std::endl;
  std::cout << "focal_length = " << focal_length << std::endl;
  std::cout << "baseline = " << baseline << std::endl;
  std::cout << "window_size = " << window_size << std::endl;
  std::cout << "occlusion weights = " << weight << std::endl;
  std::cout << "disparity added due to image cropping = " << dmin << std::endl;
  std::cout << "scaling of disparity images to show = " << scale << std::endl;
  std::cout << "output filename = " << argv[3] << std::endl;
  std::cout << "-------------------------------------------------" << std::endl;

   int height = image1.size().height;
   int width = image1.size().width;

  ////////////////////
  // Reconstruction //
  ////////////////////

  // Naive disparity image
 
  cv::Mat naive_disparities = cv::Mat::zeros(height, width, CV_8UC1);
  

  StereoEstimation_Naive(
    window_size, dmin, height, width,
    image1, image2,
    naive_disparities, scale);

  ////////////
  // Output //
  ////////////

  // reconstruction
  Disparity2PointCloud(
    output_file,
    height, width, naive_disparities,
    window_size, dmin, baseline, focal_length);

  // save / display images
  std::stringstream out1;
  out1 << output_file << "_naive.png";
  cv::imwrite(out1.str(), naive_disparities);

  cv::namedWindow("Naive", cv::WINDOW_AUTOSIZE);
  cv::imshow("Naive", naive_disparities);

  // Dynamic disparity image
  StereoEstimation_Dynamic(
      window_size,
      height,
      width,
      image1, image2, weight, output2_file); // disparity image is decalred in the function


  cv::waitKey(0);

  return 0;
}

void StereoEstimation_Naive(
  const int& window_size,
  const int& dmin,
  int height,
  int width,
  cv::Mat& image1, cv::Mat& image2, cv::Mat& naive_disparities, const double& scale)
{
  int half_window_size = window_size / 2;

  for (int i = half_window_size; i < height - half_window_size; ++i) {

    std::cout
      << "Calculating disparities for the naive approach... "
      << std::ceil(((i - half_window_size + 1) / static_cast<double>(height - window_size + 1)) * 100) << "%\r"
      << std::flush;

    for (int j = half_window_size; j < width - half_window_size; ++j) {
      int min_ssd = INT_MAX;
      int disparity = 0;

      for (int d = -j + half_window_size; d < width - j - half_window_size; ++d) {
        int ssd = 0;
        int norm_left = 0;
        int norm_right = 0;

        for (int u = -half_window_size; u <= half_window_size; ++u) {
            for (int v = -half_window_size; v <= half_window_size; ++v) {
                int val_left = image1.at<char>(i + u, j + v);
                int val_right = image2.at<char>(i + u, j + v + d);
                norm_left += val_left;
                norm_right += val_right;
            }
        }
        norm_left /= window_size * window_size;
        norm_right /= window_size * window_size;

        norm_left = std::max(1, norm_left);
        norm_right = std::max(1, norm_right);

        // TODO: sum up matching cost (ssd) in a window

        for (int u = -half_window_size; u <= half_window_size; ++u) {
            for (int v = -half_window_size; v <= half_window_size; ++v) {
                int val_left = image1.at<char>(i + u, j + v);
                int val_right = image2.at<char>(i + u, j + v +d);
                ssd += (val_left - val_right) * (val_left - val_right);
            }
        }


        //TODO: sum up normalized sum of squared difference

        //for (int u = -half_window_size; u <= half_window_size; ++u) {
        //    for (int v = -half_window_size; v <= half_window_size; ++v) {
        //        int val_left = image1.at<char>(i + u, j + v);
        //        int val_right = image2.at<char>(i + u, j + v + d);
        //        ssd += (val_left/ norm_left - val_right/ norm_right) * (val_left/ norm_left - val_right/ norm_right);
        //    }
        //}

        if (ssd < min_ssd) {
          min_ssd = ssd;
          disparity = d;
        }
      }

      naive_disparities.at<uchar>(i - half_window_size, j - half_window_size) = std::abs(disparity) * scale;
    }
  }

  std::cout << "Calculating disparities for the naive approach... Done.\r" << std::flush;
  std::cout << std::endl;
}

void Disparity2PointCloud(
  const std::string& output_file,
  int height, int width, cv::Mat& disparities, 
  const int& window_size,
  const int& dmin, const double& baseline, const double& focal_length)
{
    const int cx = height/2;
    const int cy = width/2 ;

  std::stringstream out3d;
  out3d << output_file << ".xyz";
  std::ofstream outfile(out3d.str());
  for (int i = 0; i < height - window_size; ++i) {
    std::cout << "Reconstructing 3D point cloud from disparities... " << std::ceil(((i) / static_cast<double>(height - window_size + 1)) * 100) << "%\r" << std::flush;
    for (int j = 0; j < width - window_size; ++j) {
      if (disparities.at<uchar>(i, j) <= 0  ) continue;

      //Z = fB/d  
      //X = uZ/f
      //Y = vZ/f
      //Image center should be considered
      
      // Must account for the image center
      double x = (i - cx) / focal_length;
      double y = (j - cy) / focal_length;

      const double Z = focal_length * baseline / disparities.at<uchar>(i,j);
      const double X = x * Z;
      const double Y = y * Z;

      outfile << X << " " << Y << " " << Z << std::endl;

    }
  }

  std::cout << "Reconstructing 3D point cloud from disparities... Done.\r" << std::flush;
  std::cout << std::endl;
}


void StereoEstimation_Dynamic(
    const int& window_size,
    int height,
    int width,
    cv::Mat& image1, cv::Mat& image2,
    const double& weight,
    const std::string& output_file
    ) {

    std::cout << "Calculating disparities for the dynamic programming approach..." << std::endl;
    int half_window_size = window_size / 2;
    // initializsing disparity image
    cv::Mat disparities = cv::Mat::zeros(height - window_size, width - window_size, CV_8UC1);

    for (int row = half_window_size; row < height - half_window_size; ++row) {

        std::cout << "Calculating disparities for the dynamic programming approach... " << std::ceil(((row - half_window_size + 1) / (float)(height - window_size + 1)) * 100) << "%\r" << std::flush;
        // Create dissimilarity image for the row i
        cv::Mat dissimilarity = cv::Mat::zeros(width - window_size, width - window_size, CV_32F);

        // For each pixel in the row, calculate the dissimilarity in the same row of the other image
        for (int j = half_window_size; j < width - half_window_size - 1; ++j) {


            // match pixel (row,j) with (row, j_right) in the second image
            for (int j_right = half_window_size; j_right < width - half_window_size - 1; ++j_right) {
                int dissim = 0;
                // make a window around the pixels in both images
                for (int u = -half_window_size; u <= half_window_size; ++u) {
                    for (int v = -half_window_size; v <= half_window_size; ++v) {

                        int u1 = row + u;
                        int v1 = j + v;

                        int u2 = row + u;
                        int v2 = j_right + v;

                        // Calculate dissimilarity using SSD
                        dissim += (image1.at<uchar>(u1, v1) - image2.at<uchar>(u2, v2)) * (image1.at<uchar>(u1, v1) - image2.at<uchar>(u2, v2));

                    }
                }

                dissimilarity.at<float>(j - half_window_size, j_right - half_window_size) = dissim;
            }
        }

        cv::Mat C = cv::Mat::zeros(width - window_size, width - window_size, CV_32F);

        // 0 : match, 1 : left occlusion, 2 : right occlusion
        cv::Mat M = cv::Mat::zeros(width - window_size, width - window_size, CV_8UC1);

        for (int j = 1; j < width - window_size; ++j) {
            C.at<float>(0, j) = C.at<float>(0, j - 1) + weight;
            M.at<uchar>(0, j) = 2;
        }

        for (int i = 1; i < width - window_size; ++i) {
            C.at<float>(i, 0) = C.at<float>(i - 1, 0) + weight;
            M.at<uchar>(i, 0) = 1;
        }

        for (int i = 1; i < width - window_size; ++i) {
            for (int j = 1; j < width - window_size; ++j) {
                float match = C.at<float>(i - 1, j - 1) + dissimilarity.at<float>(j, i);
                float left_occ = C.at<float>(i - 1, j) + weight;
                float right_occ = C.at<float>(i, j - 1) + weight;

                if (match < left_occ && match < right_occ) {
                    C.at<float>(i, j) = match;
                    M.at<uchar>(i, j) = 0;
                }
                else if (left_occ < match && left_occ < right_occ) {
                    C.at<float>(i, j) = left_occ;
                    M.at<uchar>(i, j) = 1;
                }
                else {
                    C.at<float>(i, j) = right_occ;
                    M.at<uchar>(i, j) = 2;
                }

            }
        }
        int i = width - window_size - 1;
        int j = width - window_size - 1;
        while (i != 0 && j != 0) {
            if (M.at<uchar>(i, j) == 0) {
                // match
                disparities.at<uchar>(row - window_size / 2, j) = (j - i) * 3;
                i--;
                j--;
            }
            else if (M.at<uchar>(i, j) == 1) {
                disparities.at<uchar>(row - window_size / 2, j) = 0;
                i--;
            }
            else {
                j--;
            }
        }
        // filling the occlusions
        for (int j = 0; j < width - window_size; ++j) {
            if (disparities.at<uchar>(row - window_size / 2, j) == 0) {
                i = j;
                while (i >= 0 && disparities.at<uchar>(row - window_size / 2, i) == 0) {
                    --i;
                }

                if (i >= 0) {
                    disparities.at<uchar>(row - window_size / 2, j) = disparities.at<uchar>(row - window_size / 2, i);
                }

            }
        }

    }
    // save / display images
    std::cout << std::endl;
    std::cout << "Done." << std::endl << std::endl;

    std::stringstream out2;
    out2 << output_file << "_dynamic.png";
    cv::imwrite(out2.str(), disparities);

    cv::namedWindow("Dynamic", cv::WINDOW_AUTOSIZE);
    cv::imshow("Dynamic", disparities);
  
}



