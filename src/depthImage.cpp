/*
  Bruce A. Maxwell
  January 2025

  Modified by Yuyang Tian and Arun Mekkad.
  User type 's' to show depth map generate 3D point cloud of an image
*/

#include <cstdio>
#include <opencv2/opencv.hpp>
#include "../include/DA2Network.hpp"
#include "../include/pclUtil.h"

string obj = "matcha";
const std::string OUTPUT_DIR = "../outputs/", extn = ".pcd";

std::string generateFilename() {
    return OUTPUT_DIR + obj + extn;
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: ./depth <path to camera_params.yml> <path to image>" << std::endl;
        return -1;
    }

    std::string cameraFile = argv[1];
    std::string imagePath = argv[2];

    cv::Mat camera_matrix, distortion_coeffs;
    // Load camera calibration
    if (!loadCameraCalibration(cameraFile, camera_matrix, distortion_coeffs)) {
        std::cerr << "Failed to load camera calibration" << std::endl;
        return -1;
    }

    // Load image
    cv::Mat src = cv::imread(imagePath);
    if (src.empty()) {
        std::cerr << "Error: Could not load image " << imagePath << std::endl;
        return -1;
    }

    cv::Mat dst, pclFrame, dst_vis;
    const float reduction = 1;
    const char *NETWORK_PATH = "../include/model_fp16.onnx";

    // Make a DANetwork object
    DA2Network da_net(NETWORK_PATH);

    float scale_factor = 256.0 / (src.rows * reduction);
    printf("Using scale factor %.2f\n", scale_factor);

    cv::resize(src, src, cv::Size(), reduction, reduction);

    // Set the network input
    da_net.set_input(src, scale_factor);

    // Run the network
    da_net.run_network(dst, src.size());
    da_net.run_network_pcl(pclFrame, src.size());

    // Apply a color map to the depth output to get a good visualization
    cv::applyColorMap(dst, dst_vis, cv::COLORMAP_INFERNO);

    // Display the images
    cv::imshow("Image", src);
    cv::imshow("Depth", dst_vis);

    // Wait for user input
    int key = cv::waitKey(0);
    if (key == 's') {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = processPointCloud(src, pclFrame, camera_matrix, distortion_coeffs);
        // Save the point cloud to a PCD file
        std::string outputFilename = generateFilename();
        if (pcl::io::savePCDFileASCII(outputFilename, *cloud) == 0) {
            std::cout << "Saved " << cloud->points.size() << " points to " << outputFilename << std::endl;
        } else {
            std::cerr << "Error: Could not save PCD file." << std::endl;
            return -1;
        }
    }

    printf("Terminating\n");
    return 0;
}
