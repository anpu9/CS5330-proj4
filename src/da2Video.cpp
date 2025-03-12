/*
  Bruce A. Maxwell
  January 2025

  Modified by Yuyang Tian.
  User type 's' to show depth map generate 3D point cloud of a image

*/

#include <cstdio>
#include <opencv2/opencv.hpp>
#include "../include/DA2Network.hpp"
#include "../include/pcl3d.h"

int id = 2;
const std::string OUTPUT_DIR = "../outputs/", extn = ".pcd";

string generateFilename() {
    return OUTPUT_DIR + to_string(id) + extn;
}
// opens a video stream and runs it through the depth anything network
// displays both the original video stream and the depth stream
int main(int argc, char *argv[]) {
    if (argc < 2) {
        cerr << "Usage: ./da2PointCloud <path to camera_params.yml>" << endl;
        return -1;
    }

    string cameraFile = argv[1];

    Mat camera_matrix, distortion_coeffs;
    // Load camera calibration
    if (!loadCameraCalibration(cameraFile, camera_matrix, distortion_coeffs)) {
        cerr << "Failed to load camera calibration" << endl;
    }

    cv::VideoCapture *capdev;
    cv::Mat src;
    cv::Mat dst;
    cv::Mat pclFrame;
    cv::Mat dst_vis;
    const float reduction = 1;
    const char *NETWORK_PATH = "../include/model_fp16.onnx";

    // make a DANetwork object
    DA2Network da_net(NETWORK_PATH);

    // open the video device
    capdev = new cv::VideoCapture(0);
    if (!capdev->isOpened()) {
        printf("Unable to open video device\n");
        return (-1);
    }

    cv::Size refS((int) capdev->get(cv::CAP_PROP_FRAME_WIDTH),
                  (int) capdev->get(cv::CAP_PROP_FRAME_HEIGHT));

    printf("Expected size: %d %d\n", refS.width, refS.height);

    float scale_factor = 256.0 / (refS.height * reduction);
    printf("Using scale factor %.2f\n", scale_factor);

    cv::namedWindow("Video", 1);
    cv::namedWindow("Depth", 2);

    for (;;) {
        // capture the next frame
        *capdev >> src;
        if (src.empty()) {
            printf("frame is empty\n");
            break;
        }
        // for speed purposes, reduce the size of the input frame by half
        cv::resize(src, src, cv::Size(), reduction, reduction);

        // set the network input
        da_net.set_input(src, scale_factor);

        // run the network
        da_net.run_network(dst, src.size());
        da_net.run_network_pcl(pclFrame, src.size());

        // apply a color map to the depth output to get a good visualization
        cv::applyColorMap(dst, dst_vis, cv::COLORMAP_INFERNO);

        // display the images
        cv::imshow("video", src);
        cv::imshow("depth", dst_vis);

        // terminate if the user types 'q'
        // Save pcl file if the user types 's'
        int key = cv::waitKey(30);
        if (key == 'q') {
            break;
        } else if (key == 's') {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = processPointCloud(src, pclFrame, camera_matrix, distortion_coeffs);
            // Save the point cloud to a PCD file
            string outputFilename = generateFilename();
            if (pcl::io::savePCDFileASCII(outputFilename, *cloud) == 0) {
                std::cout << "Saved " << cloud->points.size() << " points to " << outputFilename << std::endl;
                id++;
            } else {
                std::cerr << "Error: Could not save PCD file." << std::endl;
                return -1;
            }
        }
    }

    printf("Terminating\n");

    return (0);
}

