/*
 * Authors: Yuyang Tian and Arun Mekkad
 * Date: 2025/2/16
 * Purpose: utility functions for point cloud generation and visualization
 */

#ifndef PROJ4_PCLUTIL_H
#define PROJ4_PCLUTIL_H

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <thread>
#include <chrono>

using namespace cv;
using namespace std;

/**
 * @brief Loads camera calibration parameters from a YAML file.
 *
 * @param filename Path to the YAML calibration file.
 * @param cameraMatrix Output: Camera intrinsic matrix.
 * @param distortionCoeffs Output: Distortion coefficients.
 * @return true if loaded successfully, false otherwise.
 */
bool loadCameraCalibration(const string& filename, Mat& cameraMatrix, Mat& distortionCoeffs);

/**
 * @brief Generates a 3D point cloud from an RGB and depth frame.
 *
 * @param rgbFrame The RGB frame from the video stream.
 * @param depthFrame The depth frame from the video stream.
 * @param cameraMatrix The intrinsic camera matrix.
 * @param cloud Output: PCL point cloud.
 */
void generatePointCloud(const Mat& rgbFrame, const Mat& depthFrame,
                        const Mat& cameraMatrix, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
/**
 * @brief Processes a video frame and generates a point cloud dynamically.
 *
 * @param rgbFrame The current RGB frame.
 * @param depthFrame The corresponding depth frame.
 * @param cameraMatrix The camera intrinsic matrix.
 * @param cameraMatrix The camera intrinsic matrix.
 * @return pcl::PointCloud<pcl::PointXYZRGB>::Ptr The generated point cloud.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr processPointCloud(const Mat& rgbFrame, const Mat& depthFrame, const Mat& cameraMatrix, const Mat& distCoeffs);
/**
 * @brief Display a 3D point cloud
 * @param cloud The point cloud.
 */
void visualizePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
/**
 * @brief Display a 3D point cloud
 * @param filename pcl filename
 */
void visualizePointCloudByFile(const string& filename);
/**
 * @brief load a PCD file
 * @param filename pcl filename
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPointCloud(const string& filename);
#endif //PROJ4_PCLUTIL_H
