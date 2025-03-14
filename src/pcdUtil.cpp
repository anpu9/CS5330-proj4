/*
 * Authors: Yuyang Tian and Arun Mekkad
 * Date: 2025/3/11
 * Purpose: utility functions for point cloud generation and visualization
 */
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <thread>


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
bool loadCameraCalibration(const string& filename, Mat& cameraMatrix, Mat& distortionCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "Error: Unable to open camera calibration file: " << filename << endl;
        return false;
    }
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distortionCoeffs;
    fs.release();
    return true;
}

/**
 * @brief load a PCD file
 * @param filename pcl filename
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPointCloud(const string& filename) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, *cloud) == -1) {
        cerr << "Error loading PCD file: " << filename << endl;
        return nullptr;
    }
    return cloud;
}

/**
 * @brief Undistorts 2D pixel points using camera calibration parameters.
 *
 * This function corrects the distortion in given pixel coordinates (e.g., from an image)
 * using the intrinsic camera matrix and distortion coefficients. It maps distorted
 * pixel coordinates to their corrected positions in the rectified image space.
 *
 * @param distorted_points A `1×2` Mat containing the distorted pixel coordinates (u, v).
 * @param camera_matrix The `3×3` intrinsic camera matrix.
 * @param dist_coeffs The distortion coefficients vector (`1×5` or `1×8`).
 * @return A `1×2` Mat containing the undistorted pixel coordinates (u', v').
 */
cv::Mat undistortPoints(const cv::Mat& distorted_points, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) {
    cv::Mat undistorted_points;
    cv::undistortPoints(distorted_points, undistorted_points, camera_matrix, dist_coeffs, cv::noArray(), camera_matrix);
    return undistorted_points;
}

/**
 * @brief Back-projects a 2D pixel point to a 3D world coordinate.
 *
 * This function converts a given 2D pixel coordinate (u, v) into real-world 3D coordinates (X, Y, Z),
 * using the camera intrinsic matrix and depth information. The computed 3D points are scaled
 * according to the given `square_size` to maintain correct real-world proportions.
 *
 * @param pixel_points A 1×2 Mat containing the 2D pixel coordinates (u, v).
 * @param camera_matrix The intrinsic camera matrix (3×3).
 * @param depth The depth value at the given pixel, representing the distance from the camera.
 * @param square_size The real-world size of a calibration square in meters (default: 0.09m for 9 cm).
 * @return A 3×1 Mat containing the back-projected 3D coordinates (X, Y, Z) in meters.
 */
cv::Mat backProjectTo3D(const cv::Mat& pixel_points,
                        const cv::Mat& camera_matrix,
                        float depth,
                        float square_size = 0.09f) {
    double fx = camera_matrix.at<double>(0, 0);  // Focal length in pixels
    double fy = camera_matrix.at<double>(1, 1);
    double cx = camera_matrix.at<double>(0, 2);  // Principal point
    double cy = camera_matrix.at<double>(1, 2);

    // Convert depth to meters if needed (assuming depth is given in square_size units)
    double scaled_depth = depth * square_size;

    // Convert pixels to meters
    double X = ((pixel_points.at<double>(0,0) - cx) * scaled_depth) / fx;
    double Y = ((pixel_points.at<double>(0,1) - cy) * scaled_depth) / fy;
    double Z = scaled_depth;

    return (cv::Mat_<double>(3,1) << X, Y, Z);
}

/**
 * @brief Processes a video frame and generates a point cloud dynamically.
 *
 * @param rgbFrame The current RGB frame.
 * @param depthFrame The corresponding depth frame.
 * @param cameraMatrix The camera intrinsic matrix.
 * @return pcl::PointCloud<pcl::PointXYZRGB>::Ptr The generated point cloud.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr processPointCloud(
        const cv::Mat& src,
        const cv::Mat& dst,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs)
{
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    for (int i = 0; i < src.rows; i++) {
        for (int j = 0; j < src.cols; j++) {
            float depth = dst.at<float>(i, j);
            if (depth <= 0) continue;  // Ignore invalid depths

            cv::Mat distorted_pt = (cv::Mat_<double>(1, 2) << j, i);
            cv::Mat undistorted_pt = undistortPoints(distorted_pt, camera_matrix, dist_coeffs);
            cv::Mat point_3D = backProjectTo3D(undistorted_pt, camera_matrix, depth);

            pcl::PointXYZRGB pcl_point;
            pcl_point.x = point_3D.at<double>(0, 0);
            pcl_point.y = point_3D.at<double>(1, 0);
            pcl_point.z = point_3D.at<double>(2, 0);
            pcl_point.r = src.at<cv::Vec3b>(i, j)[2];
            pcl_point.g = src.at<cv::Vec3b>(i, j)[1];
            pcl_point.b = src.at<cv::Vec3b>(i, j)[0];

            cloud->points.push_back(pcl_point);
        }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    return cloud;
}


/**
 * @brief GUI window to display the 3D point cloud
 * @param cloud The point cloud.
 */
void visualizePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    // Normalize and fix color values for each point
    for (auto& point : cloud->points) {
        // Unpack the RGB values
        uint32_t rgb = *reinterpret_cast<int*>(&point.rgb);
        uint8_t r = (rgb >> 16) & 0x0000ff;
        uint8_t g = (rgb >> 8)  & 0x0000ff;
        uint8_t b = (rgb)       & 0x0000ff;

        // Repack properly
        uint32_t color = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        point.rgb = *reinterpret_cast<float*>(&color);
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    // Add the point cloud with RGB color handling
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");

    // Increase point size for better visibility
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");

    // Add coordinate system with smaller size
    viewer->addCoordinateSystem(0.1); // Reduced size to 0.1 from 1.0

    viewer->initCameraParameters();

    // Set a better initial viewpoint
    viewer->setCameraPosition(-1, -1, -1,    // camera position
                              0,  0,  0,    // viewpoint
                              0,  0,  1);   // up vector

    // Enable point picking (optional, for debugging)
    viewer->registerPointPickingCallback([](const pcl::visualization::PointPickingEvent& event) {
        float x, y, z;
        event.getPoint(x, y, z);
        std::cout << "Picked point: " << x << " " << y << " " << z << std::endl;
    });

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

}
/**
 * @brief GUI window to display the 3D point cloud
 * @param filename The pcd file.
 */
void visualizePointCloudByFile(const string& filename) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, *cloud) == -1) {
        PCL_ERROR("Error: Couldn't read the PCD file %s\n", filename.c_str());
    }
    visualizePointCloud(cloud);
}