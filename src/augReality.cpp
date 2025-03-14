/*
 * Authors: Yuyang Tian and Arun Mekkad
 * Date: 2025/2/28
 * Purpose: Augmented reality with ArUco marker detection and 3D object rendering
 */

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/objdetect.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "../include/pclUtil.h"
#include <iostream>

using namespace cv;
using namespace std;

// Constants
const float MARKER_LENGTH = 0.05f;  // Marker size in meters (5 cm), set this approximately as the length of aruco marker side in real life
const int EDGE_THICKNESS = 3;
const float DEPTH_OFFSET = 0.8;
const String FACE_CASCADE_PATH = "../object/haarcascade_frontalface_alt2.xml"; // Path to the Haar Cascade XML file for face detection

// Toggle options for displaying projected points and axes
bool showPoints = false;
bool showAxes = false;
bool showTetrahedron = false;
bool showFaceHat = false;

vector<Rect> recentFaces;
const int numFramesToAverage = 7; // Adjust this value as needed


// Function to render 3D axes
void draw3DAxes(Mat& frame, const Vec3d& rvec, const Vec3d& tvec, const Mat& cameraMatrix, const Mat& distCoeffs) {
    vector<Point3f> axisPoints = {
            {0, 0, 0}, {MARKER_LENGTH, 0, 0}, {0, MARKER_LENGTH, 0}, {0, 0, MARKER_LENGTH}
    };

    vector<Point2f> projectedAxes;
    projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedAxes);

    // Draw axes in different colors
    line(frame, projectedAxes[0], projectedAxes[1], Scalar(0, 0, 255), 2); // X-axis (Red)
    line(frame, projectedAxes[0], projectedAxes[2], Scalar(0, 255, 0), 2); // Y-axis (Green)
    line(frame, projectedAxes[0], projectedAxes[3], Scalar(255, 0, 0), 2); // Z-axis (Blue)
}

// Function to render 3D tetrahedron
void drawTetrahedron(Mat& frame, const Vec3d& rvec, const Vec3d& tvec, const Mat& cameraMatrix, const Mat& distCoeffs) {
    vector<Point3f> tetrahedronPoints = {
            {-0.4f * MARKER_LENGTH, -0.4f * MARKER_LENGTH, 0},   // Base vertex 1
            {0.6f * MARKER_LENGTH, -0.4f * MARKER_LENGTH, 0},    // Base vertex 2
            {0.2f * MARKER_LENGTH, 0.4f * MARKER_LENGTH, 0},     // Base vertex 3
            {0.3f * MARKER_LENGTH, 0.2f * MARKER_LENGTH, MARKER_LENGTH}  // Apex vertex
    };

    vector<Point2f> projectedPoints;
    projectPoints(tetrahedronPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);

    // Draw base triangle edges
    line(frame, projectedPoints[0], projectedPoints[1], Scalar(255, 0, 0), EDGE_THICKNESS); // Red
    line(frame, projectedPoints[1], projectedPoints[2], Scalar(0, 255, 0), EDGE_THICKNESS); // Green
    line(frame, projectedPoints[2], projectedPoints[0], Scalar(0, 0, 255), EDGE_THICKNESS); // Blue

    // Draw edges to the apex
    line(frame, projectedPoints[0], projectedPoints[3], Scalar(255, 255, 0), EDGE_THICKNESS); // Cyan
    line(frame, projectedPoints[1], projectedPoints[3], Scalar(0, 255, 255), EDGE_THICKNESS); // Yellow
    line(frame, projectedPoints[2], projectedPoints[3], Scalar(255, 0, 255), EDGE_THICKNESS); // Magenta
}

// Function to project object 3D points onto the video frame
void drawPointCloudOnFrame(Mat &frame, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           const Vec3d &rvec, const Vec3d &tvec,
                           const Mat &cameraMatrix, const Mat &distCoeffs) {
    vector<Point3f> objectPoints;
    vector<Point2f> imagePoints;

    // Fix rotation misalignment by applying correction
    Mat rotationMatrix;
    Rodrigues(rvec, rotationMatrix);
    double angleX = 80 * CV_PI / 180.0;
    double angleY = 0 * CV_PI / 180.0;
    double angleZ = 10 * CV_PI / 180.0;
    Mat Rx = (Mat_<double>(3, 3) <<
                                 1, 0, 0,
            0, cos(angleX), -sin(angleX),
            0, sin(angleX), cos(angleX)
    );
    Mat Ry = (Mat_<double>(3, 3) <<
                                 cos(angleY), 0, sin(angleY),
            0, 1, 0,
            -sin(angleY), 0, cos(angleY)
    );
    Mat Rz = (Mat_<double>(3, 3) <<
                                 cos(angleZ), -sin(angleZ), 0,
            sin(angleZ), cos(angleZ), 0,
            0, 0, 1
    );
    Mat correctionMatrix = Rz * Ry * Rx;  // Apply X, Y, Z rotation
    Rodrigues(rvec, rotationMatrix);
    rotationMatrix = rotationMatrix * correctionMatrix;  // Apply combined rotation


//    // Adjust translation (move object closer to the marker)
    Vec3d adjustedTvec = tvec;
    adjustedTvec[0] += 0.1;
    adjustedTvec[1] -= 0.1;
    adjustedTvec[2] -= 1.5;

    // Convert point cloud into objectPoints with corrected depth
    for (const auto &point: cloud->points) {
        objectPoints.push_back(Point3f(point.x - 0.2 , point.y - 0.2, point.z -0.2));
    }

    if (!objectPoints.empty()) {
        projectPoints(objectPoints, rotationMatrix, adjustedTvec, cameraMatrix, distCoeffs, imagePoints);

        // Draw projected 3D points on the video frame
        for (size_t i = 0; i < imagePoints.size(); ++i) {
//            imagePoints[i].x += 10;
            imagePoints[i].y -= 55;
            circle(frame, imagePoints[i], 1, Scalar(cloud->points[i].r, cloud->points[i].g, cloud->points[i].b), -1);
        }
    }
}

void drawFaceHat(Mat& frame, const Rect& face, const Mat& cameraMatrix, const Mat& distCoeffs) {
    float faceWidth = face.width;
    float faceHeight = face.height;
    float hatBaseRadius = 0.8 * faceWidth;
    float hatHeight = faceHeight;
    float hatZOffset = -0.7 * faceWidth; // Move the hat away from the camera

    vector<Point3f> faceObjectPoints = {
            {-0.5f * faceWidth, -0.5f * faceHeight, 0},
            { 0.5f * faceWidth, -0.5f * faceHeight, 0},
            { 0.5f * faceWidth,  0.5f * faceHeight, 0},
            {-0.5f * faceWidth,  0.5f * faceHeight, 0}
    };

    vector<Point2f> faceImagePoints = {
            Point2f(face.x, face.y),
            Point2f(face.x + faceWidth, face.y),
            Point2f(face.x + faceWidth, face.y + faceHeight),
            Point2f(face.x, face.y + faceHeight)
    };

    Mat rvec, tvec;
    solvePnP(faceObjectPoints, faceImagePoints, cameraMatrix, distCoeffs, rvec, tvec);

    vector<Point3f> hatPoints3D;
    int numPoints = 30;
    // Base circle points
    float baseCenterY = 0.5f * faceHeight;
    float hatBaseYInFaceFrame = 0.2f * faceHeight;
    for (int i = 0; i < numPoints; ++i) {
        float angle = 2.0f * CV_PI * i / numPoints;
        hatPoints3D.push_back(Point3f(hatBaseRadius * cos(angle), -(hatBaseYInFaceFrame + 0.3 * hatHeight), hatBaseRadius * sin(angle) + hatZOffset)); // Adjusted Y and added Z
    }
    // Apex point
    hatPoints3D.push_back(Point3f(0, -(hatBaseYInFaceFrame + hatHeight + 0.2 * hatHeight), hatZOffset));

    vector<Point2f> projectedHatPoints;
    projectPoints(hatPoints3D, rvec, tvec, cameraMatrix, distCoeffs, projectedHatPoints);

    Scalar hatColor = Scalar(0, 0, 255);
    int thickness = 2;

    // Draw the base circle
    for (int i = 0; i < numPoints - 1; ++i) {
        line(frame, projectedHatPoints[i], projectedHatPoints[i + 1], hatColor, thickness);
    }
    line(frame, projectedHatPoints[numPoints - 1], projectedHatPoints[0], hatColor, thickness);

    // Draw lines from the base to the apex
    Point2f apexPoint = projectedHatPoints[numPoints];
    for (int i = 0; i < numPoints; ++i) {
        if(i % 2 == 0) // Draw every second line to reduce clutter
            hatColor = Scalar(0, 255, 0); // Change color for every second line
        else
            hatColor = Scalar(255, 0, 0);
        line(frame, projectedHatPoints[i], apexPoint, hatColor, thickness);
    }

    rectangle(frame, face, Scalar(255, 0, 255), 2);
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        cerr << "Usage: ./augReality <path to camera_params.yml> <path to pcd file>" << endl;
        return -1;
    }
    string filename = argv[1];
    string pcdfile = argv[2];

    Mat camera_matrix, distortion_coeffs;
    // Load camera calibration
    if (!loadCameraCalibration(filename, camera_matrix, distortion_coeffs)) {
        cerr << "Failed to load camera calibration! " << endl;
        return -1;
    }

    // Open video capture
    VideoCapture cap(0); // Use 0 for default camera
    if (!cap.isOpened()) {
        cerr << "Failed to open camera." << endl;
        return -1;
    }

    // Initialize ArUco detector
    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_5X5_1000);
    aruco::DetectorParameters detectParams;
    aruco::ArucoDetector detector(dictionary, detectParams);

    // Face detection setup
    CascadeClassifier faceCascade;
    if (!faceCascade.load(FACE_CASCADE_PATH)) {
        cerr << "Error loading face cascade" << endl;
        return -1;
    }

    // Initialize the point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = loadPointCloud(pcdfile);
    if (!cloud) {
        cerr << "Failed to load point cloud." << endl;
        return -1;
    }

    // Toggle options for displaying projected points and axes
    bool showPoints = false;
    bool showAxes = false;
    bool showTetrahedron = false;
    bool showPointCloud = false;
    while (true) {
        Mat frame;
        cap >> frame;
        if (frame.empty()) {
            cerr << "ERROR: The frame is empty!" << endl;
            break;
        }

        // Detect ArUco markers in the frame
        vector<int> markerIds;
        vector<vector<Point2f>> markerCorners;
        detector.detectMarkers(frame, markerCorners, markerIds);

        if (!markerIds.empty() && !showFaceHat) {
            // Draw detected markers
            aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

            vector<Vec3d> rvecs(markerIds.size()), tvecs(markerIds.size());

            // Define 3D coordinates of marker corners
            vector<Point3f> objectPoints = {
                    {-MARKER_LENGTH / 2, MARKER_LENGTH / 2,  0},
                    {MARKER_LENGTH / 2,  MARKER_LENGTH / 2,  0},
                    {MARKER_LENGTH / 2,  -MARKER_LENGTH / 2, 0},
                    {-MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0}
            };

            for (size_t i = 0; i < markerIds.size(); i++) {
                // Estimate camera pose relative to marker
                solvePnP(objectPoints, markerCorners[i], camera_matrix, distortion_coeffs, rvecs[i], tvecs[i]);

                // Toggle projected marker points
                if (showPoints) {
                    vector<Point2f> projectedPoints;
                    projectPoints(objectPoints, rvecs[i], tvecs[i], camera_matrix, distortion_coeffs, projectedPoints);
                    for (size_t j = 0; j < projectedPoints.size(); j++) {
                        circle(frame, projectedPoints[j], 5, Scalar(0, 0, 255), -1); // Red circles
                    }
                }

                // Toggle 3D axis visualization
                if (showAxes) {
                    // Define 3D axis points
                    draw3DAxes(frame, rvecs[i], tvecs[i], camera_matrix, distortion_coeffs);
                }

                // Render 3D object if 'o' key is pressed
                if (showTetrahedron) {
                    drawTetrahedron(frame, rvecs[i], tvecs[i], camera_matrix, distortion_coeffs);
                }

                if (showPointCloud) {
                    drawPointCloudOnFrame(frame, cloud, rvecs[i], tvecs[i], camera_matrix, distortion_coeffs);
                }
            }
        }

        if (showFaceHat) {
            Mat gray;
            cvtColor(frame, gray, COLOR_BGR2GRAY);
            equalizeHist(gray, gray);

            vector<Rect> faces;
            faceCascade.detectMultiScale(gray, faces, 1.1, 3, 0, Size(100, 100));

            if (!faces.empty()) {
                Rect currentFace = faces[0]; // Consider the first detected face

                // Simple averaging of face rectangles
                recentFaces.push_back(currentFace);
                if (recentFaces.size() > numFramesToAverage) {
                    recentFaces.erase(recentFaces.begin());
                }

                if (!recentFaces.empty()) {
                    int avgX = 0, avgY = 0, avgWidth = 0, avgHeight = 0;
                    for (const auto& rect : recentFaces) {
                        avgX += rect.x;
                        avgY += rect.y;
                        avgWidth += rect.width;
                        avgHeight += rect.height;
                    }
                    Rect averagedFace(avgX / recentFaces.size(), avgY / recentFaces.size(),
                                      avgWidth / recentFaces.size(), avgHeight / recentFaces.size());
                    drawFaceHat(frame, averagedFace, camera_matrix, distortion_coeffs);
                } else {
                    drawFaceHat(frame, currentFace, camera_matrix, distortion_coeffs); // Fallback if no recent faces
                }
            } else {
                recentFaces.clear(); // Clear if no face is detected
            }
        }
        imshow("Augmented Reality", frame);
        
        // Handle keypress events
        char key = waitKey(30);
        if (key == 'q') break;   // Quit the program
        else if (key == 'p') showPoints = !showPoints; // Toggle marker corner points
        else if (key == 'a') showAxes = !showAxes; // Toggle 3D axes
        else if (key == 't') showTetrahedron = !showTetrahedron; // Toggle 3D axes
        else if (key == 'f') showFaceHat = !showFaceHat; // Toggle face cube
        else if (key == 'c') showPointCloud = !showPointCloud; // Toggle 3D axes
    }

    cap.release();
    destroyAllWindows();
    return 0;
}