#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// Constants
const float MARKER_LENGTH = 0.05;  // Marker size in meters (5 cm), set this approximately as the length of aruco marker side in real life
const int EDGE_THICKNESS = 3;

// Function to load camera calibration parameters
bool loadCameraCalibration(const string& filename, Mat& cameraMatrix, Mat& distortionCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "Failed to open camera parameters file: " << filename << endl;
        return false;
    }
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distortionCoeffs;
    fs.release();
    return true;
}

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

int main(int argc, char* argv[]) {
    if (argc < 2) {
        cerr << "Usage: ./augReality <path to camera_params.yml>" << endl;
        return -1;
    }
    
    string filename = argv[1];
    
    Mat camera_matrix, distortion_coeffs;
    // Load camera calibration
    if (!loadCameraCalibration(filename, camera_matrix, distortion_coeffs)) return -1;

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

    // Toggle options for displaying projected points and axes
    bool showPoints = false;
    bool showAxes = false;
    bool showTetrahedron = false;

    while (true) {
        Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        // Detect ArUco markers in the frame
        vector<int> markerIds;
        vector<vector<Point2f>> markerCorners;
        detector.detectMarkers(frame, markerCorners, markerIds);

        if (!markerIds.empty()) {
            // Draw detected markers
            aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
            
            vector<Vec3d> rvecs(markerIds.size()), tvecs(markerIds.size());
            
            // Define 3D coordinates of marker corners
            vector<Point3f> objectPoints = {
                {-MARKER_LENGTH/2, MARKER_LENGTH/2, 0},
                {MARKER_LENGTH/2, MARKER_LENGTH/2, 0},
                {MARKER_LENGTH/2, -MARKER_LENGTH/2, 0},
                {-MARKER_LENGTH/2, -MARKER_LENGTH/2, 0}
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
            }
        }
        
        imshow("Camera Pose Estimation", frame);
        
        // Handle keypress events
        char key = waitKey(30);
        if (key == 'q') break;   // Quit the program
        else if (key == 'p') showPoints = !showPoints; // Toggle marker corner points
        else if (key == 'a') showAxes = !showAxes; // Toggle 3D axes
        else if (key == 't') showTetrahedron = !showTetrahedron; // Toggle 3D axes
    }
    
 
    cap.release();
    destroyAllWindows();
    return 0;
}