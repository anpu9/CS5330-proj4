#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char* argv[]) {
    if (argc < 2) {
        cerr << "Usage: ./augReality <path to camera_params.yml>" << endl;
        return -1;
    }
    
    string filename = argv[1];
    
    // Load camera calibration parameters
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "Failed to open camera parameters file: " << filename << endl;
        return -1;
    }
    
    Mat camera_matrix, distortion_coeffs;
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> distortion_coeffs;
    fs.release();

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
            float markerLength = 0.05; // Marker size in meters (5 cm), set this approximately as the length of aruco marker side in real life
            
            // Define 3D coordinates of marker corners
            vector<Point3f> objectPoints = {
                {-markerLength/2, markerLength/2, 0},
                {markerLength/2, markerLength/2, 0},
                {markerLength/2, -markerLength/2, 0},
                {-markerLength/2, -markerLength/2, 0}
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
                    vector<Point3f> axisPoints = {
                        {0, 0, 0},             // Origin (center of marker)
                        {markerLength, 0, 0},  // X-axis direction
                        {0, markerLength, 0},  // Y-axis direction
                        {0, 0, markerLength}  // Z-axis direction (outward)
                    };
                    
                    // Project axes onto 2D image plane
                    vector<Point2f> projectedAxes;
                    projectPoints(axisPoints, rvecs[i], tvecs[i], camera_matrix, distortion_coeffs, projectedAxes);
                    
                    // Draw 3D coordinate axes
                    line(frame, projectedAxes[0], projectedAxes[1], Scalar(0, 0, 255), 2); // X-axis (Red)
                    line(frame, projectedAxes[0], projectedAxes[2], Scalar(0, 255, 0), 2); // Y-axis (Green)
                    line(frame, projectedAxes[0], projectedAxes[3], Scalar(255, 0, 0), 2); // Z-axis (Blue)
                }
            }
        }
        
        imshow("Camera Pose Estimation", frame);
        
        // Handle keypress events
        char key = waitKey(30);
        if (key == 'q') break;   // Quit the program
        else if (key == 'p') showPoints = !showPoints; // Toggle marker corner points
        else if (key == 'a') showAxes = !showAxes; // Toggle 3D axes
    }
    
 
    cap.release();
    destroyAllWindows();
    return 0;
}