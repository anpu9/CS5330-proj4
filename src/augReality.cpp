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
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Failed to open camera." << endl;
        return -1;
    }

    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_5X5_1000);
    aruco::DetectorParameters detectParams;
    aruco::ArucoDetector detector(dictionary, detectParams);

    while (true) {
        Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        vector<int> markerIds;
        vector<vector<Point2f>> markerCorners;
        detector.detectMarkers(frame, markerCorners, markerIds);

        if (!markerIds.empty()) {
            aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
            
            vector<Vec3d> rvecs(markerIds.size()), tvecs(markerIds.size());
            float markerLength = 0.05; // This should be approximately the length of side of an aruco marker in real-life (taken as 5 cms)
            vector<Point3f> objectPoints = {
                {-markerLength/2, markerLength/2, 0},
                {markerLength/2, markerLength/2, 0},
                {markerLength/2, -markerLength/2, 0},
                {-markerLength/2, -markerLength/2, 0}
            };
            
            for (size_t i = 0; i < markerIds.size(); i++) {
                solvePnP(objectPoints, markerCorners[i], camera_matrix, distortion_coeffs, rvecs[i], tvecs[i]);
                // Draw axis to show camera pose relative to the marker
                drawFrameAxes(frame, camera_matrix, distortion_coeffs, rvecs[i], tvecs[i], markerLength * 0.5);
                cout << "Marker " << markerIds[i] << " Rotation: " << rvecs[i] << " Translation: " << tvecs[i] << endl;
            }
        }
        
        imshow("Camera Pose Estimation", frame);
        if (waitKey(30) == 'q') break;
    }
    
    cap.release();
    destroyAllWindows();
    return 0;
}
