#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>

using namespace cv;
using namespace std;

void detectSIFTFeatures(VideoCapture& cap) {
    Ptr<SIFT> sift = SIFT::create();
    Mat frame, gray;
    
    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        
        cvtColor(frame, gray, COLOR_BGR2GRAY); // Convert to grayscale
        
        // Detect SIFT keypoints and compute descriptors
        vector<KeyPoint> keypoints;
        Mat descriptors;
        sift->detectAndCompute(gray, noArray(), keypoints, descriptors);
        
        // Draw keypoints on the frame
        Mat output;
        drawKeypoints(frame, keypoints, output, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        
        // Display the result
        imshow("SIFT Features", output);
        
        // Exit loop on 'q' key press
        if (waitKey(30) == 'q') break;
    }
}

int main() {
    VideoCapture cap(0); // Open the default camera
    if (!cap.isOpened()) {
        cerr << "Error: Unable to open the camera." << endl;
        return -1;
    }
    
    detectSIFTFeatures(cap);
    
    cap.release();
    destroyAllWindows();
    return 0;
}