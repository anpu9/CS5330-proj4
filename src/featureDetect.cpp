#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// Global variables
int contrastThreshold = 40; // Trackbar value (scaled by 100)
Ptr<SIFT> sift;
VideoCapture cap;

// Function to update SIFT detector when trackbar changes
void onThresholdChange(int, void*) {
    sift = SIFT::create(contrastThreshold / 100.0); // Convert to float
}

// Function to detect SIFT features in real-time video
void detectSIFTFeatures() {
    Mat frame, gray;
    
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cvtColor(frame, gray, COLOR_BGR2GRAY);
        
        // Detect SIFT keypoints
        vector<KeyPoint> keypoints;
        Mat descriptors;
        sift->detectAndCompute(gray, noArray(), keypoints, descriptors);

        // Filter keypoints by response strength
        vector<KeyPoint> filteredKeypoints;
        for (const auto& kp : keypoints) {
            if (kp.response > 0.05) {
                filteredKeypoints.push_back(kp);
            }
        }

        // Draw keypoints on the frame
        Mat output;
        drawKeypoints(frame, filteredKeypoints, output, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        // Display the result
        imshow("SIFT Features", output);

        // Exit loop on 'q' key press
        if (waitKey(30) == 'q') break;
    }
}

int main() {
    cap.open(0); // Open default camera
    if (!cap.isOpened()) {
        cerr << "Error: Unable to open the camera." << endl;
        return -1;
    }

    // Create a window and trackbar
    namedWindow("SIFT Features", WINDOW_AUTOSIZE);
    createTrackbar("Threshold", "SIFT Features", &contrastThreshold, 100, onThresholdChange);

    // Initialize SIFT with the default threshold
    onThresholdChange(contrastThreshold, nullptr);

    detectSIFTFeatures();

    cap.release();
    destroyAllWindows();
    return 0;
}
