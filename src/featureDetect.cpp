/*
 * Authors: Arun Mekkad and Yuyang Tian
 * Date: 2025/2/28
 * Purpose: Real-time SIFT feature detection with adjustable number of features
 */

 #include <opencv2/opencv.hpp>
 #include <opencv2/features2d.hpp>
 #include <iostream>
 
 using namespace cv;
 using namespace std;
 
 // Global variables
 int nFeatures = 100;          // Trackbar value (100 as initial value)
 Ptr<SIFT> sift;
 VideoCapture cap;
 
 // Callback function to update SIFT detector when trackbar changes
 void onNFeaturesChange(int pos, void*) {
     nFeatures = pos; // Update the global variable
     sift = SIFT::create(nFeatures);
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
 
         // Draw keypoints on the frame
         Mat output;
         drawKeypoints(frame, keypoints, output, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
 
         // Display the result
         imshow("SIFT Features", output);
 
         // Exit loop on 'q' key press
         if (waitKey(1) == 'q') break; // Reduced waitKey time for smoother video
     }
 }
 
 int main() {
     cap.open(2); // Open specified camera
     if (!cap.isOpened()) {
         cerr << "Error: Unable to open the camera." << endl;
         return -1;
     }
 
     // Create a window and trackbar
     namedWindow("SIFT Features", WINDOW_AUTOSIZE);
     createTrackbar("Number of Features", "SIFT Features", NULL, 2000, onNFeaturesChange); // Pass NULL as value pointer
     setTrackbarPos("Number of Features", "SIFT Features", nFeatures); //set the trackbar to the starting value
 
     // Initialize SIFT with the default number of features
     onNFeaturesChange(nFeatures, nullptr); // Call the callback to initialize
 
     detectSIFTFeatures();
 
     cap.release();
     destroyAllWindows();
     return 0;
 }