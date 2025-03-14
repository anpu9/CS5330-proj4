/*
 * Authors: Yuyang Tian and Arun Mekkad
 * Date: 2025/2/28
 * Purpose: Implementation of camera calibration
 */

 #include <opencv2/opencv.hpp>
 #include <opencv2/aruco.hpp>
 #include <iostream>
 
 using namespace cv;
 using namespace std;
 
 class CameraApp {
 private:
     // Modes for image processing
     enum class Mode {
         NORMAL,
         ARUCO,
     };
     // Aruco detected points;
     vector<int> markerIds;
     vector<vector<Point2f>> corner_set; // the latest 2D corner set for an image, each element is four corners of a Aruco maker
     vector<vector<Point3f>> point_set; // the latest 3D corner set for an image, each element is four corners of a Aruco maker
     vector<vector<vector<Point2f>>> corner_list;  // Stores selected 2D image corners for all images, each element is all image's marker's corner of an images
     vector<vector<vector<Point3f>>> point_list;   // Stores corresponding 3D world points for all images
 
     // Constants for window names
     const string WINDOW_VIDEO = "Live";
     const string WINDOW_ARUCO = "Aruco Markers";
 
     // Member variables
     VideoCapture cap;
     const string OUTPUT_DIR = "../outputs/";
     int imageId = 0;
     Mode currentMode = Mode::ARUCO;
     aruco::ArucoDetector detector;
     float scale_factor;
     Size targetSize;
 
     Mat camera_matrix;
     Mat distortion_coeffs;
     bool calibrated = false;
     double reprojection_error;
 
     struct Images {
         Mat frame;
         Mat arucoFrame;
     } imgs;
 
     string generateFilename(const string &prefix = "", const string &suffix = "", const string &extn = ".png") {
         return OUTPUT_DIR + prefix + to_string(imageId) + suffix + extn;
     }
      /**
       * @brief Saves the current frame and detected corners for calibration.
       * @param cols the cols of markers in the calibration images
       * @param square_size the size of a target square, by default, is in units of target squares, which is 1
       * @param margin_size the size of a margin between a target square and its neighbor, by default, it is 0.2
       */
     void saveCalibrationData(int cols, float square_size = 1.0f, float margin_size = 0.2f) {
         if (corner_set.empty()) {
             cerr << "No valid markers detected. Image not saved for calibration." << endl;
             return;
         }
         vector<vector<Point3f>> framePoints; // Temporary container for 3D current image points
          for (size_t i = 0; i < corner_set.size(); i++) {
              int marker_id = markerIds[i];  // Get marker ID
              int grid_x = marker_id % cols;
              int grid_y = marker_id / cols;
 
              // Apply margins only after the first row and first column
              float offset_x = grid_x * square_size + (grid_x > 0 ? grid_x * margin_size : 0);
              float offset_y = -grid_y * square_size - (grid_y > 0 ? grid_y * margin_size : 0);
 
              vector<Point3f> markerPoints;
              markerPoints.emplace_back(Point3f(offset_x, offset_y, 0));   // Top-left
              markerPoints.emplace_back(Point3f(offset_x + square_size, offset_y, 0));   // Top-right
              markerPoints.emplace_back(Point3f(offset_x + square_size, offset_y - square_size, 0));   // Bottom-right
              markerPoints.emplace_back(Point3f(offset_x, offset_y - square_size, 0));    // Bottom-left
 
              framePoints.emplace_back(markerPoints);
 
              // Print marker ID and positions
              cout << "Marker ID: " << marker_id << " at grid (" << grid_x << ", " << grid_y << ")\n";
              cout << "  Top-left: (" << offset_x << ", " << offset_y << ", 0)\n";
              cout << "  Top-right: (" << offset_x + square_size << ", " << offset_y << ", 0)\n";
              cout << "  Bottom-right: (" << offset_x + square_size << ", " << offset_y - square_size << ", 0)\n";
              cout << "  Bottom-left: (" << offset_x << ", " << offset_y - square_size << ", 0)\n";
          }
 
          // add the 2d frame to corner_list
         corner_list.emplace_back(corner_set);
         // add the 3d frame to point_list
         point_list.emplace_back(framePoints);
         // DEBUG print -- delete when finished test
          // Print `point_set` (latest markers' 3D points)
          cout << "Latest point_set (3D points per marker in current image):\n";
          for (const auto& marker : framePoints) {
              for (const auto& point : marker) {
                  cout << "(" << point.x << ", " << point.y << ", " << point.z << ") ";
              }
              cout << endl;
          }
 
          // Print `point_list` (all saved images' 3D points)
          cout << "All stored point_list (3D points for all images):\n";
          for (size_t imgIdx = 0; imgIdx < point_list.size(); imgIdx++) {
              int threeDmarkerSize = point_list[imgIdx].size();
              int twoDMarkerSize = corner_list[imgIdx].size();
              cout << "Image " << imgIdx << " has " << threeDmarkerSize << " 3D makers " << ", " << twoDMarkerSize << " 2D makers:\n";
 //             for (size_t markerIdx = 0; markerIdx < point_list[imgIdx].size(); markerIdx++) {
 //                 cout << "  Marker " << markerIdx << ": ";
 //                 for (const auto& point : point_list[imgIdx][markerIdx]) {
 //                     cout << "(" << point.x << ", " << point.y << ", " << point.z << ") ";
 //                 }
 //                 cout << endl;
 //             }
          }
 
          // Print the size of `point_list`
          cout << "Total images stored in point_list: " << point_list.size() << endl;
     }
 
     /**
     * @brief Performs camera calibration using collected points
     */
     void calibrateCamera() {
         if (point_list.size() < 5) {
             cerr << "Need at least 5 calibration images. Current: " 
                 << point_list.size() << endl;
             return;
         }
 
         // Initialize camera matrix with guess
         camera_matrix = Mat::eye(3, 3, CV_64F);
         camera_matrix.at<double>(0, 2) = targetSize.width / 2.0;
         camera_matrix.at<double>(1, 2) = targetSize.height / 2.0;
 
         // Initialize distortion coefficients (5 parameters)
         distortion_coeffs = Mat::zeros(5, 1, CV_64F);
 
         vector<Mat> rvecs, tvecs;
         int flags = CALIB_FIX_ASPECT_RATIO | CALIB_RATIONAL_MODEL;
         if(point_list.size() > 10) flags |= CALIB_USE_INTRINSIC_GUESS;
 
         
         // Convert point_list to vector<vector<Point3f>>
         vector<vector<Point3f>> object_points;
         for (auto& img_points : point_list) {
             vector<Point3f> img_object_points;
             for (auto& marker_points : img_points) {
                 img_object_points.insert(img_object_points.end(), 
                     marker_points.begin(), marker_points.end());
             }
             object_points.push_back(img_object_points);
         }
 
         // Convert corner_list to vector<vector<Point2f>>
         vector<vector<Point2f>> image_points;
         for (auto& img_corners : corner_list) {
             vector<Point2f> img_image_points;
             for (auto& marker_corners : img_corners) {
                 img_image_points.insert(img_image_points.end(),
                     marker_corners.begin(), marker_corners.end());
             }
             image_points.push_back(img_image_points);
         }
 
         // Perform calibration
         reprojection_error = ::calibrateCamera(
             object_points, image_points, targetSize,
             camera_matrix, distortion_coeffs,
             rvecs, tvecs, flags
         );
 
         cout << "Calibration complete!\n";
         cout << "Reprojection error: " << reprojection_error << endl;
         cout << "Camera matrix:\n" << camera_matrix << endl;
         cout << "Distortion coefficients:\n" << distortion_coeffs << endl;
         
         calibrated = true;
     }
 
     /**
     * @brief Saves camera parameters to file
     */
     void saveCameraParameters() {
         if (!calibrated) {
             cerr << "Camera not calibrated yet!" << endl;
             return;
         }
 
         FileStorage fs(generateFilename("camera_", "_params", ".yml"), FileStorage::WRITE);
         fs << "camera_matrix" << camera_matrix;
         fs << "distortion_coefficients" << distortion_coeffs;
         fs << "reprojection_error" << reprojection_error;
         fs.release();
         
         cout << "Saved camera parameters to file" << endl;
     }
 
 
     /**
      * @brief Saves the current frame and processed frame to disk.
      */
     void saveImages() {
         imwrite(generateFilename(), imgs.frame);
         cout << "Saved normal image " << ++imageId << endl;
         // Save other frames if they exist
         switch (currentMode) {
             case Mode::ARUCO:
                 // store the point data
                 // The config is used for marker5x5_1000.png
                 saveCalibrationData(5);
             default:
                 // No additional images to save
                 break;
         }
     }
 
 // Applies image processing
     void processFrame() {
         resize(imgs.frame, imgs.frame, targetSize);
         switch (currentMode) {
             case Mode::NORMAL:
                 imshow(WINDOW_VIDEO, imgs.frame);
                 break;
             case Mode::ARUCO:
                 detector.detectMarkers(imgs.frame, corner_set, markerIds);
                 imgs.arucoFrame = imgs.frame.clone();
                 aruco::drawDetectedMarkers(imgs.arucoFrame, corner_set, markerIds);
                 imshow(WINDOW_ARUCO, imgs.arucoFrame);
                 cout << "The size of detected corner: " << corner_set.size() << endl;
                 break;
         }
     }
 
     /**
      * @brief Routes keypresses to the appropriate handler.
      * @param key The pressed key.
      */
     void handleKeyPress(char key) {
         switch (key) {
             case 's':
                 saveImages();
                 break;
             case 'c':
                 calibrateCamera();
                 break;
             case 'w':
                 saveCameraParameters();
                 break;
             case 'q':
                 exit(0);
         }
     }
 
 public:
     /**
      * @brief Constructor to initialize the CameraApp.
      * @param deviceId The ID of the camera device to use.
      */
     CameraApp(int deviceId = 0) {
         if (!cap.open(deviceId)) {
             throw runtime_error("Failed to open camera device " + to_string(deviceId));
         }
 
         Size frameSize(cap.get(CAP_PROP_FRAME_WIDTH), cap.get(CAP_PROP_FRAME_HEIGHT));
         const float reduction = 0.9;
         scale_factor = 640.0 / (frameSize.height * reduction);
         targetSize.width = frameSize.width * scale_factor;
         targetSize.height = frameSize.height * scale_factor;
         cout << "Camera initialized with resolution: " << targetSize.width << "x" << targetSize.height << endl;
 
         namedWindow(WINDOW_VIDEO, WINDOW_AUTOSIZE);
 
     }
     /**
      * @brief initialize a Aruco marker detector
      */
     void initializeArucoDetector() {
         aruco::Dictionary dictionary;
         aruco::DetectorParameters detectParams;
         aruco::RefineParameters refineParams;
         dictionary = aruco::getPredefinedDictionary(aruco::DICT_5X5_1000);
         detectParams = aruco::DetectorParameters();
         refineParams = aruco::RefineParameters();
 
         detector = aruco::ArucoDetector(dictionary, detectParams, refineParams);
     }
 
     /**
      * @brief Main application loop.
      */
     void run() {
         cout << "Default: Enable Aruco Marker detection \n"
              << "Videos controls:\n"
              << " 's' - Save photo\n"
              << " 'q' - Quit\n";
 
         try {
             initializeArucoDetector();
             while (true) {
                 if (!cap.read(imgs.frame)) {
                     throw runtime_error("Failed to capture frame");
                 }
                 if (imgs.frame.empty()) {
                     cerr << "Error: Image not loaded!" << endl;
                     return;
                 }
                 processFrame();
                 int key = waitKey(30);
                 if (key != -1) {
                     handleKeyPress(static_cast<char>(key));
                 }
             }
         } catch (const runtime_error &e) {
             cout << "Exiting: " << e.what() << endl;
         }
     }
 
     ~CameraApp() {
         cap.release();
         destroyAllWindows();
     }
 };
 
 int main(int argc, char *argv[]) {
     try {
         CameraApp app(0); //Use app(0) for default camera
         app.run();
         return 0;
     } catch (const exception &e) {
         cerr << "Error: " << e.what() << endl;
         return -1;
     }
 }