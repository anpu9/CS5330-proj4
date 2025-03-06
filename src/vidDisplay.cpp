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

    struct Images {
        Mat frame;
        Mat arucoFrame;
    } imgs;

    string generateFilename(const string &prefix = "", const string &suffix = "") {
        return OUTPUT_DIR + prefix + to_string(imageId) + suffix + ".png";
    }
     /**
      * @brief Saves the current frame and detected corners for calibration.
      * @param cols the cols of markers in the calibration images
      * @param square_size the size of a target square, by default, is in units of target squares, which is 1
      */
    void saveCalibrationData(int cols, float square_size = 1.0f) {
        if (corner_set.empty()) {
            cerr << "No valid markers detected. Image not saved for calibration." << endl;
            return;
        }
        vector<vector<Point3f>> framePoints; // Temporary container for 3D current image points
        for (size_t i = 0; i < corner_set.size(); i++) {
            // Determine marker's row & column in a grid structure
            int grid_x = markerIds[i] % cols;  // Marker ID defines X position
            int grid_y = markerIds[i] / cols;  // Marker ID defines Y position

            // Generate 3D world coordinates based on unit squares, assuming it's a flat plane
            vector<Point3f> markerPoints;
            markerPoints.emplace_back(Point3f(grid_x * square_size, -grid_y * square_size, 0));   // Top-left
            markerPoints.emplace_back(Point3f((grid_x + 1) * square_size, -grid_y * square_size, 0));   // Top-right
            markerPoints.emplace_back(Point3f((grid_x + 1) * square_size, -(grid_y + 1) * square_size, 0));   // Bottom-right
            markerPoints.emplace_back(Point3f(grid_x * square_size, -(grid_y + 1) * square_size, 0));    // Bottom-left

            framePoints.emplace_back(markerPoints);
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
            case 'q':
                exit(0);
                break;
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
        const float reduction = 0.8;
        scale_factor = 256.0 / (frameSize.height * reduction);
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
        CameraApp app(0);
        app.run();
        return 0;
    } catch (const exception &e) {
        cerr << "Error: " << e.what() << endl;
        return -1;
    }
}
