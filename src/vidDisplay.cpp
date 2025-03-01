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
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> corner_set;

    // Constants for window names
    const string WINDOW_VIDEO = "Live";
    const string WINDOW_ARUCO = "Aruco Markers";

    // Member variables
    VideoCapture cap;
    const string OUTPUT_DIR = "../outputs/";
    int imageId = 0;
    Mode currentMode = Mode::ARUCO;
    cv::aruco::ArucoDetector detector;
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
     * @brief Saves the current frame and processed frame to disk.
     */
    void saveImages() {
        imwrite(generateFilename(), imgs.frame);
        cout << "Saved normal image " << ++imageId << endl;
        // Save other frames if they exist
        switch (currentMode) {
            case Mode::ARUCO:
                imwrite(generateFilename("aruco_"), imgs.frame);
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
                cv::aruco::drawDetectedMarkers(imgs.arucoFrame, corner_set, markerIds);
                imshow(WINDOW_ARUCO, imgs.arucoFrame);
                cout << "The size of detected corner" << corner_set.size() << endl;
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
        cv::aruco::Dictionary dictionary;
        cv::aruco::DetectorParameters detectParams;
        cv::aruco::RefineParameters refineParams;
        dictionary = aruco::getPredefinedDictionary(aruco::DICT_5X5_1000);
        detectParams = aruco::DetectorParameters();
        refineParams = aruco::RefineParameters();

        detector = cv::aruco::ArucoDetector(dictionary, detectParams, refineParams);
    }

    /**
     * @brief Main application loop.
     */
    void run() {
        cout << "Default: Enable Aruco Marker detection "
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
