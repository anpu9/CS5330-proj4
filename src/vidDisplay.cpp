/*
 * Authors: Yuyang Tian and Arun Mekkad
 * Date: 2025/2/28
 * Purpose: Implementation of camera calibration
 */

#include <opencv2/opencv.hpp>
#include <iostream>


using namespace cv;
using namespace std;

class CameraApp {
private:
    // Modes for image processing
    enum class Mode {
        NORMAL,

    };

    // Constants for window names
    const string WINDOW_VIDEO = "Live";

    // Member variables
    VideoCapture cap;
    const string OUTPUT_DIR = "../outputs/";
    int imageId = 0;
    Mode currentMode = Mode::NORMAL;
    float scale_factor;
    Size targetSize;

    struct Images {
        Mat frame;
    } imgs;

    string generateFilename(const string &task = "", const string &suffix = "") {
        return OUTPUT_DIR + task + to_string(imageId) + suffix + ".png";
    }

    /**
     * @brief Saves the current frame and processed frame to disk.
     */
    void saveImages() {
        imwrite(generateFilename(), imgs.frame);
        cout << "Saved normal image " << ++imageId << endl;
        // Save other frames if they exist
//        switch (currentMode) {
//
//            default:
//                // No additional images to save
//                break;
//        }
    }


// Applies image processing
    void processFrame() {
        resize(imgs.frame, imgs.frame, targetSize);
        imshow(WINDOW_VIDEO, imgs.frame);
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
     * @brief Main application loop.
     */
    void run() {
        cout << "Videos on, controls:\n"
             << " 's' - Save photo\n"
             << " 'q' - Quit\n";
        try {
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
