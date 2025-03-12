/*
 * Authors: Yuyang Tian and Arun Mekkad
 * Date: 2025/3/12
 * Purpose: Point cloud visualization from a PCD file using command-line input
 */

#include <opencv2/opencv.hpp>
#include "../include/DA2Network.hpp"
#include "../include/pcl3d.h"

using namespace std;
using namespace cv;

int main(int argc, char* argv[]) {
    // Check if the correct number of arguments is provided
    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <path_to_pcd_file>" << endl;
        return 1;  // Exit with error code
    }

    // Get the filename from command-line arguments
    std::string outputFilename = argv[1];

    // Visualize the specified PCD file
    visualizePointCloudByFile(outputFilename);

    cout << "Terminating" << endl;
    return 0;
}
