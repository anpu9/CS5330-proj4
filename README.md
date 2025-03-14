# Calibration and Augmented Reality
## Time travel days used : 1
## Video link : https://drive.google.com/drive/folders/1Stjw9mMDEN360IHDJUDq_YbTjbY992HQ

## 👥 Team Members
1. **Yuyang Tian**
2. **Arun Mekkad**

## 💻 Environment
- **🖥️ Yuyang Tian**: macOS 10.13.1 + CLion + CMake
- **🐧 Arun Mekkad**: Ubuntu 22.04 LTS + VS Code + CMake

### 📂 File Structure
```
Proj3/
   ├── include/              # 📁 Header files
   ├── src/                  # 📁 Source files - most of them are executables.
   ├── markers               # 🖼️ Image set for camera calibration
   ├── object                # 🪆 OBJ file and Point Cloud filefor 3D rendering
   ├── CMakeLists.txt        # ⚙️ CMake build configuration
   ├── README.md             # 📖 Project documentation
```
| **Filename**              | **Description**                                              |
|---------------------------| ------------------------------------------------------------ |
| `calibrate.cpp`           | Live-stream video application, enabling camera calibration   |
| `augReality.cpp`          | Augmented reality with ArUco marker detection and 3D object rendering |
| `featureDetect.cpp`       | Real-time SIFT feature detection with adjustable threshold   |
| `depthImage.cpp`          | Generates a 3D point cloud from depth data                   |
| `pcdUtil.h / pcdUtil.cpp` | Supports 3D Point Cloud generation/visualization logic       |
| `pointCloudDisplay.cpp`   | Point cloud visualization from a PCD file using command-line input |

## 📌 Instructions for Running Executables

#### **1. calibration**

**Description**: ArUco Marker Detection and Camera Calibration
- **Usage**:
- Board Configuration:
  
  * At startup, you'll be prompted to enter the column count of your marker grid
  * Press 'g' anytime to reset the column count if you switch to a different board layout
  
  ```
  ./calibration
  ```

🔹 **Tasks**:

- **Task 1**: Aruco marker detection (default mode). Detected corners will be drawn and their size printed.
- **Task 2**: Press **'s'** to save corner data for calibration.
- **Task 3**: Press **'c'** after saving more than 5 images to generate the calibration matrix. Press **'w'** to write calibration parameters to a `.yml` file in the `../outputs` folder.

#### 2.augReality

**Description**: Utilizes camera calibration parameters to determine the camera pose and render various 3D elements

- **Usage**:

  ```
  ./augReality <params_file_path> <pcd file>
  Eg: ./augReality ../outputs/camera_params.yml ../obj/matcha.pcd
  ```

🔹 **Tasks**:

- **Task 4**: On running the above executable, the calibration parameters saved previously will be read and camera pose will be determined.
- **Task 5**: Press **'p'** to toggle corner points display. Press **'a'** to toggle axes display.
- **Task 6**: Press **'t'** to toggle tetrahedron display.
- **Extension - OBJ loading** : Press **'o'** to toggle rendering of an OBJ file.
- **Extension - Hat over faces** : Press **'f'** to toggle display of a 3D cone shaped hat    over detected faces.
- **Extension - 3D Point Cloud** : Press **'c'** to toggle display of a 3D Point Cloud of a Matcha latte cup.

#### **3.featureDetection**

**Description:** Detects and displays corner points using Scale-Invariant Feature Transform (SIFT).

* **Usage**:

  ```
  ./featureDetect
  ```

🔹**Task 7**:  Run the executeable and point the camera towards a pattern of interest to display the corner points detected using Scale-Invariant Feature Transform (SIFT)

### Extension for 3D point cloud

### 4. DepthImage

**Description:** 

1. Displays a **depth map** using **DepthAnything**.

2. Generates a **3D point cloud** from video frames.

3. Saves the **PCD file** for later visualization in `PointCloud` executable.

* **Usage**:

```
./DepthImage <path to camera_params.yml>
e.g.  ./DepthImage outputs/camera_0_params.yml
```

#### 5. PointCloud

**Description**: Visualizes a **point cloud (PCD file)** using command-line input.

* Usage:

  ```
  ./PointCloud <path_to_pcd_file>
  ```