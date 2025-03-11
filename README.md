# Calibration and Augmented Reality

## Video link : 

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
   ├── markers          # 🖼️ Image set for camera calibration
   ├── CMakeLists.txt        # ⚙️ CMake build configuration
   ├── README.md             # 📖 Project documentation
```

## 📌 Instructions for Running Executables

#### **VidDisplay**
**Description**: ArUco Marker Detection and Camera Calibration
- **Usage**:
- Board Configuration:
  At startup, you'll be prompted to enter the column count of your marker grid
  Press 'g' anytime to reset the column count if you switch to a different board layout
  ```
  ./VidDisplay 
  Task 1 - Aruco marker detection is the default mode, simply run the program. The detected corner will be drawn and be printed out the size
  Task 2 - By hitting 's', the corner data of the current frame will be stored for calibration.
  Task 3 - Press on 'c' after saving more than 5 images to generate the calibration matrix. Subsequently on pressing 'w', the calibration parameters will be written to a .yml file in ../outputs folder.
#### augReality

**Description**: Utilizes camera calibration parameters to determine the camera pose and render various 3D elements

- **Usage**:

```
./augReality <params_file_path>
Eg: ./augReality ../outputs/camera_params.yml
Task 4 - On running the above executable, the calibration parameters saved previously will be read and camera pose will be determined.
Task 5 - Press 'p' to toggle corner points display and 'a' to toggle axes display
Task 6 - Press 't' to toggle tetrahedron display
```

#### **featureDetection**

**Description:** Detects and displays corner points using Scale-Invariant Feature Transform (SIFT).

* **Usage**:

 ```
 Task 7 - Run the executeable and point the camera towards a pattern of interest to display the corner points detected using Scale-Invariant Feature Transform (SIFT)