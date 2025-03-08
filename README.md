# Calibration and Augmented Reality

## Video link : 

## 👥 Team Members
1. **Yuyang Tian**
2. **Arun Mekkad**

## 💻 Environment
- **🖥️ Yuyang Tian**: macOS 10.13.1 + CLion + CMake
- **🐧 Arun Mekkad**: Ubuntu 22.04 LTS + VS Code + CMake

### 📂 File Structure
```bash
Proj3/
   ├── include/              # 📁 Header files
   ├── src/                  # 📁 Source files - most of them are executables.
   ├── markers          # 🖼️ Image set for camera calibration
   ├── CMakeLists.txt        # ⚙️ CMake build configuration
   ├── README.md             # 📖 Project documentation
```

## 📌 Instructions for Running Executables

#### **VidDisplay**
**Description**: Videostream that shows detected markers
- **Usage**:
  ```bash
  ./VidDisplay 
  Task 1 - Aruco marker detection is the default mode, simply run the program. The detected corner will be drawn and be printed out the size
  Task 2 - By hitting 's', the corner data of the current frame will be stored for calibration.
  Task 3 - Press on 'c' after saving more than 5 images to generate the calibration matrix. Subsequently on pressing 'w', the calibration parameters will be written to a .yml file in ../outputs folder.
  
  ./augReality <params_file_path>
   Eg: ./augReality ../outputs/camera_params.yml
  Task 4 - On running the above executable, the calibration parameters saved previously will be read and camera pose will be determined.
 ```