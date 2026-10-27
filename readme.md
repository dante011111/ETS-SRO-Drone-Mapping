# Drone 3D Mapping in CoppeliaSim

This project demonstrates automated 3D mapping using a Quadcopter equipped with a Perspective Vision Sensor in CoppeliaSim. The drone flies autonomously over a designated area to scan geometric objects (boxes and cylinders) and generates 2D and 3D point cloud maps. The data is then post-processed to perform voxel downsampling and height-based color mapping.

## 🚀 Features & Grading Criteria Addressed

This repository is built to fulfill the following hierarchical tasks:
*   **Grade C:** Autonomous drone control and flight path visualization (`--mode path`).
*   **Grade BC:** 2D scanning using a spinning, single-row depth sensor (`--mode 2d`).
*   **Grade B:** 3D scanning using a tilted depth camera and lawn-mower trajectory (`--mode 3d`).
*   **Grade AB:** Point cloud optimization via voxel downsampling.
*   **Grade A:** Post-process height color mapping (Z-axis).

## 📋 Prerequisites

*   **Simulator:** [CoppeliaSim](https://www.coppeliarobotics.com/) (Version >= 4.5 recommended for ZMQ Remote API support).
*   **Python 3.x**
*   **Python Packages:**
    ```bash
    pip install coppeliasim-zmqremoteapi-client numpy matplotlib
    pip install open3d  # Optional, for advanced 3D viewing
    ```

## 📂 Repository Structure

```text
.
├── ETS Mapping.ttt       # The main CoppeliaSim scene file
├── drone_mapping.py      # The Python script to control the drone and acquire depth data
├── postprocess.py        # The Python script to filter, voxelize, and plot the map
├── README.md             # Project documentation
└── output/               # Auto-generated folder for point clouds and plots
    ├── path_3d.csv
    ├── scan3d.npy
    ├── map3d.png
    └── ...
```

## 🛠️ Usage Instructions

### 1. Preparing the Simulation
1. Open **CoppeliaSim**.
2. Load the scene file: `File` > `Open scene...` > Select `ETS Mapping.ttt`.
3. *Note: Ensure there are no lingering, incorrectly configured sensors (like Spherical Vision sensors) attached to the drone. The script will automatically generate the correct Perspective Depth Sensor upon execution.*

### 2. Running the Data Acquisition (Flight & Scanning)
Open your terminal in the project directory and run the mapping script. **Do not stop the simulation manually**; let the script finish the trajectory and print `[done]` to ensure the data is saved successfully.

*   **For full 3D Mapping (Grade B):**
    ```bash
    python drone_mapping.py --mode 3d
    ```
*   **For 2D Mapping (Grade BC):**
    ```bash
    python drone_mapping.py --mode 2d
    ```
*   **For Path Visualization only (Grade C):**
    ```bash
    python drone_mapping.py --mode path
    ```
*(Data will be saved as `.npy` and `.csv` files inside the `output/` directory).*

### 3. Post-Processing & Visualization
Once the flight is complete, process the generated raw data to create the final maps.

*   **Voxel Downsampling & Height Coloring (Grades AB & A):**
    ```bash
    python postprocess.py --mode 3d --voxel 0.15
    ```
    This command cleans the data, downsamples the point cloud into 0.15m voxels, colors them based on height, and displays a 3D matplotlib plot along with the drone's flight path.

*   **View 2D Scan Data:**
    ```bash
    python postprocess.py --mode 2d
    ```

*   **Advanced Viewing (Open3D):**
    If you have `open3d` installed, you can use the `--o3d` flag for a more interactive 3D environment:
    ```bash
    python postprocess.py --mode 3d --voxel 0.15 --o3d
    ```

## ⚠️ Troubleshooting
*   **Empty Data/Missing Points:** If your `scan3d.npy` is missing or not updating, make sure `sim.getVisionSensorDepth(sensor, 0)` is using `0` as the second argument in `drone_mapping.py` to return normalized data instead of absolute metric distances.
*   **Mirrored Map:** If the generated 3D plot looks flipped compared to the CoppeliaSim scene, toggle `FLIP_U = True` or `FLIP_V = True` at the top of `drone_mapping.py` and run the simulation again.