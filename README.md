
# IMU Calibration Package for ROS 2

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![License](https://img.shields.io/badge/license-BSD-blue.svg)

`imu_calib` is a ROS 2 package designed to compute and apply accelerometer calibration parameters for IMU (Inertial Measurement Unit) sensors. The package includes tools for collecting calibration data, computing calibration parameters, and applying these parameters to raw IMU data.

**NOTE** The ROS 1 version is avaible at https://github.com/dpkoch/imu_calib
---

## Features

- **Calibration Workflow**:
  - Collect calibration data for accelerometers.
  - Compute scale, misalignment, and bias corrections.
  - Save and load calibration parameters.

- **Real-Time Correction**:
  - Apply calibration to raw IMU data.
  - Publish corrected IMU data for downstream use.

- **Fully Configurable**:
  - YAML-based configuration for calibration parameters.
  - Adjustable sampling rates and thresholds.

---

## Package Contents

The package contains the following nodes and libraries:

### Nodes
- `do_calib`: Collect calibration data and compute calibration parameters.
- `apply_calib`: Apply calibration parameters to raw IMU data.

### Library
- `accel_calib`: Core library for accelerometer calibration.

---

## Installation

### Prerequisites

Ensure you have the following dependencies installed:
- **ROS 2** (Humble or newer)
- **Eigen3** (for matrix operations)
- **yaml-cpp** (for YAML file handling)

### Clone and Build
```bash
# Clone the repository
git clone https://github.com/your-username/imu_calib.git

# Navigate to the workspace
cd your_workspace/src

# Build the package
colcon build
```

### Source the Workspace
```bash
source install/setup.bash
```

---

## Usage

### 1. Running Calibration

#### Launch the Calibration Node
```bash
ros2 run imu_calib do_calib \
--ros-args \
  -p measurements:=1000 \
  -p reference_acceleration:=9.81 \
  -p output_file:=/home/user/imu_calibration.yaml \
  -r imu:=/imu/data

```

#### Parameters
| Parameter             | Type    | Default Value     | Description                                      |
|-----------------------|---------|-------------------|--------------------------------------------------|
| `measurements`        | `int`   | `500`             | Number of measurements per orientation.         |
| `reference_acceleration` | `double` | `9.80665`     | Expected acceleration in m/s² (gravity).        |
| `output_file`         | `string`| `imu_calib.yaml`  | File path to save the calibration parameters.    |

### 2. Applying Calibration

#### Launch the Apply Calibration Node
```bash
ros2 run imu_calib apply_calib \
--ros-args \
  -p calib_file:=/path/to/custom_calib.yaml \
  -p calibrate_gyros:=false \
  -p gyro_calib_samples:=200 \
  -r raw:=/imu/raw_data \
  -r corrected:=/imu/corrected_data
```

#### Parameters
| Parameter             | Type    | Default Value     | Description                                      |
|-----------------------|---------|-------------------|--------------------------------------------------|
| `calib_file`          | `string`| `imu_calib.yaml`  | File path to load calibration parameters.        |
| `calibrate_gyros`     | `bool`  | `true`            | Whether to calibrate gyroscope biases.           |
| `gyro_calib_samples`  | `int`   | `100`             | Number of samples for gyro calibration.          |
| `queue_size`          | `int`   | `10`              | Message queue size for subscriptions/publishers. |

---

## Input and Output Topics

### `do_calib`
| Topic         | Type                | Description                     |
|---------------|---------------------|---------------------------------|
| `/imu`        | `sensor_msgs/Imu`   | Raw IMU data for calibration.   |

### `apply_calib`
| Topic         | Type                | Description                     |
|---------------|---------------------|---------------------------------|
| `/raw`        | `sensor_msgs/Imu`   | Raw IMU data for correction.    |
| `/corrected`  | `sensor_msgs/Imu`   | Corrected IMU data.             |

---

## File Structure

```plaintext
imu_calib/
├── include/imu_calib
│   ├── accel_calib.h
│   ├── apply_calib.h
│   └── do_calib.h
├── src
│   ├── accel_calib.cpp
│   ├── apply_calib.cpp
│   ├── do_calib.cpp
│   ├── apply_calib_node.cpp
│   └── do_calib_node.cpp
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## Contributing

Contributions are welcome! Please follow these steps:
1. Fork the repository.
2. Create a feature branch.
3. Commit your changes.
4. Submit a pull request.

---

## License

This project is licensed under the BSD License. See the [LICENSE](LICENSE) file for details.

---

## Authors

- **Daniel Koch** - Original Developer
- **[Your Name]** - ROS 2 Migration

---

## Acknowledgments

This project is based on the original ROS 1 `imu_calib` package by Daniel Koch. (https://github.com/dpkoch/imu_calib)


### Key Features of This README:

1. **Clear Sections**:
   - Divided into clear categories like installation, usage, and contributing.

2. **Markdown Tables**:
   - Used for listing parameters and topics for better readability.

3. **Code Blocks**:
   - Added for easy copying of commands.

4. **File Structure**:
   - Gives an overview of the package organization.

5. **Acknowledgment and License**:
   - Proper credits and license information included.