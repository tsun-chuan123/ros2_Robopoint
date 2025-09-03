# ros2_Robopoint

This repository contains a set of ROS&nbsp;2 packages used to run the **RoboPoint** demo.  The main components are:

- `robopoint_worker` – provides a worker node that loads a multimodal model and exposes services for text/image generation.
- `robopoint_controller` – manages worker registration and dispatches requests.
- `robopoint_ros2` – client node for sending queries and visualizing results.
- `robopoint_gui` – GUI interface node for direct user interaction and result visualization.
- `robopoint_interfaces` – custom messages and service definitions used by the other packages.

A Docker environment is provided to simplify setup.

## Prerequisites

- Docker and NVIDIA GPU drivers (for GPU acceleration)
- ROS&nbsp;2 Humble (already installed inside the Docker image)

## Building the Docker image

```bash
cd docker
docker build -t ros2-humble-dockeruser -f Dockerfile .
```

## Running the container

Use the helper script which launches the container with GPU and X11 support:

```bash
bash docker/docker_run.sh
```

When the script starts you will be dropped into an interactive shell inside `/workspace`.  Choose `ros2_ws` from the menu or manually enter:

```bash
cd ros2_ws
```

## Building the workspace

Inside the container build all packages with `colcon` and source the environment:

```bash
source /opt/ros/humble/setup.bash
colcon build --merge-install
source install/local_setup.bash
```

## Launching the nodes

1. **Controller** – starts the worker manager:

   ```bash
   ros2 launch robopoint_controller controller.launch.py
   ```

2. **Worker** – loads a model and connects to the controller:

   ```bash
   ros2 launch robopoint_worker model_worker.launch.py model_path:=wentao-yuan/robopoint-v1-vicuna-v1.5-13b
   ```

   You can override model settings by passing parameters, e.g. `model_path:=wentao-yuan/robopoint-v1-vicuna-v1.5-13b`.

3. **Client** – sends queries and publishes results:

   ```bash
   ros2 launch robopoint_ros2 robopoint_launch.py
   ```

   Use `--ros-args` to set parameters such as `controller_url` or `model_name`.

4. **GUI Node** – optional Qt-based interface for sending queries and displaying results:

   ```bash
   ros2 run robopoint_gui robopoint_gui_node
   ```

5. **Stretch Bridge** – converts RoboPoint affordance results into goal points for Stretch:

   Using the launch file (推薦方式 / Recommended):
   ```bash
   ros2 launch robopoint_stretch_bridge stretch_goal_follower.launch.py
   ```
   
   Or using the node directly:
   ```bash
   ros2 run robopoint_stretch_bridge stretch_bridge
   ```
   This node listens to affordance outputs from robopoint_node, extracts the highest-confidence point, projects it into 3D using depth data and camera intrinsics, and publishes a geometry_msgs/PointStamped goal for Stretch to act upon.

### 如何使用 stretch_goal_follower.launch.py (中文說明)

`stretch_goal_follower.launch.py` 是一個 ROS2 launch 檔案，用於啟動 Stretch 機器人目標跟隨功能。此檔案會啟動 `robopoint_stretch_bridge` 節點，該節點負責：

1. **訂閱 RoboPoint 辨識結果**：從 `spatial_affordance_response` 主題接收物體可操作性分析結果
2. **訂閱深度影像**：從相機獲取深度資訊
3. **2D 到 3D 轉換**：將 2D 影像座標轉換為 3D 世界座標
4. **發布目標點**：向 `stretch_goal_point` 主題發布 Stretch 機器人的目標位置

#### 基本使用方法：

```bash
ros2 launch robopoint_stretch_bridge stretch_goal_follower.launch.py
```

#### 自訂參數使用方法：

```bash
ros2 launch robopoint_stretch_bridge stretch_goal_follower.launch.py \
    depth_topic:=/realsense/depth/image_raw \
    camera_frame:=camera_depth_optical_frame \
    fx:=616.0 \
    fy:=616.0 \
    cx:=320.0 \
    cy:=240.0
```

#### 可調整參數說明：
- `depth_topic`: 深度影像主題名稱 (預設: `/camera/depth/image_raw`)
- `camera_frame`: 相機座標系 ID (預設: `camera_frame`)
- `fx`, `fy`: 相機焦距參數 (預設: 525.0)
- `cx`, `cy`: 相機主點座標 (預設: 319.5, 239.5)
   
## Repository layout

```
ros2_ws/            ROS 2 workspace
├── src/
│   ├── robopoint_worker/       Worker node package
│   ├── robopoint_controller/   Controller node package
│   ├── robopoint_ros2/         Client demo node
│   ├── robopoint_gui/          GUI interface node
│   ├── robopoint_interfaces/   Interface definitions
│   └── robopoint_interfaces/   Message/service definitions
└── ...
```

`docker/` contains the Dockerfile and `docker_run.sh` helper.

## Notes

The worker node relies on a GPU to run the chosen model.  Make sure your host machine has a compatible NVIDIA GPU and that Docker is configured to access it.
