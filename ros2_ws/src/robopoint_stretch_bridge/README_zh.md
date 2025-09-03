# Stretch Goal Follower 使用說明 (中文)

## 概述
`stretch_goal_follower.launch.py` 是一個 ROS2 launch 檔案，用於啟動 Stretch 機器人的目標跟隨功能。

## 功能說明
此 launch 檔案會啟動 `robopoint_stretch_bridge` 節點，該節點：
1. 訂閱 RoboPoint 的物體可操作性分析結果
2. 訂閱相機的深度影像
3. 將 2D 影像座標轉換為 3D 世界座標
4. 發布 Stretch 機器人的目標位置

## 基本使用方法

### 1. 基本啟動
```bash
ros2 launch robopoint_stretch_bridge stretch_goal_follower.launch.py
```

### 2. 自訂相機參數啟動
```bash
ros2 launch robopoint_stretch_bridge stretch_goal_follower.launch.py \
    depth_topic:=/realsense/depth/image_raw \
    camera_frame:=camera_depth_optical_frame \
    fx:=616.0 \
    fy:=616.0 \
    cx:=320.0 \
    cy:=240.0
```

## 參數說明

| 參數名稱 | 預設值 | 說明 |
|---------|--------|------|
| `depth_topic` | `/camera/depth/image_raw` | 深度影像的 ROS 主題名稱 |
| `camera_frame` | `camera_frame` | 相機座標系的框架 ID |
| `fx` | `525.0` | 相機在 X 軸的焦距參數 |
| `fy` | `525.0` | 相機在 Y 軸的焦距參數 |
| `cx` | `319.5` | 相機主點的 X 座標 |
| `cy` | `239.5` | 相機主點的 Y 座標 |

## 輸入主題 (Subscribed Topics)
- `spatial_affordance_response` (robopoint_interfaces/SpatialAffordanceResponse): RoboPoint 的物體可操作性分析結果
- `depth_topic` 參數指定的主題 (sensor_msgs/Image): 深度影像

## 輸出主題 (Published Topics)  
- `stretch_goal_point` (geometry_msgs/PointStamped): Stretch 機器人的 3D 目標位置

## 使用流程
1. 確保 RoboPoint 系統正在運行並發布可操作性分析結果
2. 確保相機正在發布深度影像
3. 啟動此 launch 檔案
4. 系統會自動將檢測到的可操作點轉換為 Stretch 機器人的目標位置

## 故障排除
- 如果沒有收到目標點，請檢查 RoboPoint 是否正在發布結果到 `spatial_affordance_response` 主題
- 如果深度轉換不正確，請確認相機參數設定是否正確
- 使用 `ros2 topic echo /stretch_goal_point` 來檢查是否有目標點輸出