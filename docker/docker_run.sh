#!/bin/bash

CONTAINER_NAME=ros2_humble_docker
IMAGE_NAME=ros2-humble-dockeruser
WORKDIR=/workspace
HOST_DIR=$HOME/ros2_Robopoint
HUGGINGFACE_TOKEN=${HUGGINGFACE_TOKEN:-"your_token_here"}
DISPLAY_TO_USE=${DISPLAY:-":0"}
XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}

# 檢查 XAUTHORITY 是否存在
if [ ! -f "$XAUTHORITY" ]; then
  echo "找不到 XAUTHORITY 檔案：$XAUTHORITY"
  echo "請確認你是透過 MobaXterm SSH 登入，並啟用 X11 forwarding"
  exit 1
fi

# 判斷是否為本地 X11（:0 或 :1）或遠端 X11 forwarding (localhost:10.0)
IS_LOCAL_DISPLAY=false
if [[ "$DISPLAY_TO_USE" =~ ^: ]]; then
  IS_LOCAL_DISPLAY=true
fi

# xhost +local:docker 只有本地桌面需要用
if $IS_LOCAL_DISPLAY; then
  xhost +local:docker
else
  echo "MobaXterm detected, skip xhost"
fi

start_interactive_session() {
  docker exec -e DISPLAY=$DISPLAY_TO_USE -e XAUTHORITY=/root/.Xauthority -e ROS_DOMAIN_ID=15 -u root -it $CONTAINER_NAME bash -ic '
    export ROS_DOMAIN_ID=15
    CHOICE=$(whiptail --title "ROS 2 Launch Menu" --menu "Please select a task:" 18 70 10 \
      "Controller" "starts the worker manager" \
      "Worker" "loads a model and connects to the controller" \
      "Client" "sends queries and publishes results" \
      "GUI" "sends queries and displaying results" \
      "Stretch" "Maps RoboPoint points to Stretch goals" \
      "ros2_ws" "Enter ros2_ws workspace" \
      "skip" "Skip all tasks and enter the container" 3>&1 1>&2 2>&3)

    case "$CHOICE" in
      Controller)
        export ROS_DOMAIN_ID=15
        cd ros2_ws/
        source /opt/ros/humble/setup.bash
        colcon build --merge-install
        source install/local_setup.bash
        ros2 launch robopoint_controller controller.launch.py
        ;;
      Worker)
        export ROS_DOMAIN_ID=15
        cd ros2_ws/
        source /opt/ros/humble/setup.bash
        colcon build --merge-install
        source install/local_setup.bash
        ros2 launch robopoint_worker model_worker.launch.py model_path:=wentao-yuan/robopoint-v1-vicuna-v1.5-13b
        ;;
      Client)
        export ROS_DOMAIN_ID=15
        cd ros2_ws/
        source /opt/ros/humble/setup.bash
        colcon build --merge-install
        source install/local_setup.bash
        ros2 launch robopoint_ros2 robopoint_launch.py
        ;;
      GUI)
        export ROS_DOMAIN_ID=15
        cd ros2_ws/
        source /opt/ros/humble/setup.bash
        colcon build --merge-install
        source install/local_setup.bash
        ros2 run robopoint_gui robopoint_gui_node
        ;;
      Stretch)
        export ROS_DOMAIN_ID=15
        cd ros2_ws/
        source /opt/ros/humble/setup.bash
        colcon build --merge-install
        source install/local_setup.bash
        ros2 launch robopoint_stretch_bridge stretch_goal_follower.launch.py
        ;;
      ros2_ws)
        export ROS_DOMAIN_ID=15
        cd ros2_ws/
        source /opt/ros/humble/setup.bash
        colcon build --merge-install
        source install/local_setup.bash
        exec bash
        ;;
      skip)
        exec bash
        ;;
    esac
  '
}

if [ "$(docker ps -aq -f name=^/${CONTAINER_NAME}$)" ]; then
  if [ "$(docker ps -q -f status=running -f name=^/${CONTAINER_NAME}$)" ]; then
    echo "Container is already running. Attaching..."
    start_interactive_session
  else
    echo "Starting existing container..."
    docker start $CONTAINER_NAME > /dev/null
    sleep 1
    start_interactive_session
  fi
else
  echo "🆕 Creating and running new container..."

  DOCKER_CMD="docker run -dit \
    --privileged \
    --name $CONTAINER_NAME \
    -u root \
    -e DISPLAY=$DISPLAY_TO_USE \
    -e XAUTHORITY=/root/.Xauthority \
    -v $XAUTHORITY:/root/.Xauthority \
    -e HUGGINGFACE_TOKEN=$HUGGINGFACE_TOKEN \
    -e QT_X11_NO_MITSHM=1 \
    -e PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native \
    -e ROS_DOMAIN_ID=15 \
    --device /dev/snd \
    --group-add audio \
    -v $HOST_DIR:$WORKDIR \
    -v /dev:/dev \
    -v /run/user/1000/pulse:/run/user/1000/pulse \
    -v /etc/machine-id:/etc/machine-id \
    -v /run/dbus:/run/dbus \
    -v $XDG_RUNTIME_DIR/pulse:$XDG_RUNTIME_DIR/pulse \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -w $WORKDIR \
    --network host \
    $IMAGE_NAME bash"

  eval $DOCKER_CMD

  sleep 1
  start_interactive_session
fi
