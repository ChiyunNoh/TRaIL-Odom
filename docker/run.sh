#!/usr/bin/env bash

xhost +

PROJECT_DIR="/path/to/your/ros2_ws"
DATASET_DIR="/path/to/your/dataset"

docker run --rm -it --ipc=host --net=host --privileged \
    --env="DISPLAY" \
    --env="RMW_IMPLEMENTATION=rmw_zenoh_cpp" \
    --name trail_odom \
    --user root \
    --hostname trail-odom \
    --entrypoint /bin/bash \
    --volume="$PROJECT_DIR:/root/ros2_ws/src/TRaIL-Odom" \
    --volume="$DATASET_DIR:/root/data" \
    chiyunnn/trail-odom:latest \
    -lc '
source /opt/ros/humble/setup.bash

if ! ros2 pkg prefix rmw_zenoh_cpp >/dev/null 2>&1; then
  echo "ERROR: rmw_zenoh_cpp is not installed in chiyunnn/trail-odom:latest." >&2
  echo "Rebuild it by running ./build.sh from the repository docker directory." >&2
  exit 1
fi

cat > /root/.docker_prompt.bash <<'"'"'EOF'"'"'
PS1='\''\[\e[1;33m\][DOCKER]\[\e[0m\] \[\e[1;34m\]\u@\h:\[\e[0m\]\[\e[1;36m\]\w\[\e[0m\]$ '\''
export PS1
EOF

grep -qxF "source /root/.docker_prompt.bash" /root/.bashrc || \
  echo "source /root/.docker_prompt.bash" >> /root/.bashrc

cd /root/ros2_ws 2>/dev/null || cd /root
exec bash
'

xhost -
