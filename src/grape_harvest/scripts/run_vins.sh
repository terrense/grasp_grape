#!/bin/sh
# Run VINS-Mono against the running simulation.
#
# The vins-mono:noetic image already contains a *built* /catkin_ws: its
# Dockerfile clones VINS-Mono and runs catkin_make at image build time. The
# docker-compose.yml next to it mounts empty host build/ and devel/ directories
# on top of that, which is exactly why `source /catkin_ws/devel/setup.bash`
# never found anything -- the mounts shadowed the thing they were meant to
# expose. So this script mounts nothing but the config.
#
# --net host --ipc host so the container shares the host ROS master and can use
# shared-memory image transport; the sim must already be up.
#
#   sh run_vins.sh            start it
#   sh run_vins.sh stop       stop it
#   sh run_vins.sh log        follow its output
set -e

NAME=vins-mono-run
IMAGE=vins-mono:noetic
CFG_DIR=/home/shenxin/grape_ws/src/grape_harvest/config
CFG=/cfg/vineyard_vins.yaml

case "$1" in
stop)
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    echo "stopped $NAME"
    exit 0
    ;;
log)
    docker logs -f "$NAME"
    exit 0
    ;;
esac

if [ ! -f "$CFG_DIR/vineyard_vins.yaml" ]; then
    echo "no config: run  rosrun grape_harvest make_vins_config.py $CFG_DIR/vineyard_vins.yaml"
    echo "against a running sim first (it reads camera_info and TF)."
    exit 1
fi

docker rm -f "$NAME" >/dev/null 2>&1 || true
docker run -d --name "$NAME" \
    --net host --ipc host \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -v "$CFG_DIR":/cfg:ro \
    "$IMAGE" \
    bash -lc "mkdir -p /catkin_ws/vins_output/pose_graph && \
              source /catkin_ws/devel/setup.bash && \
              roslaunch vins_estimator euroc.launch \
                  config_path:=$CFG \
                  vins_path:=/catkin_ws/src/VINS-Mono/config/../"

echo "started $NAME"
sleep 12
docker logs --tail 20 "$NAME" 2>&1 || true
