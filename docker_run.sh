#!/usr/bin/env sh
xhost +local:docker

# Setup the style of color
COLOR_RED='\033[0;31m'
COLOR_YELLOW='\033[0;33m'
COLOR_NC='\033[0m'

#
# Check the command 'nvidia-docker' existed or not
#
ret_code="$(command -v nvidia-docker)"
if [ -z "$ret_code" ]; then
    if [ -z "$(command -v nvidia-container-cli)" ]; then
        NVIDIA_SUPPORT_OPTION="--gpus all "
    else
        NVIDIA_SUPPORT_OPTION=""
    fi
    DOCKER_CMD="docker"
else
    DOCKER_CMD="nvidia-docker"
    NVIDIA_SUPPORT_OPTION=""
fi


#
# Specify cuda version
#
if [ $# -gt 0 ]; then
    if [[ "$1" == "cuda10" || "$1" == "cuda10.0" ]] ; then
        echo -e "RUN: \"${DOCKER_CMD}\""
        DOCKER_TAG="cuda10.0"
    elif [ "$1" == "same" ] ; then
        echo -e "RUN: \"docker exec\""
    else
        echo -e "Please specify which cuda version your GPU support."
        echo -e "${COLOR_RED}Usage: source docker_run.sh [cuda10 | same]${COLOR_NC}"
    fi
else
    echo -e "${COLOR_RED}Usage: source docker_run.sh [cuda10| same]${COLOR_NC}"
fi

# Find current directory and transfer it to container directory for Docker
current_dir="$(pwd)"
host_dir="${HOME}/"
container_dir="/home/developer/"
goal_dir=${current_dir//$host_dir/$container_dir}
#echo "goal_dir: \"${goal_dir}\""


#
# Execute command
#
if [ $# -gt 0 ]; then
    if [ "$1" == "same" ]; then
        docker exec -it walker-container bash
    else
        ${DOCKER_CMD} run ${NVIDIA_SUPPORT_OPTION} --name walker-container --rm -it --net=host --privileged -v /dev:/dev \
            -e DISPLAY=$DISPLAY \
            -v /etc/localtime:/etc/localtime:ro -v /var/run/docker.sock:/var/run/docker.sock \
            -v ${current_dir}:${goal_dir} \
            -v /tmp/.X11-unix/:/tmp/.X11-unix:rw \
            -w ${goal_dir} \
            --device=/dev/dri:/dev/dri \
            --device=/dev/nvhost-ctrl \
            --device=/dev/nvhost-ctrl-gpu \
            --device=/dev/nvhost-prof-gpu \
            --device=/dev/nvmap \
            --device=/dev/nvhost-gpu \
            --device=/dev/nvhost-as-gpu \
            -v /dev/bus/usb:/dev/bus/usb \
            coolcat647/walker-docker:${DOCKER_TAG}      
    fi
else
    echo "please provide docker tag name."
fi
