#!/usr/bin/env bash


DOCKER_REPO="mzahana:vins_gpu"
CONTAINER_NAME="vins_gpu"
USER_VOLUME=""
USER_COMMAND=""
CONTAINER_USER_NAME=xaviernx

# This will enable running containers with different names
# It will create a local workspace and link it to the image's catkin_ws
if [ "$1" != "" ]; then
    CONTAINER_NAME=$1
fi

WORKSPACE_DIR=${HOME}/${CONTAINER_NAME}_shared_volume/
if [ ! -d $WORKSPACE_DIR ]; then
    mkdir -p $WORKSPACE_DIR
fi
echo "Container name:$CONTAINER_NAME WORSPACE DIR:$WORKSPACE_DIR"


##################################################################################


echo "Starting Container: ${CONTAINER_NAME} with REPO: $DOCKER_REPO"
 
if [ "$(docker ps -aq -f name=${CONTAINER_NAME})" ]; then
    if [ "$(docker ps -aq -f status=exited -f name=${CONTAINER_NAME})" ]; then
        # cleanup
        docker start ${CONTAINER_NAME}
    fi
    if [ -z "$CMD" ]; then
        docker exec -it  ${CONTAINER_NAME} bash
    else
        docker exec -it  ${CONTAINER_NAME} bash -c "$CMD"
    fi
else


####################################################################################
ENV_VARS_FILE_NAME="vins_env_vars.txt"
if [ -d "$HOME/jetson_vins_fusion_docker" ]; then
    . $HOME/jetson_vins_fusion_docker/scripts/set_env_vars.sh $ENV_VARS_FILE_NAME
    echo "Found $HOME/jetson_vins_fusion_docker/scripts/set_env_vars.sh"
elif [ -d "$HOME/src/jetson_vins_fusion_docker" ]; then
    . $HOME/src/jetson_vins_fusion_docker/scripts/set_env_vars.sh $ENV_VARS_FILE_NAME
    echo "Found $HOME/src/jetson_vins_fusion_docker/scripts/set_env_vars.sh"
else
    echo "ERROR Could not find jetson_vins_fusion_docker package. Exiting" && echo
    exit 1
fi
####################################################################################


echo "Updated environment variables" && echo
sleep 1

# cmd_str is exported by the set_env_vars.sh script
CMD=" eval $cmd_str &&\
       /bin/bash"

# run the container
xhost +si:localuser:root
# xhost +local:root
#xhost +
docker run --runtime nvidia -it --network host -e DISPLAY=$DISPLAY --restart always \
    -v /tmp/.X11-unix/:/tmp/.X11-unix \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -v /dev:/dev \
    -e QT_X11_NO_MITSHM=1 \
    --group-add=dialout \
    --group-add=video \
    --group-add=tty \
    --tty=true \
    --device=/dev/ttyUSB0 \
    --device=/dev/ttyTHS0 \
    -v ${WORKSPACE_DIR}:/root/shared_volume \
    --workdir="/root" \
    --name=${CONTAINER_NAME} \
    --privileged \
    ${DOCKER_REPO} \
    bash -c "${CMD}"
fi
