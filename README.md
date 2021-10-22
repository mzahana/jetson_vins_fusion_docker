# jetson_vins_fusion_docker
This repository provides Docker files and scripts to easily setup and run [VINS-FUSION-gpu](https://github.com/pjrambo/VINS-Fusion-gpu) on NVIDIA jetson boards inside a docker container. 

**This is tested on Xavier NX with Jetpack 4.4 [L4T 32.4.3]**

This repository is inspired by [jteson-containers](https://github.com/dusty-nv/jetson-containers) and [vins-fusion-gpu-tx2-nano](https://github.com/arjunskumar/vins-fusion-gpu-tx2-nano)

# Docker image
The Docker image `Docker.vins.gpu` contains
* Ubuntu 18
* ROS melodic 
* VINS-FUSION-gpu
* MAVROS/MAVLink
* Realsense SDK
* `realsense-ros` package
* [`jetson_vins_fusion_scripts`](https://github.com/mzahana/jetson_vins_fusion_scripts)

# Installtion steps

## Hardware
* It's recommended to use Xavier NX with SSD drive for best performance. Check [this video](https://www.youtube.com/watch?v=ZK5FYhoJqIg&t=327s) to see how to use Xavier NX image from the SSD drive

## Docker Default Runtime

To enable access to the CUDA compiler (nvcc) during `docker build` operations, add `"default-runtime": "nvidia"` to your `/etc/docker/daemon.json` configuration file before attempting to build the containers:

``` json
{
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },

    "default-runtime": "nvidia"
}
```

You will then want to restart the Docker service or reboot your system before proceeding.

## Building the docker image
* Open a terminal, and execute the folllwing commands
    ```bash
    # Create a directory to clone this package into
    mkdir -p $HOME/src && cd $HOME/src/
    # Clone this package
    git clone https://github.com/mzahana/jetson_vins_fusion_docker.git
    ```

* Build the `mzahana:vins_gpu` Docker image
    ```bash
    cd $HOME/src/jetson_vins_fusion_docker
    ./scripts/setup_jetson.sh
    ```
    You may need to provide passowrd for `sudo` when asked

* Once the image is built, you can verify that by listing Docker images `docker images`. You should see `mzahana:vins_gpu` availble in the listed images

* An alias will be added in the `~/.bashrc` for convenience. The alias is called `vins_container`. You can simply run the VINS container by executing `vins_container` in a terminal window

* Once the container is running, an interactive terminal (inside the container) will be running. 

# Running VINS nodes
* The `Docker.vins.gpu` image comes with a convenience ROS package, [jetson_vins_fusion_scripts](https://github.com/mzahana/jetson_vins_fusion_scripts), which has some launch files and configuration files for the VINS-FUSION-gpu package

* You can run the VINS node (loop_fusion, vins_node) using the following command
    ```bash
    roslaunch jetson_vins_fusion_scripts vins.launch 
    ```
    The `vins.launch` script runs two nodes, loop_fusion, and vins_node). To disable the loop_fusion,
    ```bash
    roslaunch jetson_vins_fusion_scripts vins.launch run_loop_fusion:=false
    ```

* Then you can either run a bag file in another terminal (you can log into the container again using `vins_container` alias)
    ```bash
    rosbag play bag_name.bag
    ```
    You can downlod one of the [EuROC bags](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets), inside the container using `wget` command

* The [jetson_vins_fusion_scripts](https://github.com/mzahana/jetson_vins_fusion_scripts) package also has example configuration files for D435i cameras to use with VINS-FUSION. You will need to calibrate the D435i IMU using [this document](https://github.com/arjunskumar/vins-fusion-gpu-tx2-nano/blob/master/docs/RealSense_Depth_D435i_IMU_Calib.pdf). Then you need to calbrate the stereo setup+IMU using [Kalibr](https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration)

* For easy calibration with Kalibr, a docker image is available [here](https://github.com/mzahana/kalibr/tree/master/docker)

* **NOTE** There is a shared folder between the container (container name is `vins_gpu`) and the host system (Xavier) in order to easily share files between the two. The shared folder is located in the home folder of the host (Xavier) under the name `vins_gpu_shared_volume`, and available inside the container's home folder under the name `shared_folder`

## Runing on Jetson Nano
Althouhg this package was not tested on Jetson Nano, in theory it should work. However, you need to make sure that you increase the swap size

**Add a swap of 4GB**
```bash
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo swapon --show
sudo cp /etc/fstab /etc/fstab.bak
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab 
```
**To lock Jetson Nano at its maximum frequency and power mode by running the following commands:**
```bash
sudo jetson_clocks
sudo nvpmodel -m 0 
```
# Using VINS with cameras
 
## Intel Realsense D435i
* Install the docker image as described above
* Calibrate the D435i IMU as described [here](https://github.com/arjunskumar/vins-fusion-gpu-tx2-nano/blob/master/docs/RealSense_Depth_D435i_IMU_Calib.pdf)
* Calibrate the D435i stereo infrared cameras
    * Prepare a [calibration target](https://github.com/ethz-asl/kalibr/wiki/calibration-targets). It is recommended to print an apriltag calibration target, you can download from [here](https://github.com/ethz-asl/kalibr/wiki/downloads)
    * Connect the D435i camera using the USB C cable
    * If you are using a PC, make sure to install Realsense SDK and [realsense-ros](https://github.com/IntelRealSense/realsense-ros) in order to run the D435i. Otherwise, you can run it directly in the docker container provided by this repo
    * Run the `rs_camera.launch` script
        ```bash
        roslaunch jetson_vins_fusion_scripts rs_camera.launch
        ```
    * Record a ROS bag with the D435i two infra camera topics
        ```bash
        rosbag record -o d43i_stereo_imu.bag /img1_tpoic /img2_topic
        ```
        **NOTE** Make sure to write the topic names correctly
    * You will need to print a calibration target (apriltag target is recommended, see [here](https://github.com/ethz-asl/kalibr/wiki/calibration-targets) )
    * Calibrate the D435i stereo camera using Kalibr nad the recorded rosbag. For convenience, a docker image with Kalibr is available [here](https://github.com/mzahana/kalibr/tree/master/docker)
    * Use Kalibr [camera-IMU calibration](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)
    * Use the calibration results from Kalibr to update VINS [configuratoin files](https://github.com/mzahana/jetson_vins_fusion_scripts/tree/main/config/d435i). Those configuration files would be available inside the `vins_gpu` container in `~/catkin_ws/jetson_vins_fusion_scripts/config/d435i`
* Inside the container terminal, tun `vins` nodes
```bash
roslaunch jetson_vins_fusion_scripts vins.launch config_file:=/root/catkin_ws/src/
jetson_vins_fusion_scripts/config/d435i/rs_stereo_imu_confg.yaml
```