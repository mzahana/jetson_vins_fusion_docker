# jetson_vins_fusion_docker
This repository provides Docker files and scripts to easily setup and run [VINS-FUSION-gpu](https://github.com/pjrambo/VINS-Fusion-gpu) on NVIDIA jetson boards inside a docker container. 

**This is tested on Xavier NX with Jetpack 4.4 [L4T 32.4.3]**

This repository is inspired by [jteson-containers](https://github.com/dusty-nv/jetson-containers) and [vins-fusion-gpu-tx2-nano](https://github.com/arjunskumar/vins-fusion-gpu-tx2-nano)

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
    # Craete directory
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

* You can run the VINS node(s) using the following command
```bash
roslaunch jetson_vins_fusion_scripts vins.launch 
```

* Then you can either run a bag file in another terminal (you can log into the container again using `vins_container` alias)
```bash
rosbag play bag_name.bag
```
You can downlod one of the [EuROC bags(https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)]

* The [jetson_vins_fusion_scripts](https://github.com/mzahana/jetson_vins_fusion_scripts) package also has example configuration files for D435i cameras to use with VINS-FUSION. You will need to calibrate the D435i IMU using [this document](https://github.com/arjunskumar/vins-fusion-gpu-tx2-nano/blob/master/docs/RealSense_Depth_D435i_IMU_Calib.pdf). Then you need to calbrate the stereo setup+IMU using [Kalibr](https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration)

* For easy calibration with Kalibr, a docker image is available [here](https://github.com/mzahana/kalibr/tree/master/docker)