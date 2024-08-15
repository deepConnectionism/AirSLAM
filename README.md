Fork from https://github.com/sair-lab/AirSLAM

### 本地环境准备：

1. 克隆代码到本地：
```bash
mkdir -p /home/h/docker_workspace/airslam_ws/src
cd /home/h/docker_workspace/airslam_ws/src
git clone https://github.com/xukuanhit/AirSLAM.git
mkdir AirSLAM/debug
```

### Docker (Recommend)
```bash
docker pull xukuanhit/air_slam:v4
docker run -it --env DISPLAY=$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix --privileged --gpus all --volume /home/h/docker_workspace:/workspace --workdir /workspace --name air_slam_v4 xukuanhit/air_slam:v4 /bin/bash
cd airslam_ws/
catkin_make
source devel/setup.bash
```

### 魔改代码以支持 rosbag 输入

略，见代码

```bash
roslaunch air_slam vo_euroc_ros.launc
rosbag play ../datasets/euroc/MH_01_easy.bag
```

[euroc bag result](debug/2024-08-15_17.42-airslam-euroc-bag.mkv)


### Map Optimization
**1**: Change "map_root" in [MR launch file](launch/map_refinement/mr_euroc.launch) to your own map path.

**2**: Run the launch file:

```
roslaunch air_slam mr_euroc.launch 
```

