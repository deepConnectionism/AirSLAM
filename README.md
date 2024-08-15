### 下载代码：

1. 克隆代码到本地：
```bash
mkdir -p /home/h/docker_workspace/airslam_ws/src
cd /home/h/docker_workspace/airslam_ws/src
git clone https://github.com/xukuanhit/AirSLAM.git
```

### Docker (Recommend)
```bash
docker pull xukuanhit/air_slam:v4
docker run -it --env DISPLAY=$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix --privileged --gpus all --volume /home/h/docker_workspace:/workspace --workdir /workspace --name air_slam xukuanhit/air_slam:v4 /bin/bash
```



