Dependencies
------------

```
apt install -y fuse3 python3-pyfuse3
```

Build
-----

```
colcon build ros2_runtime_fs
```

Usage
------

```
mkdir -p ~/run
ros2 launch ros2_runtime_fs ros2_runtime_fs.launch.xml
<Ctrl+Z>
bg
cd ~/run
cd graph
```

In case of a container
------

Docker:
```
--cap-add SYS_ADMIN
--device /dev/fuse
--security-opt apparmor:unconfined
```

Podman:
```
--device /dev/fuse:/dev/fuse
--privileged
```

Quick start
-----------

```
sudo podman pull docker.io/library/ros:rolling
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone git@github.com:gozhev/ros2_runtime_fs.git
sudo podman run --volume ~/devel/ros2_ws/:/opt/ros2_ws --volume /dev/fuse:/dev/fuse --privileged --rm -it docker.io/library/ros:rolling
apt install -y fuse3 python3-pyfuse3
cd /opt/ros2_ws
colcon build ros2_runtime_fs
. install/setup.bash
mkdir -p run
ros2_runtime_fs ./run &
cd ./run
```
