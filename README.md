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
