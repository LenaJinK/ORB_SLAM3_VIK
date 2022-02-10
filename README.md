## 编译ORB3:

```bash
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

## 增加ROS接口:
> ros_rgbd_inertial_kinematic.cc

1. Add the path including *Examples/ROS/ORB_SLAM3* to the ROS_PACKAGE_PATH environment variable. Open .bashrc file:

  ```bash
gedit ~/.bashrc
  ```

and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM3:

  ```bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM3/Examples/ROS
  ```

remember update source !!!!

2. Execute `build_ros.sh` script:

  ```bash
chmod +x build_ros.sh
./build_ros.sh
  ```

## 1.0版本新增:
1. 运动学模块

2. 新yaml参数文件 c_i_640480.yaml


