## 编译ORB3:

```bash
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

## 增加ROS接口:

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

## 新增修改:
1. 添加运动学模块，增加了rgbd_inertial_kinematic模式
2. 增加了VIK模式的ROS接口
3. 修改了yaml参数读取文件，添加了运动学外参

