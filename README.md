## 编译ORB3:

```bash
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

## 增加ROS接口:

1. `.bashrc`中添加ROS_PACKAGE_PATH

  ```bash
gedit ~/.bashrc 
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM3/Examples/ROS
  ```
修改完记得source一下。

2. 执行`build_ros.sh` :
  ```bash
chmod +x build_ros.sh
./build_ros.sh
  ```

## 1.0 版本新增修改:
1. 添加运动学模块，增加了rgbd_inertial_kinematic模式
2. 增加了VIK模式的ROS接口
3. 修改了yaml参数读取文件，添加了运动学外参

## 新增依赖
1. KDL \
   安装orocos kdl：https://www.orocos.org/wiki/Installation_Manual.html   (下载master branch就可以)\
   安装kdl_parser：https://github.com/ros/kdl_parser(下载默认的版本，可以正常安装，branch melodic无法正常安装)
   
## 注意事项
1. urdf文件路径
   如有变动，代码中对应位置需要修改，更新成自己的urdf文件路径

2. 参数文件有更新，最新版本为 c_i_640480.yaml
