# IMU雷达安装

## Lidar
### 安装

这个项目使用 [镭神智能_C16_V3.0_客户服务资料.zip](https://pan.baidu.com/s/1KxqLfY-BlWKelVQvH7wMWg) 提取码: hfy3 请确保你本地安装了它们。
压缩包在lslidar送的U盘中

### 安装方式


```sh
$ unzip 镭神智能_C16_V3.0_客户服务资料.zip 

$ cd    镭神智能_C16_V3.0_客户服务资料

$ unzip ROS1.zip  //注意此处修改了压缩包名称为ROS1，请选择Ubuntu 18 和16的压缩包

$ mkdir ~/Download/lidar_ws/src

$ sudo mv ROS1 ~/Download/lidar_ws/src

$ cd ~/Download/lidar_ws

$ catkin_make

$ source devel/setup.bash

$ roslaunch lslidar_c16_decoder lslidar_c16.launch 



```


## IMU

这个项目使用 [WHEELTEC N系列惯导资料V3.4_20220512.rar ](https://pan.baidu.com/s/1sqo4bE_OvGZuJT814iIw2w) 提取码: jf59 请确保你本地安装了它们。


```sh
$ unzip WHEELTEC N系列惯导资料V3.4_20220512.zip 

$ cd    WHEELTEC N系列惯导资料V3.4_20220512

$ mkdir ~/Download/IMU_ws/src

$ sudo mv ROS1 ~/Download/IMU_ws/src

$ cd ~/Download/IMU_ws

$ catkin_make

$ source devel/setup.bash

$ roslaunch fdilink_ahrs ahrs_data.launch 


```




