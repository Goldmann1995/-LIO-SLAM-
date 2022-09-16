# IMU雷达内外参标定

## IMU内参标定
### 安装

这个项目使用 [imu_utils](https://github.com/gaowenliang/imu_utils) 和 [code_utils](https://github.com/gaowenliang/code_utils)。请确保你本地安装了它们。

安装依赖：

```sh
$ sudo apt-get install libdw-dev

code_utils 依赖 ceres库， 需要先全局安装ceres库；
imu_utils 依赖 code_utils，故先catkin_make code_utils, 再将imu_utils放入编译。
```
#### 编译报错解决：

编译code_utils 报错， fatal error: backward.hpp: No such file or directory

解决办法： 在code_utils文件夹下面找到sumpixel_test.cpp，修改#include “backward.hpp” 为 #include“code_utils/backward.hpp”即可编译成功。

然后将imu_utils放入再编译一次。


### 标定方式


```sh
$ roslaunch fdilink_ahrs ahrs_data.launch //运行IMU 注意IMU的topic

$ rosbag record -o imu.bag /imu_raw   	 //注意bag现在保存在当前目录下，和IMUtopic为/imu_raw

$ roslaunch imu_utils kalibieren.launch  //增加imu标定包的launch文件如下。 在/imu_utils/launch下创建


<launch>
    <node pkg="imu_utils" type="imu_an" name="imu_an" output="screen">
        <param name="imu_topic" type="string" value= "imu_raw"/> //这里的value是你的imu的topic
        <param name="imu_name" type="string" value= "wheeltec"/>//这里的value是你的imu名字自定义
        <param name="data_save_path" type="string" value= "$(find imu_utils)/data/"/>
        <param name="max_time_min" type="int" value= "120"/> //这里的value值是你录制的imu的数据包的时间长度，这个value值一定要小于你录制的imu数据包
        <param name="max_cluster" type="int" value= "100"/>
    </node>
</launch>


$ rosbag play -r 200 imu.bag 

注：标定时间根据自己设备运行时间来设定，如果设备一般是运行2个小时，那么就标定2个小时即可。
    输出文件在/imu_utils/data文件夹中，我们只需要加速度和角速度的平均噪音和偏置 avg-axis中的gry_n,gry_w,acc_n,acc_w，不需要每个方向的。


```


### 相关仓库

- [知乎](https://zhuanlan.zhihu.com/p/434710744) — 十四.激光和惯导LIO-SLAM框架学习之惯导内参标定
- [CSDN](https://blog.csdn.net/er_dan_love/article/details/124370788) — 利用 imu_utils 标定 imu


## IMU和Lidar外参标定


即确认惯导与雷达的旋转变换矩阵

平移向量影响不是很大，直接从结构图纸中得到即可，即是惯导原点到雷达原点的向量。旋转矩阵对建图影响比较大，首先我们得确认理论的旋转矩阵，然后再进行标定。

确认理论的旋转矩阵的方法：

将imageProjection.cpp里这个回调函数中的注释打开。

确认惯导的输出是否正常，如果Z轴的加速度是负数，则Z轴反过来了，查看

LIO-SAM中config文件夹中的配置yaml文件extrinsicRot 和extrinsicRPY。

这两个矩阵需要做调整，根据与视频的现象进行的对比，这里发现单位阵是可以达到Z轴加速度是正数9.8的（重力加速度）。

这里就已经确定了Z轴，我们暂时无法确定X轴和Y轴的朝向，可以参考后面标定出来的旋转矩阵。

其中extrinsicRot表示imu->lidar的坐标变换, 用于旋转imu坐标系下的加速度和角速度到lidar坐标系下, extrinsicRPY则用于旋转imu坐标系下的欧拉角到lidar坐标下, 由于lio-sam作者使用的imu的欧拉角旋转方向与lidar坐标系不一致（即是按照什么旋转顺序旋转）, 因此使用了两个旋转不同, 但是大部分的设备两个旋转应该是设置为相同的，我们这里也是设置为相同即可。

### 安装

这个项目使用 [nlopt](https://github.com/stevengj/nlopt) 和 [lidar_align](https://github.com/ethz-asl/lidar_align)。请确保你本地安装了它们。
github下载nlopt 并cmake编译.
github下载源码进行catkin_make编译

#### 编译报错解决：

编译时出现Could not find NLOPTConfig.cmake

解决办法：在nlopt目录下找到这个NLOPTConfig.cmake文件并将其放入到lidar_align工程目录下，并在CMakeLists.txt里加上这样一句话：

list(APPEND CMAKE_FIND_ROOT_PATH ${PROJECT_SOURCE_DIR})

由于这个标定软件没有IMU的数据接口,报错 error: conflicting declaration ‘typedef struct LZ4_streamDecode_t LZ4_streamDecode_t
所以改写loader.cc的include 加入
'''
#include <sensor_msgs/Imu.h>

并且 建立软链接

$ sudo mv /usr/include/flann/ext/lz4.h /usr/include/flann/ext/lz4.h.bak

$ sudo mv /usr/include/flann/ext/lz4hc.h /usr/include/flann/ext/lz4.h.bak
 
$ sudo ln -s /usr/include/lz4.h /usr/include/flann/ext/lz4.h

$ sudo ln -s /usr/include/lz4hc.h /usr/include/flann/ext/lz4hc.h
'''

再次catkin_make
### 标定方式(IMU和Lidar同时标定外参)

录制话题数据，旋转三个轴，XY轴不要大幅度旋转 注意有小运动

rosbag record -o lidar.bag out /velodyne_points /imu_raw 

修改标定软件包launch文件中的数据包路径，然后运行launch文件，等待漫长迭代优化时间。

```sh

<?xml version="1.0" encoding="UTF-8"?>
<launch>

  <arg name="bag_file" default="/PATH/TO/YOUR.bag"/> //bag的路径,只要修改这里
  <arg name="transforms_from_csv" default="false"/>
  <arg name="csv_file" default="PATH/TO/YOUR.csv"/> 

  <node pkg="lidar_align" name="lidar_align" type="lidar_align_node" output="screen">
    <param name="input_bag_path" value="$(arg bag_file)" />
    <param name="input_csv_path" value="$(arg csv_file)" />
    <param name="output_pointcloud_path" value="$(find lidar_align)/results/$(anon lidar_points).ply" />
    <param name="output_calibration_path" value="$(find lidar_align)/results/$(anon calibration).txt" />
    <param name="transforms_from_csv" value="$(arg transforms_from_csv)"/>
  </node>

</launch>
$ rosbag play lidar.bag

$ roslaunch lidar_align lidar_align.launch

```

最后输出数据：
这里发现标定矩阵类似于单位阵，说明单位阵是理论外参，而标定后的矩阵是考虑了小角度误差后的外参。

把旋转矩阵复制到LIO-SAM中config文件夹中的配置yaml文件中，更改这  extrinsicRot: [-1, 0, 0, 0, 1, 0, 0, 0, -1]
  extrinsicRPY: [0, 1, 0, -1, 0, 0, 0, 0, 1]
两个矩阵。extrinsicRot 和 extrinsicRPY 这里设置为一致。


### 相关仓库

- [古月居](https://guyuehome.com/35276) — 一种lidar和里程计标定的方法介绍--lidar_align
- [知乎](https://zhuanlan.zhihu.com/p/434718435) — 十五.激光和惯导LIO-SLAM框架学习之惯导与雷达外参标定（1）

