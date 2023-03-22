# New Feature
Lviorf is a lidar-visual-inertial odometry and mapping system, which modified based on [lvi-sam](https://github.com/TixiaoShan/LVI-SAM) and [liorf](https://github.com/YJZLuckyBoy/liorf). The lviorf has the following new features:

------------------- Update Date: 2023-02-06 -------------------
- Removed the feature extraction module, making it easier to adapt to different lidars;
  
- Support 'robosense' lidar and Mulran datasets, make the following changes in "*.yaml":
  - sensor: “robosense” or sensor: “mulran”

- Support 6-axis IMU, make the following changes in "*.yaml":
  - imuType: 0 # 0: 6-axis IMU, 1: 9-axis IMU

- Support low frequency IMU（50HZ、100HZ）, make the following changes in "*.yaml":
  - imuRate: 500

- Re-derivation the LM optimization, don't need coordinate transformation;

- Modified gps factor, no longer depending on the 'robot_localization' package, and make it easier to adapt to different gnss device(RTK/GPS). In theoretically, lviorf is easier to adapt to gnss devices with different frequencies(10HZ~500HZ).

- The original version of lvi-sam sets many sensor external parameters to fixed values in the code, lviorf extracts the external parameters into the yaml file, making it easier to configure;

- The original version of lvi-sam dose not consider the translation amount between lidar and camera, this version adds the translation amount;

- In the lio system, you can choose whether to add visual loop closure constraint by changing the yaml file:
  ```
  visualLoopClosureEnableFlag: true     # visual loop closure
  loopClosureEnableFlag: true           # RS loop closure
  ```
- Support UrbanNav datasets;

- Support M2DGR datasets;

- Support kitti datasets.

Blog：[LVI-SAM：配置环境、安装测试、适配自己采集数据集](https://blog.csdn.net/qq_42938987/article/details/126005082?spm=1001.2014.3001.5501)

Video：[基于LIO-SAM框架SLAM算法开发（八）：建图之雷达视觉惯性融合建图-lviorf](https://www.bilibili.com/video/BV1ce4y1w7ND/?share_source=copy_web&vd_source=dee7afd16d8b7115a533915be5690f55)

## Run the package
1. compile package
  ```
    mkdir -p ~/lviorf/src
    cd ~/lviorf/src
    git clone https://github.com/YJZLuckyBoy/lviorf.git
    cd ..
    catkin_make -j8
  ```

2. Run the launch file:
  ```
    roslaunch lviorf run_kitti.launch
  ```

3. Play bag files:
  ```
    rosbag play kitti_2011_09_30_drive_0018_synced.bag
  ```

## For fusion gps factor
- Make sure your gnss topic type is 'sensor_msgs::NavSatFix'. If the gps data contains covariance information, ensure that only the data with small covariance is used;

- Modify 'gpsTopic' paramter in '*.yaml' with yourself gnss topic;
  ```
    gpsTopic: "gps/fix"    # GPS topic
  ```
- If you want to use lviorf with integrated gps factor in kitti dataset, you can use the modified python script in "config/doc/kitti2bag" to obtain high-frequency gps data(Rate: 100HZ, Topic: '/gps/fix/correct'). About how to use "kitti2bag.py", please refer to [doc/kitti2bag](https://github.com/TixiaoShan/LIO-SAM/tree/master/config/doc/kitti2bag);

- The synchronized bag can be downloaded from Google Driver. [kitti_2011_09_30_drive_0018_synced.bag](https://drive.google.com/file/d/1uLSVayprhJcTqvY5q8lgRoWl8nNjkWAM/view?usp=share_link)

## How to run your own data
With lviorf, you can easily run your own dataset, and only modify the following parameters for the yaml file:
1. *_camera.yaml
```
imu_topic: "/imu/data"               # IMU Topic
image_topic: "/camera/image_color"   # Camera Topic

# camera model
model_type: PINHOLE
camera_name: camera
image_width: 1920
image_height: 1200
distortion_parameters:
   k1: -0.109203
   k2: 0.063536
   p1: -0.003427
   p2: -0.000629
projection_parameters:
   fx: 1086.160899
   fy: 1090.242963
   cx: 940.067502
   cy: 586.740077

#imu parameters       The more accurate parameters you provide, the worse performance
acc_n: 8.1330537434371481e-03         # accelerometer measurement noise standard deviation.
gyr_n: 7.4266825125507141e-03         # gyroscope measurement noise standard deviation.
acc_w: 1.2123362494392119e-04        # accelerometer bias random work noise standard deviation.
gyr_w: 8.6572985145653080e-05       # gyroscope bias random work noise standard deviation.

#Rotation from camera frame to imu frame, imu^R_cam
extrinsicRotation: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 9.9934608718980233e-01, -1.5715484428488590e-02,-3.2564114721728155e-02, 
           3.2359037356803094e-02, -1.3131917124154624e-02,9.9939003669937865e-01, 
           -1.6133527815482926e-02, -9.9979026615676858e-01,-1.2614792047622947e-02]

#Translation from camera frame to imu frame, imu^T_cam
extrinsicTranslation: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [-1.7434527332030676e-02, 1.7171139776467173e-01, -4.5251036141047592e-02]
```
2. *_lidar.yaml
```
  pointCloudTopic: "/velodyne_points"           # Lidar Topic
  imuTopic: "/imu/data"                         # IMU Topic

  # Sensor Settings
  sensor: velodyne           # lidar sensor type, 'velodyne' or 'ouster' or 'livox' or 'robosense'
  N_SCAN: 32                 # number of lidar channel (i.e., Velodyne/Ouster: 16, 32, 64, 128, Livox Horizon: 6)
  Horizon_SCAN: 2000         # lidar horizontal resolution (Velodyne:1800, Ouster:512,1024,2048, Livox Horizon: 4000)
  
  # IMU Settings
  imuType: 0                                  # 0: 6-axis  1: 9-axis
  imuRate: 100.0
  imuAccNoise: 8.1330537434371481e-03
  imuGyrNoise: 7.4266825125507141e-03
  imuAccBiasN: 1.2123362494392119e-04
  imuGyrBiasN: 8.6572985145653080e-05

  # Extrinsics: T_lb (lidar -> imu)
  extrinsicTrans: [0.0, 0.0, 0.0]
  extrinsicRot: [1, 0, 0,
                  0, 1, 0,
                  0, 0, 1]

  # This parameter is set only when the 9-axis IMU is used, but it must be a high-precision IMU. e.g. MTI-680
  extrinsicRPY: [0, -1, 0,
                 1, 0, 0,
                 0, 0, 1]
```
## Mapping
  1. lvi-sam dataset
  <p align='center'>
      <img src="https://github.com/YJZLuckyBoy/pic/blob/master/lviorf/lvi-sam/lvi-sam-dataset.gif" alt="drawing" width="800" height = "400"/>
  </p>
  2. kitti-05 dataset
  <p align='center'>
      <img src="https://github.com/YJZLuckyBoy/pic/blob/master/lviorf/Kitti/Kitti05.gif" alt="drawing" width="800" height = "400"/>
  </p>
  3. UrbanNav dataset
  <p align='center'>
      <img src="https://github.com/YJZLuckyBoy/pic/blob/master/lviorf/UrbanNav/UrbanNavDataset.gif" alt="drawing" width="800" height = "400"/>
  </p>
  4. M2DGR dataset
  <p align='center'>
      <img src="https://github.com/YJZLuckyBoy/pic/blob/master/lviorf/M2DGR/gate_01.gif" alt="drawing" width="800" height = "400"/>
  </p>
  5. R3live dataset
  <p align='center'>
      <img src="https://github.com/YJZLuckyBoy/pic/blob/master/lviorf/R3live/hku_main_building.gif" alt="drawing" width="800" height = "400"/>
  </p>
  6. My dataset
  <p align='center'>
      <img src="https://github.com/YJZLuckyBoy/pic/blob/master/lviorf/Mydata/device00.png" alt="drawing" width="400"/>
      <img src="https://github.com/YJZLuckyBoy/pic/blob/master/lviorf/Mydata/device01.png" alt="drawing" width="400"/>
  </p>

## Performance
  1. Kitti 05

## TODO
- [ ] Add performance comparison;

## Acknowledgments
  Thanks for [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM), [FAST_LIO2](https://github.com/hku-mars/FAST_LIO), [M2DGR](https://github.com/SJTU-ViSYS/M2DGR) and [UrbanNavDataset](https://github.com/weisongwen/UrbanNavDataset).

# LVI-SAM

This repository contains code for a lidar-visual-inertial odometry and mapping system, which combines the advantages of [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM/tree/a246c960e3fca52b989abf888c8cf1fae25b7c25) and [Vins-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) at a system level.

<p align='center'>
    <img src="./doc/demo.gif" alt="drawing" width="800"/>
</p>

---

## Dependency

- [ROS](http://wiki.ros.org/ROS/Installation) (Tested with kinetic and melodic)
- [gtsam](https://gtsam.org/get_started/) (Georgia Tech Smoothing and Mapping library)
  ```
  sudo add-apt-repository ppa:borglab/gtsam-release-4.0
  sudo apt install libgtsam-dev libgtsam-unstable-dev
  ```
- [Ceres](https://github.com/ceres-solver/ceres-solver/releases) (C++ library for modeling and solving large, complicated optimization problems)
  ```
  sudo apt-get install -y libgoogle-glog-dev
  sudo apt-get install -y libatlas-base-dev
  wget -O ~/Downloads/ceres.zip https://github.com/ceres-solver/ceres-solver/archive/1.14.0.zip
  cd ~/Downloads/ && unzip ceres.zip -d ~/Downloads/
  cd ~/Downloads/ceres-solver-1.14.0
  mkdir ceres-bin && cd ceres-bin
  cmake ..
  sudo make install -j4
  ```

### Getting start with Docker  

When you use Docker, you could solve the dependency at once.  
For more information, you can check [docker_start.md](./docker/docker_start.md).    

---

## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/TixiaoShan/LVI-SAM.git
cd ..
catkin_make
```

---

## Datasets

<p align='center'>
    <img src="./doc/sensor.jpeg" alt="drawing" width="600"/>
</p>

The datasets used in the paper can be downloaded from Google Drive. The data-gathering sensor suite includes: Velodyne VLP-16 lidar, FLIR BFS-U3-04S2M-CS camera, MicroStrain 3DM-GX5-25 IMU, and Reach RS+ GPS.

```
https://drive.google.com/drive/folders/1q2NZnsgNmezFemoxhHnrDnp1JV_bqrgV?usp=sharing
```

**Note** that the images in the provided bag files are in compressed format. So a decompression command is added at the last line of ```launch/module_sam.launch```. If your own bag records the raw image data, please comment this line out.

<p align='center'>
    <img src="./doc/jackal-earth.png" alt="drawing" width="286.5"/>
    <img src="./doc/handheld-earth.png" alt="drawing" width="328"/>
</p>

---

## Run the package

1. Configure parameters:

```
Configure sensor parameters in the .yaml files in the ```config``` folder.
```

2. Run the launch file:
```
roslaunch lvi_sam run.launch
```

3. Play existing bag files:
```
rosbag play handheld.bag 
```

---

## Related Packages

  - [LVI_SAM_fixed by epicjung](https://github.com/epicjung/LVI_SAM_fixed)
  - [LVI-SAM-modified by skyrim835](https://github.com/skyrim835/LVI-SAM-modified)

---

## TODO

  - [ ] Update graph optimization using all three factors in imuPreintegration.cpp, simplify mapOptimization.cpp, increase system stability 

---

## Paper 

Thank you for citing our [paper](./doc/paper.pdf) if you use any of this code or datasets.

```
@inproceedings{lvisam2021shan,
  title={LVI-SAM: Tightly-coupled Lidar-Visual-Inertial Odometry via Smoothing and Mapping},
  author={Shan, Tixiao and Englot, Brendan and Ratti, Carlo and Rus Daniela},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
  pages={5692-5698},
  year={2021},
  organization={IEEE}
}
```

---

## Acknowledgement

  - The visual-inertial odometry module is adapted from [Vins-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono).
  - The lidar-inertial odometry module is adapted from [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM/tree/a246c960e3fca52b989abf888c8cf1fae25b7c25).