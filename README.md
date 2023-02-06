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

- Support ubran-nav datasets;

- Support kitti datasets.

Blog：[LVI-SAM：配置环境、安装测试、适配自己采集数据集](https://blog.csdn.net/qq_42938987/article/details/126005082?spm=1001.2014.3001.5501)

Video：[unkown]()

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
- If you want to use lviorf with integrated gps factor in kitti dataset, you can use the modified python script in "config/doc/kitti2bag" to obtain high-frequency gps data(Rate: 100HZ, Topic: '/gps/fix/correct'). About how to use "kitti2bag.py", please refer to [doc/kitti2bag](https://github.com/TixiaoShan/LIO-SAM/tree/master/config/doc/kitti2bag).

- For more details, please check the demo video: [unkown]()

## Mapping
  1. lvi-sam dataset

  2. kitti dataset

  3. ubran_hongkong dataset

  4. my data

## Performance
  1. Kitti 05

## TODO
- [ ] Add performance comparison;
- [ ] Add visual loop.

## Acknowledgments
  Thanks for [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM), [FAST_LIO2](https://github.com/hku-mars/FAST_LIO) and [UrbanNavDataset](https://github.com/weisongwen/UrbanNavDataset).

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