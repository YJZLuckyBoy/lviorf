#include "parameters.h"

std::string PROJECT_NAME;

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string IMU_TOPIC;
double ROW, COL;
double TD, TR;

int USE_LIDAR;
int ALIGN_CAMERA_LIDAR_COORDINATE;

tf::Quaternion liextrinsicRot_;
Eigen::Vector3d liextrinsicTrans_;
tf::Transform lidar_to_imu_transform;
tf::Transform camera_to_ros_transform;
tf::Transform lidar_to_camera_transform;
tf::Transform imu_to_camera_transform;

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    n.getParam("vins_config_file", config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    // fsSettings["project_name"] >> PROJECT_NAME;
    PROJECT_NAME = "lviorf";
    std::string pkg_path = ros::package::getPath(PROJECT_NAME);

    fsSettings["imu_topic"] >> IMU_TOPIC;

    fsSettings["use_lidar"] >> USE_LIDAR;
    fsSettings["align_camera_lidar_estimation"] >> ALIGN_CAMERA_LIDAR_COORDINATE;

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("Image dimention: ROW: %f COL: %f ", ROW, COL);

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = pkg_path + "/config/extrinsic_parameter.csv";

    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_INFO(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = pkg_path + "/config/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_INFO(" Fix extrinsic param.");
        
        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
        ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());

        Eigen::Quaterniond icExtrinsicRotation_(eigen_R);
        imu_to_camera_transform = tf::Transform(tf::Quaternion(icExtrinsicRotation_.x(), icExtrinsicRotation_.y(), icExtrinsicRotation_.z(), icExtrinsicRotation_.w()), tf::Vector3(eigen_T[0], eigen_T[1], eigen_T[2]));
    } 

    // add by YJZ
    // lidar to camera extrinsic
    std::vector<double> extRotV, extTransV;
    std::string PROJECT_NAME = "lviorf";
    // n.param<std::string>("/PROJECT_NAME", PROJECT_NAME, "lvi_sam");
    n.param<std::vector<double>>(PROJECT_NAME+ "/extrinsicRot", extRotV, std::vector<double>());
    n.param<std::vector<double>>(PROJECT_NAME+ "/extrinsicTrans", extTransV, std::vector<double>());
    Eigen::Matrix3d extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
    Eigen::Vector3d extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);


    Eigen::Quaterniond liExtrinsicRotation_(extRot.block<3, 3>(0, 0));
    liextrinsicRot_.setW( liExtrinsicRotation_.w() );
    liextrinsicRot_.setX( liExtrinsicRotation_.x() );
    liextrinsicRot_.setY( liExtrinsicRotation_.y() );
    liextrinsicRot_.setZ( liExtrinsicRotation_.z() );

    liextrinsicTrans_ = extTrans;
    lidar_to_imu_transform = tf::Transform(liextrinsicRot_, tf::Vector3(liextrinsicTrans_[0], liextrinsicTrans_[1], liextrinsicTrans_[2]));

    cv::Mat cv_camera_2_ros_R;
    Eigen::Matrix3d eigen_camera_2_ros_R;
    fsSettings["cameraToROSStandard"] >> cv_camera_2_ros_R;
    cv::cv2eigen(cv_camera_2_ros_R, eigen_camera_2_ros_R);
    Eigen::Quaterniond cameraToROS_(eigen_camera_2_ros_R.block<3, 3>(0, 0));
    camera_to_ros_transform = tf::Transform(tf::Quaternion(cameraToROS_.x(), cameraToROS_.y(), cameraToROS_.z(), cameraToROS_.w()), tf::Vector3(0., 0., 0.));

    lidar_to_camera_transform = lidar_to_imu_transform * imu_to_camera_transform;

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }
    
    fsSettings.release();
    usleep(100);
}
