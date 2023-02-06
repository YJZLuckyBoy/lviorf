#include "parameters.h"

std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::string POINT_CLOUD_TOPIC;
std::string PROJECT_NAME;

std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;

tf::Quaternion liextrinsicRot_;
Eigen::Vector3d liextrinsicTrans_;
tf::Transform lidar_to_imu_transform;
tf::Transform camera_to_ros_transform;
tf::Transform lidar_to_camera_transform;
tf::Transform imu_to_camera_transform;
Eigen::Matrix3d camera_to_ros_matrix;

int USE_LIDAR;
int LIDAR_SKIP;


void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    n.getParam("vins_config_file", config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    // project name
    fsSettings["project_name"] >> PROJECT_NAME;
    std::string pkg_path = ros::package::getPath(PROJECT_NAME);

    // sensor topics
    fsSettings["image_topic"]       >> IMAGE_TOPIC;
    fsSettings["imu_topic"]         >> IMU_TOPIC;
    fsSettings["point_cloud_topic"] >> POINT_CLOUD_TOPIC;

    // lidar configurations
    fsSettings["use_lidar"] >> USE_LIDAR;
    fsSettings["lidar_skip"] >> LIDAR_SKIP;

    // feature and image settings
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];

    {
        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);

        Eigen::Quaterniond icExtrinsicRotation_(eigen_R.block<3, 3>(0, 0));
        imu_to_camera_transform = tf::Transform(tf::Quaternion(icExtrinsicRotation_.x(), icExtrinsicRotation_.y(), icExtrinsicRotation_.z(), icExtrinsicRotation_.w()), tf::Vector3(eigen_T[0], eigen_T[1], eigen_T[2]));

        // add by YJZ
        // lidar to camera extrinsic
        vector<double> extRotV, extTransV;
        std::string PROJECT_NAME = "lviorf";
        // n.param<std::string>("/PROJECT_NAME", PROJECT_NAME, "lvi_sam");
        n.param<vector<double>>(PROJECT_NAME+ "/extrinsicRot", extRotV, vector<double>());
        n.param<vector<double>>(PROJECT_NAME+ "/extrinsicTrans", extTransV, vector<double>());
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

        camera_to_ros_matrix = eigen_camera_2_ros_R;

        lidar_to_camera_transform = lidar_to_imu_transform * imu_to_camera_transform;
    }


    // fisheye mask
    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
    {
        std::string mask_name;
        fsSettings["fisheye_mask"] >> mask_name;
        FISHEYE_MASK = pkg_path + mask_name;
    }

    // camera config
    CAM_NAMES.push_back(config_file);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();
    usleep(100);
}

float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

void publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    if (thisPub->getNumSubscribers() == 0)
        return;
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    thisPub->publish(tempCloud); 
}
