#include "ros/ros.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <ros/package.h>


#include <chrono>
#include <ctime>
#include <sstream>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/correspondence.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include "dislam_msgs/SubMap.h"

namespace fs = std::filesystem;

using namespace std;

std::string getCurrentTimeString()
{
  auto now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);

  std::tm *parts = std::localtime(&now_c);

  char buffer[100];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%d-%H-%M-%S", parts);
  return std::string(buffer);
}

class LIO_Pub
{
public:
  LIO_Pub(ros::NodeHandle &n);
  ~LIO_Pub();

private:
  double dis_th;
  int signal_num = 0;
  bool isFirstOdom = true;
  bool Signal = false;
  float distance = 0;
  float x0, y0, z0;
  float x1, y1, z1;
  string NameSpace;
  string SensorName;

  ros::Subscriber odometrySubscriber_;
  ros::Subscriber pointCloudSubscriber_;
  ros::Publisher pointCloudPublisher_;
  ros::Publisher signalPublisher_;
  ros::Publisher subMapPublisher_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr registeredCloud;

  std::ofstream tum_file_;
  std::string save_bin_folder_;
  int bin_file_index_ = 0;

  void OdomCallback(nav_msgs::Odometry msg);
  void PCCallback(sensor_msgs::PointCloud2 msg);
  void pub_Signal();
  void pub_PC(sensor_msgs::PointCloud2 msg);
  void pub_TF(nav_msgs::Odometry msg);
  void reformatRobotName();
  Eigen::Isometry3d odom2isometry(const nav_msgs::Odometry odom_msg);
  void savePointCloudAsBin(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const std::string &file_path);
};

LIO_Pub::~LIO_Pub()
{
  if (tum_file_.is_open())
    tum_file_.close();
  ROS_WARN("Good Bye!!!");
}

LIO_Pub::LIO_Pub(ros::NodeHandle &n)
{
  n.getParam("dis_th", dis_th);
  n.getParam("NameSpace", NameSpace);
  n.getParam("SensorName", SensorName);

  ROS_INFO("Get param dis_th = %lf", dis_th);

  reformatRobotName();

  registeredCloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

  // 获取当前包路径
  std::string package_path = ros::package::getPath("vehicle_simulator");

  // 获取时间戳
  std::string time_string = getCurrentTimeString();

  // 拼接保存路径
  save_bin_folder_ = package_path + "/submaps/" + NameSpace + "_submap_bin_" + time_string;
  if (!fs::exists(save_bin_folder_))
  {
    fs::create_directory(save_bin_folder_);
    ROS_INFO("Created bin folder: %s", save_bin_folder_.c_str());
  }
  else
  {
    ROS_INFO("Bin folder exists: %s", save_bin_folder_.c_str());
  }

  // 拼接TUM文件路径
  std::string tum_file_name = package_path + "/submap_poses/" + NameSpace + "_submap_pose_" + time_string + ".txt";
  tum_file_.open(tum_file_name.c_str(), std::ios::out | std::ios::trunc);
  if (!tum_file_.is_open())
  {
    ROS_ERROR("Failed to open TUM file: %s", tum_file_name.c_str());
  }
  else
  {
    ROS_INFO("Open TUM file for writing: %s", tum_file_name.c_str());
  }

  odometrySubscriber_ = n.subscribe(NameSpace + "/state_estimation", 1, &LIO_Pub::OdomCallback, this);
  pointCloudSubscriber_ = n.subscribe(NameSpace + "/registered_scan", 1, &LIO_Pub::PCCallback, this);

  pointCloudPublisher_ = n.advertise<sensor_msgs::PointCloud2>(NameSpace + "/merged_cloud_registered", 5);
  signalPublisher_ = n.advertise<std_msgs::Bool>(NameSpace + "/new_keyframe", 5);
  subMapPublisher_ = n.advertise<dislam_msgs::SubMap>(NameSpace + "/submap", 5);
}

void LIO_Pub::reformatRobotName()
{
  if ((NameSpace.find("/")) == string::npos && !NameSpace.empty())
    NameSpace = "/" + NameSpace;
  if ((SensorName.find("/")) == string::npos && !SensorName.empty())
    SensorName = "/" + SensorName;
}

void LIO_Pub::OdomCallback(nav_msgs::Odometry msg)
{
  if (isFirstOdom)
  {
    x0 = msg.pose.pose.position.x;
    y0 = msg.pose.pose.position.y;
    z0 = msg.pose.pose.position.z;
    isFirstOdom = false;
  }

  x1 = msg.pose.pose.position.x;
  y1 = msg.pose.pose.position.y;
  z1 = msg.pose.pose.position.z;

  distance = sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2) + pow(z1 - z0, 2));

  pub_TF(msg);

  if (distance > dis_th)
  {
    x0 = x1;
    y0 = y1;
    z0 = z1;

    signal_num++;
    Signal = true;
    pub_Signal();
    Signal = false;
    distance = 0;

    double timestamp = msg.header.stamp.toSec();
    double tx = msg.pose.pose.position.x;
    double ty = msg.pose.pose.position.y;
    double tz = msg.pose.pose.position.z;
    double qx = msg.pose.pose.orientation.x;
    double qy = msg.pose.pose.orientation.y;
    double qz = msg.pose.pose.orientation.z;
    double qw = msg.pose.pose.orientation.w;

    if (tum_file_.is_open())
    {
      tum_file_ << std::fixed << std::setprecision(6)
                << timestamp << " "
                << tx << " " << ty << " " << tz << " "
                << qx << " " << qy << " " << qz << " " << qw << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr registeredCloudBody(new pcl::PointCloud<pcl::PointXYZI>);
    Eigen::Isometry3d transform = odom2isometry(msg);
    Eigen::Matrix4d transformMatrix = transform.inverse().matrix();
    pcl::transformPointCloud(*registeredCloud, *registeredCloudBody, transformMatrix);

    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    float leafSize = 0.2;
    voxel.setInputCloud(registeredCloudBody);
    voxel.setLeafSize(leafSize, leafSize, leafSize);
    voxel.filter(*registeredCloudBody);

    while (registeredCloudBody->size() > 15000)
    {
      voxel.filter(*registeredCloudBody);
      leafSize += 0.1;
      voxel.setLeafSize(leafSize, leafSize, leafSize);
      voxel.setInputCloud(registeredCloudBody);
      voxel.filter(*registeredCloudBody);
    }

    char filename[256];
    sprintf(filename, "%s/%06d.bin", save_bin_folder_.c_str(), bin_file_index_);
    savePointCloudAsBin(registeredCloudBody, std::string(filename));
    bin_file_index_++;

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*registeredCloudBody, output);

    std::string tmpNameSpace = NameSpace[0] == '/' ? NameSpace.substr(1) : NameSpace;
    output.header.frame_id = tmpNameSpace + SensorName;

    dislam_msgs::SubMap submapMsg;
    output.header.stamp = msg.header.stamp;
    submapMsg.keyframePC = output;

    submapMsg.pose.position.x = tx;
    submapMsg.pose.position.y = ty;
    submapMsg.pose.position.z = tz;
    submapMsg.pose.orientation.x = qx;
    submapMsg.pose.orientation.y = qy;
    submapMsg.pose.orientation.z = qz;
    submapMsg.pose.orientation.w = qw;

    subMapPublisher_.publish(submapMsg);
    pointCloudPublisher_.publish(output);
    registeredCloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
  }
}

void LIO_Pub::PCCallback(sensor_msgs::PointCloud2 msg)
{
  pcl::PCLPointCloud2 registeredCloudPCL;
  pcl_conversions::toPCL(msg, registeredCloudPCL);
  pcl::PointCloud<pcl::PointXYZI>::Ptr localRegisteredCloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromPCLPointCloud2(registeredCloudPCL, *localRegisteredCloud);

  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setInputCloud(localRegisteredCloud);
  voxel.setLeafSize(0.2, 0.2, 0.2);
  voxel.filter(*localRegisteredCloud);

  *registeredCloud += *localRegisteredCloud;
}

void LIO_Pub::pub_Signal()
{
  std_msgs::Bool signal;
  signal.data = Signal;
  signalPublisher_.publish(signal);
}

void LIO_Pub::pub_TF(nav_msgs::Odometry msg)
{
  static tf::TransformBroadcaster tf_broadcaster;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z));
  q.setValue(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  transform.setRotation(q);
  usleep(100);
  tf_broadcaster.sendTransform(tf::StampedTransform(transform, msg.header.stamp, NameSpace + "/odom", NameSpace + SensorName));
}

Eigen::Isometry3d LIO_Pub::odom2isometry(const nav_msgs::Odometry odom_msg)
{
  const auto &orientation = odom_msg.pose.pose.orientation;
  const auto &position = odom_msg.pose.pose.position;

  Eigen::Quaterniond quat(orientation.w, orientation.x, orientation.y, orientation.z);
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.linear() = quat.toRotationMatrix();
  isometry.translation() = Eigen::Vector3d(position.x, position.y, position.z);

  return isometry;
}

void LIO_Pub::savePointCloudAsBin(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const std::string &file_path)
{
  std::ofstream ofs(file_path, std::ios::out | std::ios::binary);
  if (!ofs.is_open())
  {
    ROS_ERROR("Failed to open bin file: %s", file_path.c_str());
    return;
  }

  for (const auto &point : cloud->points)
  {
    ofs.write(reinterpret_cast<const char *>(&point.x), sizeof(float));
    ofs.write(reinterpret_cast<const char *>(&point.y), sizeof(float));
    ofs.write(reinterpret_cast<const char *>(&point.z), sizeof(float));
    ofs.write(reinterpret_cast<const char *>(&point.intensity), sizeof(float));
  }

  ofs.close();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LIO_Publisher");
  ros::NodeHandle n("~");
  LIO_Pub LIO_Pub(n);
  ROS_INFO("Init well!");
  ros::spin();
  return 0;
}
