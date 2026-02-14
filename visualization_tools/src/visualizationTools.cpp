#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

string metricFile;                    // 输出探索空间的各项参数的文件路径
string trajFile;                      // 输出探索轨迹的文件路径
string mapFile;                       // 输入地图信息文件的路径
double overallMapVoxelSize = 0.5;     // 表示总体地图体素的尺寸
double exploredAreaVoxelSize = 0.3;   // 表示探索空间体素的尺寸
double exploredVolumeVoxelSize = 0.5; // 表示探索体积提速的尺寸
double transInterval = 0.2;           // 表示探索时间间隔
double yawInterval = 10.0;            // 表示航向时间间隔
int overallMapDisplayInterval = 2;    // 全局地图显示时间间隔
int overallMapDisplayCount = 0;       // 全局地图显示计数
int exploredAreaDisplayInterval = 1;  // 探索空间显示时间间隔
int exploredAreaDisplayCount = 0;     // 探索空间显示计数

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());           // lidar点云数据
pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloud(new pcl::PointCloud<pcl::PointXYZ>());        // 全局地图点云数据
pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloudDwz(new pcl::PointCloud<pcl::PointXYZ>());     // 全局地图点云下采样数据
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud(new pcl::PointCloud<pcl::PointXYZI>());    // 探索空间点云数据
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud2(new pcl::PointCloud<pcl::PointXYZI>());   // 探索空间点云数据2
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud(new pcl::PointCloud<pcl::PointXYZI>());  // 探索空间体积点云
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud2(new pcl::PointCloud<pcl::PointXYZI>()); // 探索空间体积点云2
pcl::PointCloud<pcl::PointXYZI>::Ptr trajectory(new pcl::PointCloud<pcl::PointXYZI>());           // 轨迹

const int systemDelay = 5;      // 系统时间延时
int systemDelayCount = 0;       // 系统延时计数
bool systemDelayInited = false; // 系统延时初始化
double systemTime = 0;          // 系统时间
double systemInitTime = 0;      // 系统初始化时间
bool systemInited = false;      // 系统初始化标志

float vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float exploredVolume = 0, travelingDis = 0, runtime = 0, timeDuration = 0;

pcl::VoxelGrid<pcl::PointXYZ> overallMapDwzFilter;
pcl::VoxelGrid<pcl::PointXYZI> exploredAreaDwzFilter;
pcl::VoxelGrid<pcl::PointXYZI> exploredVolumeDwzFilter;

sensor_msgs::PointCloud2 overallMap2;

ros::Publisher *pubExploredAreaPtr = NULL;
ros::Publisher *pubTrajectoryPtr = NULL;
ros::Publisher *pubExploredVolumePtr = NULL;
ros::Publisher *pubTravelingDisPtr = NULL;
ros::Publisher *pubTimeDurationPtr = NULL;

FILE *metricFilePtr = NULL;
FILE *trajFilePtr = NULL;

void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom)
{
  systemTime = odom->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  // 控制航向角为[-pi, pi]之间
  float dYaw = fabs(yaw - vehicleYaw);
  if (dYaw > PI)
    dYaw = 2 * PI - dYaw;

  float dx = odom->pose.pose.position.x - vehicleX;
  float dy = odom->pose.pose.position.y - vehicleY;
  float dz = odom->pose.pose.position.z - vehicleZ;
  float dis = sqrt(dx * dx + dy * dy + dz * dz);

  if (!systemDelayInited)
  {
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;
    return;
  }

  if (systemInited)
  {
    timeDuration = systemTime - systemInitTime;

    std_msgs::Float32 timeDurationMsg;
    timeDurationMsg.data = timeDuration;
    pubTimeDurationPtr->publish(timeDurationMsg);
  }

  // 若车辆运动距离小于阈值或者角度小于阈值，则判定车辆没运动，结束回调
  if (dis < transInterval && dYaw < yawInterval)
  {
    return;
  }

  if (!systemInited)
  {
    dis = 0;
    systemInitTime = systemTime;
    systemInited = true;
  }

  travelingDis += dis;

  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;

  // 输出以下参数，具体路径看launch中关于trajFilePtr的取值
  fprintf(trajFilePtr, "%f %f %f %f %f %f %f\n", vehicleX, vehicleY, vehicleZ, roll, pitch, yaw, timeDuration);

  pcl::PointXYZI point;
  point.x = vehicleX;
  point.y = vehicleY;
  point.z = vehicleZ;
  point.intensity = travelingDis;
  // std::cout << "point.intensity = "<< point.intensity <<std::endl;
  trajectory->push_back(point);

  // 发布运动轨迹
  sensor_msgs::PointCloud2 trajectory2;
  pcl::toROSMsg(*trajectory, trajectory2);
  trajectory2.header.stamp = odom->header.stamp;
  trajectory2.header.frame_id = "map";
  pubTrajectoryPtr->publish(trajectory2);
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudIn)
{
  if (!systemDelayInited)
  {
    systemDelayCount++;
    if (systemDelayCount > systemDelay)
    {
      systemDelayInited = true;
    }
  }

  if (!systemInited)
  {
    return;
  }

  /*这段代码的目的是将接收到的激光雷达点云数据添加到探测体积的点云中，
   *然后对点云进行滤波处理，最后计算探测体积的大小。这个大小可以用于评
   *估探测任务的进展和覆盖区域的大小。
   */
  laserCloud->clear();
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);

  *exploredVolumeCloud += *laserCloud;

  exploredVolumeCloud2->clear();
  exploredVolumeDwzFilter.setInputCloud(exploredVolumeCloud);
  exploredVolumeDwzFilter.filter(*exploredVolumeCloud2);

  pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud = exploredVolumeCloud;
  exploredVolumeCloud = exploredVolumeCloud2;
  exploredVolumeCloud2 = tempCloud;

  // 计算探索过的空间体积
  exploredVolume = exploredVolumeVoxelSize * exploredVolumeVoxelSize * exploredVolumeVoxelSize * exploredVolumeCloud->points.size();

  /*这段代码的作用是定期发布已滤波和更新的探测区域点云消息，以便在可视化工具中查看实时探测区域的变化。*/
  *exploredAreaCloud += *laserCloud;

  exploredAreaDisplayCount++;
  if (exploredAreaDisplayCount >= 5 * exploredAreaDisplayInterval)
  {
    exploredAreaCloud2->clear();
    exploredAreaDwzFilter.setInputCloud(exploredAreaCloud);
    exploredAreaDwzFilter.filter(*exploredAreaCloud2);

    tempCloud = exploredAreaCloud;
    exploredAreaCloud = exploredAreaCloud2;
    exploredAreaCloud2 = tempCloud;

    sensor_msgs::PointCloud2 exploredArea2;
    pcl::toROSMsg(*exploredAreaCloud, exploredArea2);
    exploredArea2.header.stamp = laserCloudIn->header.stamp;
    exploredArea2.header.frame_id = "map";
    pubExploredAreaPtr->publish(exploredArea2);

    exploredAreaDisplayCount = 0;
  }

  // 输出上述参数，具体文件路径看launch中关于metricFilePtr的值
  fprintf(metricFilePtr, "%f %f %f %f\n", exploredVolume, travelingDis, runtime, timeDuration);

  // 发布探索体积
  std_msgs::Float32 exploredVolumeMsg;
  exploredVolumeMsg.data = exploredVolume;
  pubExploredVolumePtr->publish(exploredVolumeMsg);

  // 发布探索的行驶距离
  std_msgs::Float32 travelingDisMsg;
  travelingDisMsg.data = travelingDis;
  pubTravelingDisPtr->publish(travelingDisMsg);
}

void runtimeHandler(const std_msgs::Float32::ConstPtr &runtimeIn)
{
  // 读取时间参数
  runtime = runtimeIn->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visualizationTools");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("metricFile", metricFile);
  nhPrivate.getParam("trajFile", trajFile);
  nhPrivate.getParam("mapFile", mapFile);
  nhPrivate.getParam("overallMapVoxelSize", overallMapVoxelSize);
  nhPrivate.getParam("exploredAreaVoxelSize", exploredAreaVoxelSize);
  nhPrivate.getParam("exploredVolumeVoxelSize", exploredVolumeVoxelSize);
  nhPrivate.getParam("transInterval", transInterval);
  nhPrivate.getParam("yawInterval", yawInterval);
  nhPrivate.getParam("overallMapDisplayInterval", overallMapDisplayInterval);
  nhPrivate.getParam("exploredAreaDisplayInterval", exploredAreaDisplayInterval);

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("state_estimation", 5, odometryHandler);
  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("registered_scan_filted", 5, laserCloudHandler);

  // ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("state_estimation_noisy", 5, odometryHandler);
  // ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("transformed_cloud", 5, laserCloudHandler);

  ros::Subscriber subRuntime = nh.subscribe<std_msgs::Float32>("runtime", 5, runtimeHandler);

  ros::Publisher pubOverallMap = nh.advertise<sensor_msgs::PointCloud2>("overall_map", 5);

  ros::Publisher pubExploredArea = nh.advertise<sensor_msgs::PointCloud2>("explored_areas", 5);
  pubExploredAreaPtr = &pubExploredArea;

  ros::Publisher pubTrajectory = nh.advertise<sensor_msgs::PointCloud2>("trajectory", 5);
  pubTrajectoryPtr = &pubTrajectory;

  ros::Publisher pubExploredVolume = nh.advertise<std_msgs::Float32>("explored_volume", 5);
  pubExploredVolumePtr = &pubExploredVolume;

  ros::Publisher pubTravelingDis = nh.advertise<std_msgs::Float32>("traveling_distance", 5);
  pubTravelingDisPtr = &pubTravelingDis;

  ros::Publisher pubTimeDuration = nh.advertise<std_msgs::Float32>("time_duration", 5);
  pubTimeDurationPtr = &pubTimeDuration;

  overallMapDwzFilter.setLeafSize(overallMapVoxelSize, overallMapVoxelSize, overallMapVoxelSize);
  exploredAreaDwzFilter.setLeafSize(exploredAreaVoxelSize, exploredAreaVoxelSize, exploredAreaVoxelSize);
  exploredVolumeDwzFilter.setLeafSize(exploredVolumeVoxelSize, exploredVolumeVoxelSize, exploredVolumeVoxelSize);

  // 读入地图数据
  pcl::PLYReader ply_reader;
  if (ply_reader.read(mapFile, *overallMapCloud) == -1)
  {
    printf("\nCouldn't read pointcloud.ply file.\n\n");
  }

  overallMapCloudDwz->clear();
  overallMapDwzFilter.setInputCloud(overallMapCloud);
  overallMapDwzFilter.filter(*overallMapCloudDwz);
  overallMapCloud->clear();

  pcl::toROSMsg(*overallMapCloudDwz, overallMap2);

  // 取系统时间，设置string，并添加到上述输出的文件名中
  time_t logTime = time(0);
  tm *ltm = localtime(&logTime);
  string timeString = to_string(1900 + ltm->tm_year) + "-" + to_string(1 + ltm->tm_mon) + "-" + to_string(ltm->tm_mday) + "-" +
                      to_string(ltm->tm_hour) + "-" + to_string(ltm->tm_min) + "-" + to_string(ltm->tm_sec);

  std::string robot_ns = ros::this_node::getNamespace(); // 获取命名空间
  if (robot_ns.front() == '/')                           // 去掉前导斜杠
    robot_ns = robot_ns.substr(1);

  // 输出文件路径调试信息
  ROS_INFO_STREAM("[metricFile] " << metricFile);
  ROS_INFO_STREAM("[trajFile] " << trajFile);

  metricFile += "/" + robot_ns + "_" + timeString + ".txt";
  trajFile += "/" + robot_ns + "_" + timeString + ".txt";
  metricFilePtr = fopen(metricFile.c_str(), "w");
  trajFilePtr = fopen(trajFile.c_str(), "w");

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status)
  {
    ros::spinOnce();

    // 通过控制overallMapDisplayCount的次数进行overallMap2的更新发布
    overallMapDisplayCount++;
    if (overallMapDisplayCount >= 100 * overallMapDisplayInterval)
    {
      overallMap2.header.stamp = ros::Time().fromSec(systemTime);
      overallMap2.header.frame_id = "map";
      pubOverallMap.publish(overallMap2);

      overallMapDisplayCount = 0;
    }

    status = ros::ok();
    rate.sleep();
  }

  fclose(metricFilePtr);
  fclose(trajFilePtr);

  printf("\nExploration metrics and vehicle trajectory are saved in 'src/vehicle_simulator/log'.\n\n");

  return 0;
}
