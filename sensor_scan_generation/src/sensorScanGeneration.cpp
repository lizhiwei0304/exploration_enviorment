#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl/filters/filter.h>

struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(float, time, time))

using namespace std;

pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn(new pcl::PointCloud<PointXYZIRT>());
pcl::PointCloud<PointXYZIRT>::Ptr laserCLoudInSensorFrame(new pcl::PointCloud<PointXYZIRT>());

double robotX = 0;
double robotY = 0;
double robotZ = 0;
double roll = 0;
double pitch = 0;
double yaw = 0;

bool newTransformToMap = false;

string sensor_scan_frame;

nav_msgs::Odometry odometryIn;
ros::Publisher *pubOdometryPointer = NULL;
tf::StampedTransform transformToMap;
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;

ros::Publisher pubLaserCloud;

void laserCloudAndOdometryHandler(const nav_msgs::Odometry::ConstPtr &odometry,
                                  const sensor_msgs::PointCloud2ConstPtr &laserCloud2)
{
  // 清空上次数据
  laserCloudIn->clear();
  laserCLoudInSensorFrame->clear();

  // 读入点云数据
  pcl::fromROSMsg(*laserCloud2, *laserCloudIn);

  // 读入odom数据
  odometryIn = *odometry;

  // 创建一个transformToMap变换，将里程计信息应用于地图坐标系的点云数据。这个变换包括了位置和方向的信息。
  transformToMap.setOrigin(
      tf::Vector3(odometryIn.pose.pose.position.x, odometryIn.pose.pose.position.y, odometryIn.pose.pose.position.z));
  transformToMap.setRotation(tf::Quaternion(odometryIn.pose.pose.orientation.x, odometryIn.pose.pose.orientation.y,
                                            odometryIn.pose.pose.orientation.z, odometryIn.pose.pose.orientation.w));
                                            
  int laserCloudInNum = laserCloudIn->points.size();
  PointXYZIRT p1;
  tf::Vector3 vec;

  for (int i = 0; i < laserCloudInNum; i++)
  {
    p1 = laserCloudIn->points[i];
    vec.setX(p1.x);
    vec.setY(p1.y);
    vec.setZ(p1.z);

    vec = transformToMap.inverse() * vec;

    PointXYZIRT p2 = p1;
    p2.x = vec.x();
    p2.y = vec.y();
    p2.z = vec.z();

    // 保留原有属性
    p2.intensity = p1.intensity;
    p2.ring = p1.ring;
    p2.time = p1.time;

    laserCLoudInSensorFrame->points.push_back(p2);
  }

  // 发布map到sensor_at_scan的odom
  odometryIn.header.stamp = laserCloud2->header.stamp;
  odometryIn.header.frame_id = "map";
  odometryIn.child_frame_id = sensor_scan_frame;
  pubOdometryPointer->publish(odometryIn);

  // 发布map到sensor_at_scan的tf关系
  transformToMap.stamp_ = laserCloud2->header.stamp;
  transformToMap.frame_id_ = "map";
  transformToMap.child_frame_id_ = sensor_scan_frame;
  tfBroadcasterPointer->sendTransform(transformToMap);

  // 发布到sensor_at_scan话题上的点云
  sensor_msgs::PointCloud2 scan_data;
  pcl::toROSMsg(*laserCLoudInSensorFrame, scan_data);
  scan_data.header.stamp = laserCloud2->header.stamp;
  scan_data.header.frame_id = sensor_scan_frame;
  pubLaserCloud.publish(scan_data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensor_scan");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("sensor_scan_frame", sensor_scan_frame);

  // Ros 时间同步消息滤波
  message_filters::Subscriber<nav_msgs::Odometry> subOdometry;
  message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserCloud;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> syncPolicy;
  typedef message_filters::Synchronizer<syncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
  subOdometry.subscribe(nh, "state_estimation", 1);
  subLaserCloud.subscribe(nh, "registered_scan", 1);
  sync_.reset(new Sync(syncPolicy(100), subOdometry, subLaserCloud));
  sync_->registerCallback(boost::bind(laserCloudAndOdometryHandler, _1, _2));

  ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry>("state_estimation_at_scan", 5);
  pubOdometryPointer = &pubOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;

  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("sensor_scan", 2);

  ros::spin();

  return 0;
}
