#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <algorithm>
#include <cctype>
#include <string>

#include <ros/ros.h>

#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

string backend = "loam";
string robotNamespace;

string stateEstimationTopic = "/integrated_to_init";
string registeredScanTopic = "/velodyne_cloud_registered";
string outputOdometryTopic = "state_estimation";
string outputCloudTopic = "registered_scan";

string outputFrame = "map";
string outputChildFrame = "sensor";

bool flipStateEstimation = true;
bool flipRegisteredScan = true;
bool sendTF = true;
bool reverseTF = false;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());

nav_msgs::Odometry odomData;
tf::StampedTransform odomTrans;
ros::Publisher *pubOdometryPointer = NULL;
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;
ros::Publisher *pubLaserCloudPointer = NULL;

string trimSlashes(const string &value)
{
  if (value.empty())
    return "";

  size_t start = 0;
  size_t end = value.size();

  while (start < end && value[start] == '/')
    ++start;
  while (end > start && value[end - 1] == '/')
    --end;

  return value.substr(start, end - start);
}

string normalizedBackendName(string value)
{
  string normalized;
  normalized.reserve(value.size());

  for (size_t i = 0; i < value.size(); ++i)
  {
    const unsigned char c = static_cast<unsigned char>(value[i]);
    if (c == '_' || c == '-' || std::isspace(c))
      continue;
    normalized.push_back(static_cast<char>(std::tolower(c)));
  }

  return normalized;
}

string absoluteTopic(const string &ns, const string &topic)
{
  if (topic.empty())
    return "";
  if (topic[0] == '/')
    return topic;
  if (ns.empty())
    return "/" + topic;

  return "/" + ns + "/" + topic;
}

string defaultChildFrame(const string &ns)
{
  if (ns.empty())
    return "sensor";

  return ns + "/sensor";
}

bool getNonEmptyParam(ros::NodeHandle &nh, const string &name, string &value)
{
  string paramValue;
  if (!nh.getParam(name, paramValue))
    return false;
  if (paramValue.empty())
    return false;

  value = paramValue;
  return true;
}

void applyBackendDefaults()
{
  const string normalizedBackend = normalizedBackendName(backend);

  outputFrame = "map";
  outputChildFrame = defaultChildFrame(robotNamespace);
  outputOdometryTopic = absoluteTopic(robotNamespace, "state_estimation");
  outputCloudTopic = absoluteTopic(robotNamespace, "registered_scan");
  sendTF = true;
  reverseTF = false;

  if (normalizedBackend == "fastlio")
  {
    stateEstimationTopic = absoluteTopic(robotNamespace, "Odometry");
    registeredScanTopic = absoluteTopic(robotNamespace, "cloud_registered");
    flipStateEstimation = false;
    flipRegisteredScan = false;
  }
  else if (normalizedBackend == "liosam")
  {
    stateEstimationTopic = "/lio_sam/mapping/odometry";
    registeredScanTopic = "/lio_sam/mapping/cloud_registered";
    flipStateEstimation = false;
    flipRegisteredScan = false;
  }
  else if (normalizedBackend == "loam")
  {
    stateEstimationTopic = "/integrated_to_init";
    registeredScanTopic = "/velodyne_cloud_registered";
    flipStateEstimation = true;
    flipRegisteredScan = true;
  }
  else if (normalizedBackend == "custom")
  {
    stateEstimationTopic = "";
    registeredScanTopic = "";
    flipStateEstimation = false;
    flipRegisteredScan = false;
  }
  else
  {
    ROS_WARN_STREAM("Unknown loam_interface backend '" << backend
                                                       << "', using LOAM-compatible defaults.");
    backend = "loam";
    stateEstimationTopic = "/integrated_to_init";
    registeredScanTopic = "/velodyne_cloud_registered";
    flipStateEstimation = true;
    flipRegisteredScan = true;
  }
}

bool loadParameters(ros::NodeHandle &nhPrivate)
{
  getNonEmptyParam(nhPrivate, "backend", backend);

  if (!getNonEmptyParam(nhPrivate, "robotNamespace", robotNamespace) &&
      !getNonEmptyParam(nhPrivate, "namespace", robotNamespace) &&
      !getNonEmptyParam(nhPrivate, "NameSpace", robotNamespace))
  {
    const string nodeNamespace = ros::this_node::getNamespace();
    if (nodeNamespace != "/")
      robotNamespace = nodeNamespace;
  }

  robotNamespace = trimSlashes(robotNamespace);

  applyBackendDefaults();

  getNonEmptyParam(nhPrivate, "stateEstimationTopic", stateEstimationTopic);
  getNonEmptyParam(nhPrivate, "registeredScanTopic", registeredScanTopic);
  getNonEmptyParam(nhPrivate, "stateTopic", stateEstimationTopic);
  getNonEmptyParam(nhPrivate, "cloudTopic", registeredScanTopic);
  getNonEmptyParam(nhPrivate, "outputOdometryTopic", outputOdometryTopic);
  getNonEmptyParam(nhPrivate, "outputCloudTopic", outputCloudTopic);
  getNonEmptyParam(nhPrivate, "outputFrame", outputFrame);
  getNonEmptyParam(nhPrivate, "outputChildFrame", outputChildFrame);

  nhPrivate.getParam("flipStateEstimation", flipStateEstimation);
  nhPrivate.getParam("flipRegisteredScan", flipRegisteredScan);
  nhPrivate.getParam("sendTF", sendTF);
  nhPrivate.getParam("reverseTF", reverseTF);

  if (stateEstimationTopic.empty() || registeredScanTopic.empty())
  {
    ROS_FATAL_STREAM("loam_interface needs both odometry and cloud input topics. "
                     << "Set backend to loam/fastlio/liosam, or provide stateTopic/cloudTopic.");
    return false;
  }

  ROS_INFO_STREAM("loam_interface backend=" << backend
                                            << "\n  robotNamespace="
                                            << (robotNamespace.empty() ? "<none>" : robotNamespace)
                                            << "\n  state input=" << stateEstimationTopic
                                            << "\n  cloud input=" << registeredScanTopic
                                            << "\n  odom output=" << outputOdometryTopic
                                            << "\n  cloud output=" << outputCloudTopic
                                            << "\n  output frame=" << outputFrame
                                            << "\n  output child frame=" << outputChildFrame
                                            << "\n  flip odom="
                                            << (flipStateEstimation ? "true" : "false")
                                            << " flip cloud="
                                            << (flipRegisteredScan ? "true" : "false")
                                            << " sendTF=" << (sendTF ? "true" : "false"));
  return true;
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  odomData = *odom;

  if (flipStateEstimation)
  {
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

    pitch = -pitch;
    yaw = -yaw;

    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = odom->pose.pose.position.z;
    odomData.pose.pose.position.y = odom->pose.pose.position.x;
    odomData.pose.pose.position.z = odom->pose.pose.position.y;
  }

  odomData.header.frame_id = outputFrame;
  odomData.child_frame_id = outputChildFrame;
  pubOdometryPointer->publish(odomData);

  odomTrans.stamp_ = odom->header.stamp;
  odomTrans.frame_id_ = outputFrame;
  odomTrans.child_frame_id_ = outputChildFrame;
  odomTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
  odomTrans.setOrigin(tf::Vector3(odomData.pose.pose.position.x,
                                  odomData.pose.pose.position.y,
                                  odomData.pose.pose.position.z));

  if (sendTF)
  {
    if (!reverseTF)
    {
      tfBroadcasterPointer->sendTransform(odomTrans);
    }
    else
    {
      tfBroadcasterPointer->sendTransform(tf::StampedTransform(odomTrans.inverse(),
                                                               odom->header.stamp,
                                                               outputChildFrame,
                                                               outputFrame));
    }
  }
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudIn)
{
  if (!flipRegisteredScan)
  {
    sensor_msgs::PointCloud2 laserCloud2 = *laserCloudIn;
    laserCloud2.header.frame_id = outputFrame;
    pubLaserCloudPointer->publish(laserCloud2);
    return;
  }

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);

  const int laserCloudSize = laserCloud->points.size();
  for (int i = 0; i < laserCloudSize; i++)
  {
    const float temp = laserCloud->points[i].x;
    laserCloud->points[i].x = laserCloud->points[i].z;
    laserCloud->points[i].z = laserCloud->points[i].y;
    laserCloud->points[i].y = temp;
  }

  sensor_msgs::PointCloud2 laserCloud2;
  pcl::toROSMsg(*laserCloud, laserCloud2);
  laserCloud2.header.stamp = laserCloudIn->header.stamp;
  laserCloud2.header.frame_id = outputFrame;
  pubLaserCloudPointer->publish(laserCloud2);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "loamInterface");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  if (!loadParameters(nhPrivate))
    return 1;

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>(stateEstimationTopic, 5, odometryHandler);
  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(registeredScanTopic, 5, laserCloudHandler);

  ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry>(outputOdometryTopic, 5);
  pubOdometryPointer = &pubOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;

  ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>(outputCloudTopic, 5);
  pubLaserCloudPointer = &pubLaserCloud;

  ros::spin();

  return 0;
}
