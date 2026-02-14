#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Header.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <vector>

class PointCloudTransformer
{
public:
  PointCloudTransformer(ros::NodeHandle &nh)
      : nh_(nh)
  {
    // std::string ns;
    // nh_.param<std::string>("robot_namespace", ns, "vehicle0");

    // pose_topic_ = ns + "/state_estimation_noisy";
    // scan_topic_ = ns + "/sensor_scan";
    // output_topic_ = ns + "/transformed_cloud";

    pose_topic_ = "state_estimation_noisy";
    scan_topic_ = "sensor_scan";
    output_topic_ = "transformed_cloud";

    pose_sub_ = nh_.subscribe(pose_topic_, 10, &PointCloudTransformer::poseCallback, this);
    cloud_sub_ = nh_.subscribe(scan_topic_, 10, &PointCloudTransformer::cloudCallback, this);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);

    latest_pose_received_ = false;
    // ROS_INFO("PointCloud Transformer Initialized for [%s]", ns.c_str());
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_, cloud_sub_;
  ros::Publisher cloud_pub_;
  std::string pose_topic_, scan_topic_, output_topic_;

  geometry_msgs::Pose latest_pose_;
  bool latest_pose_received_;

  void poseCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    latest_pose_ = msg->pose.pose;
    latest_pose_received_ = true;
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
  {
    if (!latest_pose_received_)
    {
      ROS_WARN_THROTTLE(5.0, "No pose received yet.");
      return;
    }

    Eigen::Affine3d transform = getTransformMatrix(latest_pose_);
    std::vector<Eigen::Vector3f> transformed_points;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      Eigen::Vector4d pt(*iter_x, *iter_y, *iter_z, 1.0);
      Eigen::Vector4d pt_map = transform.matrix() * pt;
      transformed_points.emplace_back(pt_map[0], pt_map[1], pt_map[2]);
    }

    sensor_msgs::PointCloud2 output;
    output.header.stamp = msg->header.stamp;
    output.header.frame_id = "map";
    output.height = 1;
    output.width = transformed_points.size();
    output.is_dense = false;
    output.is_bigendian = false;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(transformed_points.size());

    sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");

    for (size_t i = 0; i < transformed_points.size(); ++i, ++out_x, ++out_y, ++out_z)
    {
      *out_x = transformed_points[i].x();
      *out_y = transformed_points[i].y();
      *out_z = transformed_points[i].z();
    }

    cloud_pub_.publish(output);
  }

  Eigen::Affine3d getTransformMatrix(const geometry_msgs::Pose &pose)
  {
    Eigen::Translation3d translation(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    return translation * q;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_transformer");
  ros::NodeHandle nh;
  PointCloudTransformer transformer(nh);
  ros::spin();
  return 0;
}
