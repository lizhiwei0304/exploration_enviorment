#include <random>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

class DistanceBasedDriftPublisher
{
public:
  DistanceBasedDriftPublisher(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
      : nh_global_(nh), nh_private_(nh_private), total_distance_(0.0), last_position_set_(false)
  {
    // ==== 私有参数（~xxx） ====
    nh_private_.param("position_noise_std", position_noise_std_, 0.02);
    nh_private_.param("orientation_noise_std_deg", orientation_noise_std_deg_, 0.05);
    nh_private_.param("position_drift_per_meter", position_drift_per_meter_, 0.02);
    nh_private_.param("orientation_drift_per_meter_deg", orientation_drift_per_meter_deg_, 0.05);

    // ==== 全局参数 robot_namespace ====

    // nh_global.param<std::string>("robot_namespace", robot_namespace_, "vehicle0");

    // input_topic_ = "/" + robot_namespace_ + "/state_estimation_at_scan";
    // output_topic_ = "/" + robot_namespace_ + "/state_estimation_noisy";

    input_topic_ = "state_estimation_at_scan";
    output_topic_ = "state_estimation_noisy";

    sub_ = nh_global_.subscribe(input_topic_, 10, &DistanceBasedDriftPublisher::callback, this);
    pub_ = nh_global_.advertise<nav_msgs::Odometry>(output_topic_, 10);

    drift_pos_ = Eigen::Vector3d::Zero();
    drift_rpy_ = Eigen::Vector3d::Zero();

    ROS_INFO("Subscribed to: %s", input_topic_.c_str());
    ROS_INFO("Publishing noisy state to: %s", output_topic_.c_str());
  }

private:
  ros::NodeHandle nh_global_;
  ros::NodeHandle nh_private_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::string input_topic_, output_topic_, robot_namespace_;
  double position_noise_std_, orientation_noise_std_deg_;
  double position_drift_per_meter_, orientation_drift_per_meter_deg_;
  double total_distance_;
  bool last_position_set_;
  Eigen::Vector3d last_position_;
  Eigen::Vector3d drift_pos_;
  Eigen::Vector3d drift_rpy_;

  std::default_random_engine gen_;

  void callback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    Eigen::Vector3d current_position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    double delta_distance = 0.0;
    if (last_position_set_)
    {
      Eigen::Vector3d delta = current_position - last_position_;
      delta_distance = delta.head<2>().norm();
    }
    last_position_ = current_position;
    last_position_set_ = true;
    total_distance_ += delta_distance;

    std::normal_distribution<double> noise_dist(0.0, 1.0);

    // Add drift step
    drift_pos_.x() += noise_dist(gen_) * position_drift_per_meter_ * delta_distance;
    drift_pos_.y() += noise_dist(gen_) * position_drift_per_meter_ * delta_distance;

    for (int i = 0; i < 3; ++i)
    {
      drift_rpy_[i] += noise_dist(gen_) * orientation_drift_per_meter_deg_ * delta_distance * M_PI / 180.0;
    }

    // Generate noisy odom
    nav_msgs::Odometry noisy_msg = *msg;
    noisy_msg.pose.pose.position.x += noise_dist(gen_) * position_noise_std_ + drift_pos_.x();
    noisy_msg.pose.pose.position.y += noise_dist(gen_) * position_noise_std_ + drift_pos_.y();
    // z unchanged

    // Orientation
    tf::Quaternion q_orig, q_noise, q_drift, q_final;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q_orig);

    q_noise.setRPY(
        noise_dist(gen_) * orientation_noise_std_deg_ * M_PI / 180.0,
        noise_dist(gen_) * orientation_noise_std_deg_ * M_PI / 180.0,
        noise_dist(gen_) * orientation_noise_std_deg_ * M_PI / 180.0);

    q_drift.setRPY(drift_rpy_.x(), drift_rpy_.y(), drift_rpy_.z());

    q_final = q_orig * q_noise * q_drift;
    tf::quaternionTFToMsg(q_final, noisy_msg.pose.pose.orientation);

    pub_.publish(noisy_msg);

    ROS_INFO_THROTTLE(5, "[%.2f m] Drift XY: [%.2f, %.2f]", total_distance_, drift_pos_.x(), drift_pos_.y());
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "distance_based_drift_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  DistanceBasedDriftPublisher node(nh, nh_private);
  ros::spin();
  return 0;
}
