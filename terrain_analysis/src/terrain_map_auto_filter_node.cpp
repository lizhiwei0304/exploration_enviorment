#include <ros/ros.h>
#include <ros/master.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <regex>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>
#include <atomic>
#include <algorithm>

class TerrainMapAutoFilter
{
public:
  TerrainMapAutoFilter(ros::NodeHandle &nh, ros::NodeHandle &pnh)
      : nh_(nh), pnh_(pnh)
  {
    // ========== params ==========
    odom_suffix_ = pnh_.param<std::string>("odom_suffix", "state_estimation_at_scan");

    radius_xy_ = pnh_.param<double>("radius_xy", 1.0);
    z_min_rel_ = pnh_.param<double>("z_min_rel", -1.0);
    z_max_rel_ = pnh_.param<double>("z_max_rel", 1.0);

    require_min_others_ = pnh_.param<int>("require_min_others", 1);
    max_pose_age_sec_ = pnh_.param<double>("max_pose_age_sec", 1.0);

    namespace_ = pnh_.param<std::string>("NameSpace", "vehicle0");
    self_id_ = ExtractRobotIDFromNamespace(namespace_);

    if (self_id_ < 0)
    {
      self_id_ = pnh_.param<int>("self_id", 0);
      ROS_WARN_STREAM("[Init] Use fallback param self_id = " << self_id_);
    }

    discovery_hz_ = pnh_.param<double>("discovery_hz", 2.0);

    // topics (可改成参数，但默认与你系统一致)
    in_topic_ = pnh_.param<std::string>("in_topic", "terrain_map");
    out_topic_ = pnh_.param<std::string>("out_topic", "terrain_map_filted");

    // ========== pub/sub ==========
    pub_filtered_ = nh_.advertise<sensor_msgs::PointCloud2>(out_topic_, 1);

    sub_cloud_ = nh_.subscribe(in_topic_, 1, &TerrainMapAutoFilter::cloudCb, this,
                               ros::TransportHints().tcpNoDelay(true));

    // ========== start discovery thread ==========
    running_.store(true);
    discovery_thread_ = std::thread(&TerrainMapAutoFilter::discoveryLoop, this);

    ROS_WARN_STREAM("[TerrainAutoFilter] started"
                    << "\n  in_topic=" << in_topic_
                    << "\n  out_topic=" << out_topic_
                    << "\n  odom_suffix=" << odom_suffix_
                    << "\n  self_id=" << self_id_
                    << "\n  radius_xy=" << radius_xy_ << " z_min_rel=" << z_min_rel_ << " z_max_rel=" << z_max_rel_
                    << "\n  require_min_others=" << require_min_others_
                    << "\n  max_pose_age_sec=" << max_pose_age_sec_
                    << "\n  discovery_hz=" << discovery_hz_);
  }

  ~TerrainMapAutoFilter()
  {
    running_.store(false);
    if (discovery_thread_.joinable())
      discovery_thread_.join();
  }

private:
  struct PoseCache
  {
    double x{0}, y{0}, z{0};
    ros::Time stamp;
    bool valid{false};
  };

  // ============================ Discovery ============================
  void discoveryLoop()
  {
    ros::Rate r(std::max(0.1, discovery_hz_));

    while (ros::ok() && running_.load())
    {
      ros::master::V_TopicInfo topics;
      ros::master::getTopics(topics);

      for (const auto &t : topics)
      {
        if (!endsWith(t.name, "/" + odom_suffix_))
          continue;

        int rid = parseVehicleId(t.name);
        if (rid < 0)
          continue;

        if (rid == self_id_)
          continue;

        {
          std::lock_guard<std::mutex> lk(mtx_);
          if (subscribed_odom_topics_.count(t.name))
            continue;
          subscribed_odom_topics_.insert(t.name);
        }

        ros::Subscriber sub = nh_.subscribe<nav_msgs::Odometry>(
            t.name, 5,
            boost::bind(&TerrainMapAutoFilter::odomCb, this, _1, rid),
            ros::VoidConstPtr(),
            ros::TransportHints().tcpNoDelay(true));

        {
          std::lock_guard<std::mutex> lk(mtx_);
          odom_subs_[t.name] = sub; // 保持生命周期
          poses_[rid] = PoseCache();
        }

        ROS_WARN_STREAM("[Discovery] subscribe odom: " << t.name << " rid=" << rid);
      }

      r.sleep();
    }
  }

  // ============================ Odom ============================
  void odomCb(const nav_msgs::OdometryConstPtr &msg, int rid)
  {
    PoseCache pc;
    pc.x = msg->pose.pose.position.x;
    pc.y = msg->pose.pose.position.y;
    pc.z = msg->pose.pose.position.z;
    pc.stamp = msg->header.stamp;
    pc.valid = true;

    std::lock_guard<std::mutex> lk(mtx_);
    poses_[rid] = pc;
  }

  // ============================ Cloud ============================
  void cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {
    // 1) gather other robot positions
    std::vector<Eigen::Vector3d> other_positions;
    {
      std::lock_guard<std::mutex> lk(mtx_);

      const ros::Time stamp = cloud_msg->header.stamp;
      for (const auto &kv : poses_)
      {
        const int rid = kv.first;
        if (rid == self_id_)
          continue;

        const PoseCache &pc = kv.second;
        if (!pc.valid)
          continue;

        if (max_pose_age_sec_ > 0.0)
        {
          const double age = (stamp - pc.stamp).toSec();
          if (age > max_pose_age_sec_)
            continue;
        }

        other_positions.emplace_back(pc.x, pc.y, pc.z);
      }
    }

    if (require_min_others_ > 0 && static_cast<int>(other_positions.size()) < require_min_others_)
    {
      ROS_WARN_STREAM_THROTTLE(1.0,
                               "[TerrainAutoFilter] BYPASS (no other poses): have=" << other_positions.size()
                                                                                    << " need>=" << require_min_others_);
      pub_filtered_.publish(*cloud_msg);
      return;
    }

    // 2) PointCloud2 -> PCL (XYZI)
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    if (!cloud || cloud->points.empty())
      return;

    const size_t before_sz = cloud->points.size();

    // 3) filter by cylinders around other robots
    RemovePointsNearRobots(cloud, other_positions, radius_xy_, z_min_rel_, z_max_rel_);

    const size_t after_sz = cloud->points.size();

    ROS_WARN_STREAM_THROTTLE(0.5,
                             "[TerrainAutoFilter] in=" << before_sz << " out=" << after_sz
                                                       << " removed=" << (before_sz > after_sz ? before_sz - after_sz : 0)
                                                       << " others=" << other_positions.size()
                                                       << " frame=" << cloud_msg->header.frame_id);

    // 4) publish
    sensor_msgs::PointCloud2 out;
    pcl::toROSMsg(*cloud, out);
    out.header = cloud_msg->header;
    pub_filtered_.publish(out);
  }

  // ============================ Filtering ============================
  static void RemovePointsNearRobots(
      pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
      const std::vector<Eigen::Vector3d> &other_robot_positions,
      double radius_xy,
      double z_min_rel,
      double z_max_rel)
  {
    if (!cloud || cloud->points.empty() || other_robot_positions.empty())
      return;

    const double r2 = radius_xy * radius_xy;

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
    filtered->header = cloud->header;
    filtered->points.reserve(cloud->points.size());

    for (const auto &p : cloud->points)
    {
      bool keep = true;

      for (const auto &rp : other_robot_positions)
      {
        const double dx = p.x - rp.x();
        const double dy = p.y - rp.y();
        if (dx * dx + dy * dy > r2)
          continue;

        const double z_rel = p.z - rp.z();
        if (z_rel >= z_min_rel && z_rel <= z_max_rel)
        {
          keep = false;
          break;
        }
      }

      if (keep)
        filtered->points.push_back(p);
    }

    filtered->width = static_cast<uint32_t>(filtered->points.size());
    filtered->height = 1;
    filtered->is_dense = false;

    cloud.swap(filtered);
  }

  // ============================ Helpers ============================
  static bool endsWith(const std::string &s, const std::string &suffix)
  {
    if (s.size() < suffix.size())
      return false;
    return std::equal(suffix.rbegin(), suffix.rend(), s.rbegin());
  }

  // 从 /vehicle12/state_estimation_at_scan 提取 12；失败返回 -1
  static int parseVehicleId(const std::string &topic)
  {
    std::regex re("/vehicle([0-9]+)/");
    std::smatch m;
    if (std::regex_search(topic, m, re))
    {
      try
      {
        return std::stoi(m[1].str());
      }
      catch (...)
      {
        return -1;
      }
    }
    return -1;
  }

  static int ExtractRobotIDFromNamespace(const std::string &ns)
  {
    int i = static_cast<int>(ns.size()) - 1;
    while (i >= 0 && std::isdigit(ns[i]))
      --i;

    if (i == static_cast<int>(ns.size()) - 1)
      return -1; // no digits

    return std::stoi(ns.substr(i + 1));
  }

private:
  ros::NodeHandle nh_, pnh_;

  std::string in_topic_;
  std::string out_topic_;
  std::string odom_suffix_;
  std::string namespace_;

  double radius_xy_{2.0};
  double z_min_rel_{-3.0};
  double z_max_rel_{3.0};

  int require_min_others_{1};
  double max_pose_age_sec_{1.0};
  int self_id_{0};
  double discovery_hz_{2.0};

  ros::Subscriber sub_cloud_;
  ros::Publisher pub_filtered_;

  std::atomic<bool> running_{false};
  std::thread discovery_thread_;

  std::mutex mtx_;

  std::unordered_set<std::string> subscribed_odom_topics_;
  std::unordered_map<std::string, ros::Subscriber> odom_subs_;
  std::unordered_map<int, PoseCache> poses_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "terrain_map_auto_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  TerrainMapAutoFilter node(nh, pnh);
  ros::spin();
  return 0;
}
