#include <ros/ros.h>
#include <ros/master.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <regex>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>

class RegisteredScanAutoFilter
{
public:
  RegisteredScanAutoFilter(ros::NodeHandle &nh, ros::NodeHandle &pnh)
      : nh_(nh), pnh_(pnh)
  {
    // 匹配 “/vehicleX/state_estimation_at_scan”
    odom_suffix_ = pnh_.param<std::string>("odom_suffix", "state_estimation_at_scan");

    // 过滤参数：XY 半径 + Z 相对窗口
    radius_xy_ = pnh_.param<double>("radius_xy", 1.0);
    z_min_rel_ = pnh_.param<double>("z_min_rel", -1.0);
    z_max_rel_ = pnh_.param<double>("z_max_rel", 1.0);

    // 门控：至少需要多少个“其他机器人”位置才过滤（0 = 不门控）
    require_min_others_ = pnh_.param<int>("require_min_others", 1);

    // freshness：只使用不超过这个年龄的 pose（秒）；<=0 表示不检查
    max_pose_age_sec_ = pnh_.param<double>("max_pose_age_sec", 1.0);

    //
    namespace_ = pnh_.param<std::string>("NameSpace", "vehicle0");
    self_id_ = ExtractRobotIDFromNamespace(namespace_);

    if (self_id_ < 0)
    {
      self_id_ = pnh_.param<int>("self_id", 0);
      ROS_WARN_STREAM("[Init] Use fallback param self_id = " << self_id_);
    }

    // discovery 频率
    discovery_hz_ = pnh_.param<double>("discovery_hz", 2.0);

    // ---- pub/sub ----
    pub_filtered_ = nh_.advertise<sensor_msgs::PointCloud2>("registered_scan_filted", 1);

    sub_cloud_ = nh_.subscribe("registered_scan", 1, &RegisteredScanAutoFilter::cloudCb, this,
                               ros::TransportHints().tcpNoDelay(true));

    // ---- start discovery thread ----
    running_.store(true);
    discovery_thread_ = std::thread(&RegisteredScanAutoFilter::discoveryLoop, this);

    ROS_WARN_STREAM("[AutoFilter] started"
                    << "\n  odom_suffix=" << odom_suffix_
                    << "\n  self_id=" << self_id_
                    << "\n  radius_xy=" << radius_xy_ << " z_min_rel=" << z_min_rel_ << " z_max_rel=" << z_max_rel_
                    << "\n  require_min_others=" << require_min_others_
                    << "\n  max_pose_age_sec=" << max_pose_age_sec_
                    << "\n  discovery_hz=" << discovery_hz_);
  }

  ~RegisteredScanAutoFilter()
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
        // 只找 odom：结尾是 /state_estimation_at_scan 的
        // t.name 例：/vehicle1/state_estimation_at_scan
        if (!endsWith(t.name, "/" + odom_suffix_))
          continue;

        // 从 topic 名里提取 vehicle id（支持 /vehicle12/xxx）
        int rid = parseVehicleId(t.name);
        if (rid < 0)
          continue;

        // 跳过自己
        if (rid == self_id_)
          continue;

        // 去重：已订阅就跳过
        {
          std::lock_guard<std::mutex> lk(mtx_);
          if (subscribed_odom_topics_.count(t.name))
            continue;
          subscribed_odom_topics_.insert(t.name);
        }

        // 新建订阅（注意：Subscriber 需要保存起来，否则会析构）
        ros::Subscriber sub = nh_.subscribe<nav_msgs::Odometry>(
            t.name, 5,
            boost::bind(&RegisteredScanAutoFilter::odomCb, this, _1, rid),
            ros::VoidConstPtr(),
            ros::TransportHints().tcpNoDelay(true));

        {
          std::lock_guard<std::mutex> lk(mtx_);
          odom_subs_[t.name] = sub;
          // 初始化 pose cache
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
    // 1) 取其他机器人位置（Eigen::Vector3d）
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

    // if (require_min_others_ > 0 && static_cast<int>(other_positions.size()) < require_min_others_)
    // {
    //   ROS_WARN_STREAM_THROTTLE(1.0,
    //                            "[AutoFilter] WAIT other poses: have=" << other_positions.size()
    //                                                                   << " need>=" << require_min_others_);
    //   // 也可以选择直接原样发布，这里我选择不发布（避免污染）
    //   return;
    // }

    if (require_min_others_ > 0 && static_cast<int>(other_positions.size()) < require_min_others_)
    {
      ROS_WARN_STREAM_THROTTLE(1.0,
                               "[AutoFilter] BYPASS (no other poses): have=" << other_positions.size()
                                                                             << " need>=" << require_min_others_);
      // 原样发布
      pub_filtered_.publish(*cloud_msg);
      return;
    }

    // 2) cloud_msg -> pcl
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    if (!cloud || cloud->points.empty())
      return;

    const size_t before_sz = cloud->points.size();

    // 3) 过滤（圆柱+Z窗口）
    RemovePointsNearRobots(cloud, other_positions, radius_xy_, z_min_rel_, z_max_rel_);

    const size_t after_sz = cloud->points.size();

    ROS_WARN_STREAM_THROTTLE(0.5,
                             "[AutoFilter] in=" << before_sz << " out=" << after_sz
                                                << " removed=" << (before_sz > after_sz ? before_sz - after_sz : 0)
                                                << " others=" << other_positions.size()
                                                << " frame=" << cloud_msg->header.frame_id);

    // 4) pcl -> ROS msg publish
    sensor_msgs::PointCloud2 out;
    pcl::toROSMsg(*cloud, out);
    out.header = cloud_msg->header;
    pub_filtered_.publish(out);
  }

  int ExtractRobotIDFromNamespace(const std::string &ns)
  {
    int i = ns.size() - 1;
    while (i >= 0 && std::isdigit(ns[i]))
      --i;

    if (i == static_cast<int>(ns.size()) - 1)
      return -1; // 没有数字

    return std::stoi(ns.substr(i + 1));
  }

  // ============================ Filtering function ============================
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
    // 支持 /vehicle0/xxx 或 /robot_1/xxx：你可以按你系统改正则
    // 这里先按 vehicle
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

private:
  ros::NodeHandle nh_, pnh_;

  std::string cloud_topic_;
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

  // 已订阅的话题集合：防止重复订阅
  std::unordered_set<std::string> subscribed_odom_topics_;

  // 保存 subscriber，否则析构会退订
  std::unordered_map<std::string, ros::Subscriber> odom_subs_;

  // rid -> pose cache
  std::unordered_map<int, PoseCache> poses_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "registered_scan_auto_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  RegisteredScanAutoFilter node(nh, pnh);
  ros::spin();
  return 0;
}
