#include <ros/ros.h>
#include <ros/master.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cctype>
#include <deque>
#include <limits>
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
    odom_suffix_ = pnh_.param<std::string>("odom_suffix", "state_estimation_at_scan");

    vehicle_box_length_ = pnh_.param<double>("vehicle_box_length", 0.75);
    vehicle_box_width_ = pnh_.param<double>("vehicle_box_width", 0.55);
    vehicle_box_z_min_rel_ = pnh_.param<double>("vehicle_box_z_min_rel", -0.75);
    vehicle_box_z_max_rel_ = pnh_.param<double>("vehicle_box_z_max_rel", -0.25);

    filter_min_robot_distance_ = pnh_.param<double>("filter_min_robot_distance", 0.1);
    filter_visibility_range_ = pnh_.param<double>("filter_visibility_range", 12.0);
    filter_horizontal_fov_rad_ = deg2rad(pnh_.param<double>("filter_horizontal_fov_deg", 360.0));
    filter_visibility_z_min_rel_ = pnh_.param<double>("filter_visibility_z_min_rel", -2.0);
    filter_visibility_z_max_rel_ = pnh_.param<double>("filter_visibility_z_max_rel", 2.0);
    filter_visibility_bearing_tolerance_rad_ =
        deg2rad(pnh_.param<double>("filter_visibility_bearing_tolerance_deg", 4.0));
    filter_visibility_occlusion_margin_ = pnh_.param<double>("filter_visibility_occlusion_margin", 0.8);
    filter_visibility_support_range_margin_ = pnh_.param<double>("filter_visibility_support_range_margin", 1.0);
    filter_visibility_min_points_ = pnh_.param<int>("filter_visibility_min_points", 1);
    filter_require_scan_support_ = pnh_.param<bool>("filter_require_scan_support", true);
    filter_require_self_pose_ = pnh_.param<bool>("filter_require_self_pose", true);

    require_min_others_ = pnh_.param<int>("require_min_others", 1);

    max_pose_age_sec_ = pnh_.param<double>("max_pose_age_sec", 1.0);
    pose_history_duration_sec_ = pnh_.param<double>("pose_history_duration_sec", 2.0);
    pose_window_padding_sec_ = pnh_.param<double>("pose_window_padding_sec", 0.05);
    first_cloud_pose_window_sec_ = pnh_.param<double>("first_cloud_pose_window_sec", 0.2);

    // 用于包围盒删除的历史位姿窗口。
    // 当 Viewpoint、odom 或点云存在延迟时，仅使用当前位姿会漏删机器人点云。
    // 因此保留最近一段时间内的多个机器人位姿，每个位姿都作为一个删除包围盒。
    bbox_filter_history_window_sec_ = pnh_.param<double>("bbox_filter_history_window_sec", 0.5);
    bbox_filter_max_samples_per_robot_ = pnh_.param<int>("bbox_filter_max_samples_per_robot", 8);

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
    sub_self_odom_ = nh_.subscribe<nav_msgs::Odometry>(odom_suffix_, 5, &RegisteredScanAutoFilter::selfOdomCb,
                                                       this, ros::TransportHints().tcpNoDelay(true));

    // ---- start discovery thread ----
    running_.store(true);
    discovery_thread_ = std::thread(&RegisteredScanAutoFilter::discoveryLoop, this);

    ROS_WARN_STREAM("[AutoFilter] started"
                    << "\n  odom_suffix=" << odom_suffix_
                    << "\n  self_id=" << self_id_
                    << "\n  vehicle_box_length=" << vehicle_box_length_
                    << " vehicle_box_width=" << vehicle_box_width_
                    << " z_min_rel=" << vehicle_box_z_min_rel_
                    << " z_max_rel=" << vehicle_box_z_max_rel_
                    << "\n  filter_min_robot_distance=" << filter_min_robot_distance_
                    << "\n  filter_visibility_range=" << filter_visibility_range_
                    << " filter_horizontal_fov_deg=" << rad2deg(filter_horizontal_fov_rad_)
                    << " filter_require_scan_support=" << filter_require_scan_support_
                    << " filter_require_self_pose=" << filter_require_self_pose_
                    << "\n  require_min_others=" << require_min_others_
                    << "\n  max_pose_age_sec=" << max_pose_age_sec_
                    << " pose_history_duration_sec=" << pose_history_duration_sec_
                    << " pose_window_padding_sec=" << pose_window_padding_sec_
                    << " first_cloud_pose_window_sec=" << first_cloud_pose_window_sec_
                    << "\n  bbox_filter_history_window_sec=" << bbox_filter_history_window_sec_
                    << " bbox_filter_max_samples_per_robot=" << bbox_filter_max_samples_per_robot_
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
    double yaw{0};
    ros::Time stamp;
    bool valid{false};
  };

  struct RobotPose
  {
    Eigen::Vector3d position;
    double yaw{0};
    int robot_id{-1};
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
            boost::bind(&RegisteredScanAutoFilter::odomCb, this, _1, rid),
            ros::VoidConstPtr(),
            ros::TransportHints().tcpNoDelay(true));

        {
          std::lock_guard<std::mutex> lk(mtx_);
          odom_subs_[t.name] = sub;
          pose_histories_[rid] = std::deque<PoseCache>();
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
    pc.yaw = yawFromQuaternion(msg->pose.pose.orientation);
    pc.stamp = msg->header.stamp;
    pc.valid = true;

    std::lock_guard<std::mutex> lk(mtx_);
    auto &history = pose_histories_[rid];
    history.push_back(pc);
    prunePoseHistory(history, pc.stamp);
  }

  void selfOdomCb(const nav_msgs::OdometryConstPtr &msg)
  {
    PoseCache pc;
    pc.x = msg->pose.pose.position.x;
    pc.y = msg->pose.pose.position.y;
    pc.z = msg->pose.pose.position.z;
    pc.yaw = yawFromQuaternion(msg->pose.pose.orientation);
    pc.stamp = msg->header.stamp;
    pc.valid = true;

    std::lock_guard<std::mutex> lk(mtx_);
    self_pose_ = pc;
    self_pose_history_.push_back(pc);
    prunePoseHistory(self_pose_history_, pc.stamp);
  }

  // ============================ Cloud ============================
  void cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {
    // 1) cloud_msg -> pcl
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    if (!cloud || cloud->points.empty())
      return;

    // 2) Get self pose and all other robot poses between adjacent cloud frames.
    const ros::Time cloud_stamp = cloud_msg->header.stamp.isZero() ? ros::Time::now() : cloud_msg->header.stamp;
    PoseCache self_pose;
    std::vector<RobotPose> other_poses;
    int known_other_robot_count = 0;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!getClosestPose(self_pose_history_, cloud_stamp, self_pose))
        self_pose = self_pose_;

      // 使用更长的历史位姿窗口，而不是只使用当前 cloud interval 内的位姿。
      // 这样可以覆盖 Viewpoint / odom / registered_scan 不同步造成的位置延迟。
      const ros::Duration padding(std::max(0.0, pose_window_padding_sec_));
      const ros::Duration history_window(std::max(0.0, bbox_filter_history_window_sec_));
      ros::Time window_start = cloud_stamp - history_window - padding;
      ros::Time window_end = cloud_stamp + padding;

      last_cloud_stamp_ = cloud_stamp;
      have_last_cloud_stamp_ = true;

      for (const auto &kv : pose_histories_)
      {
        const int rid = kv.first;
        if (rid == self_id_)
          continue;

        std::vector<RobotPose> samples =
            getRobotPoseSamplesForBBoxFiltering(kv.second, rid, window_start, window_end,
                                                cloud_stamp, bbox_filter_max_samples_per_robot_);
        if (samples.empty())
        {
          continue;
        }

        ++known_other_robot_count;
        other_poses.insert(other_poses.end(), samples.begin(), samples.end());
      }
    }

    if (filter_require_self_pose_ &&
        (!self_pose.valid || !poseFresh(self_pose, cloud_stamp, max_pose_age_sec_)))
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "[AutoFilter] BYPASS (no fresh self pose)");
      pub_filtered_.publish(*cloud_msg);
      return;
    }

    if (require_min_others_ > 0 && known_other_robot_count < require_min_others_)
    {
      ROS_WARN_STREAM_THROTTLE(1.0,
                               "[AutoFilter] BYPASS (no other pose histories): robots=" << known_other_robot_count
                                                                             << " samples=" << other_poses.size()
                                                                             << " need>=" << require_min_others_);
      // 原样发布
      pub_filtered_.publish(*cloud_msg);
      return;
    }

    // 不再根据机器人间距离、FOV 或扫描支撑点判断“是否可见”。
    // 只要存在其他机器人的位姿样本，就直接用该机器人车体包围盒删除落入盒内的点。
    const std::vector<RobotPose> &filter_robot_poses = other_poses;

    if (filter_robot_poses.empty())
    {
      ROS_WARN_STREAM_THROTTLE(1.0,
                               "[AutoFilter] BYPASS (no other robot pose samples): known=" << other_poses.size());
      pub_filtered_.publish(*cloud_msg);
      return;
    }

    const size_t before_sz = cloud->points.size();

    // 3) Filter by an oriented vehicle body box around other robots.
    RemovePointsNearRobots(cloud, filter_robot_poses, vehicle_box_length_, vehicle_box_width_,
                           vehicle_box_z_min_rel_, vehicle_box_z_max_rel_);

    const size_t after_sz = cloud->points.size();

    ROS_WARN_STREAM_THROTTLE(0.5,
                             "[AutoFilter] in=" << before_sz << " out=" << after_sz
                                                << " removed=" << (before_sz > after_sz ? before_sz - after_sz : 0)
                                                << " filter_pose_samples=" << filter_robot_poses.size()
                                                << " pose_samples=" << other_poses.size()
                                                << " known_robots=" << known_other_robot_count
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

  void prunePoseHistory(std::deque<PoseCache> &history, const ros::Time &newest_stamp) const
  {
    if (pose_history_duration_sec_ <= 0.0 || newest_stamp.isZero())
      return;

    while (!history.empty() &&
           history.front().valid &&
           (newest_stamp - history.front().stamp).toSec() > pose_history_duration_sec_)
    {
      history.pop_front();
    }
  }

  static RobotPose toRobotPose(const PoseCache &pose, int robot_id)
  {
    return RobotPose{Eigen::Vector3d(pose.x, pose.y, pose.z), pose.yaw, robot_id};
  }

  bool getClosestPose(const std::deque<PoseCache> &history, const ros::Time &stamp, PoseCache &pose) const
  {
    if (history.empty())
      return false;

    double best_dt = std::numeric_limits<double>::infinity();
    bool found = false;
    for (const PoseCache &candidate : history)
    {
      if (!candidate.valid)
        continue;
      const double dt = std::abs((stamp - candidate.stamp).toSec());
      if (dt < best_dt)
      {
        best_dt = dt;
        pose = candidate;
        found = true;
      }
    }
    return found;
  }

  std::vector<RobotPose> getRobotPoseSamplesInWindow(const std::deque<PoseCache> &history,
                                                     int robot_id,
                                                     const ros::Time &window_start,
                                                     const ros::Time &window_end,
                                                     const ros::Time &cloud_stamp) const
  {
    std::vector<RobotPose> samples;
    if (history.empty())
      return samples;

    for (const PoseCache &pose : history)
    {
      if (!pose.valid)
        continue;
      if (pose.stamp < window_start || pose.stamp > window_end)
        continue;
      samples.push_back(toRobotPose(pose, robot_id));
    }

    // If no odom arrived exactly inside this cloud interval, fall back to the freshest
    // pose close to the cloud stamp so a sparse odom topic still filters the robot body.
    if (samples.empty())
    {
      const PoseCache &latest_pose = history.back();
      if (poseFresh(latest_pose, cloud_stamp, max_pose_age_sec_))
      {
        samples.push_back(toRobotPose(latest_pose, robot_id));
      }
    }

    return samples;
  }

  std::vector<RobotPose> getRobotPoseSamplesForBBoxFiltering(const std::deque<PoseCache> &history,
                                                              int robot_id,
                                                              const ros::Time &window_start,
                                                              const ros::Time &window_end,
                                                              const ros::Time &cloud_stamp,
                                                              int max_samples_per_robot) const
  {
    std::vector<PoseCache> selected_cache;
    std::vector<RobotPose> samples;
    if (history.empty())
      return samples;

    const int max_samples = std::max(1, max_samples_per_robot);

    // 先从最新位姿向前取样。这样当历史窗口内位姿过多时，优先保留离当前点云时间最近的位置。
    for (auto it = history.rbegin(); it != history.rend(); ++it)
    {
      const PoseCache &pose = *it;
      if (!pose.valid)
        continue;
      if (pose.stamp < window_start || pose.stamp > window_end)
        continue;

      selected_cache.push_back(pose);
      if (static_cast<int>(selected_cache.size()) >= max_samples)
        break;
    }

    // 如果窗口内没有采到位姿，则退化为使用最新且不过期的位姿。
    if (selected_cache.empty())
    {
      const PoseCache &latest_pose = history.back();
      if (poseFresh(latest_pose, cloud_stamp, max_pose_age_sec_))
      {
        selected_cache.push_back(latest_pose);
      }
    }

    // 反转为时间从旧到新，便于日志理解；实际滤波不依赖顺序。
    std::reverse(selected_cache.begin(), selected_cache.end());
    samples.reserve(selected_cache.size());
    for (const PoseCache &pose : selected_cache)
    {
      samples.push_back(toRobotPose(pose, robot_id));
    }

    return samples;
  }

  // ============================ Filtering function ============================
  static void RemovePointsNearRobots(
      pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
      const std::vector<RobotPose> &other_robot_poses,
      double box_length,
      double box_width,
      double z_min_rel,
      double z_max_rel)
  {
    if (!cloud || cloud->points.empty() || other_robot_poses.empty())
      return;

    const double half_length = std::max(0.0, box_length * 0.5);
    const double half_width = std::max(0.0, box_width * 0.5);
    if (half_length <= 0.0 || half_width <= 0.0 || z_min_rel > z_max_rel)
      return;

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
    filtered->header = cloud->header;
    filtered->points.reserve(cloud->points.size());

    for (const auto &p : cloud->points)
    {
      bool keep = true;

      for (const auto &robot_pose : other_robot_poses)
      {
        const double dx = p.x - robot_pose.position.x();
        const double dy = p.y - robot_pose.position.y();

        const double c = std::cos(robot_pose.yaw);
        const double s = std::sin(robot_pose.yaw);
        const double body_x = c * dx + s * dy;
        const double body_y = -s * dx + c * dy;
        const double body_z = p.z - robot_pose.position.z();

        if (std::abs(body_x) <= half_length &&
            std::abs(body_y) <= half_width &&
            body_z >= z_min_rel && body_z <= z_max_rel)
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
    filtered->is_dense = true;

    cloud.swap(filtered);
  }

  // ============================ Helpers ============================
  bool robotInCurrentSensorView(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                const RobotPose &self_pose,
                                const RobotPose &other_pose) const
  {
    const Eigen::Vector3d delta = other_pose.position - self_pose.position;
    const double range_xy = std::hypot(delta.x(), delta.y());
    // 注意：这里不再因为机器人距离过近而判定不可见。
    // 近距离时仍应允许后续包围盒滤除机器人自身点云。
    if (range_xy > filter_visibility_range_)
      return false;

    const double z_rel = delta.z();
    if (z_rel < filter_visibility_z_min_rel_ || z_rel > filter_visibility_z_max_rel_)
      return false;

    const double bearing = normalizeAngle(std::atan2(delta.y(), delta.x()) - self_pose.yaw);
    if (filter_horizontal_fov_rad_ < twoPi() &&
        std::abs(bearing) > filter_horizontal_fov_rad_ * 0.5)
    {
      return false;
    }

    if (!filter_require_scan_support_)
      return true;

    double nearest_range_in_bearing = std::numeric_limits<double>::infinity();
    int support_points = 0;
    for (const auto &p : cloud->points)
    {
      const double dx = p.x - self_pose.position.x();
      const double dy = p.y - self_pose.position.y();
      const double point_range = std::hypot(dx, dy);
      if (point_range <= 1e-3 || point_range > filter_visibility_range_)
        continue;

      const double point_bearing = normalizeAngle(std::atan2(dy, dx) - self_pose.yaw);
      if (std::abs(normalizeAngle(point_bearing - bearing)) > filter_visibility_bearing_tolerance_rad_)
        continue;

      nearest_range_in_bearing = std::min(nearest_range_in_bearing, point_range);
      const double point_z_rel_to_robot = p.z - other_pose.position.z();
      const bool in_robot_height =
          point_z_rel_to_robot >= vehicle_box_z_min_rel_ &&
          point_z_rel_to_robot <= vehicle_box_z_max_rel_;
      const bool near_robot_range =
          std::abs(point_range - range_xy) <= filter_visibility_support_range_margin_;
      if (in_robot_height && near_robot_range)
      {
        ++support_points;
      }
    }

    if (!std::isfinite(nearest_range_in_bearing))
      return false;

    if (nearest_range_in_bearing < range_xy - filter_visibility_occlusion_margin_)
      return false;

    return support_points >= std::max(1, filter_visibility_min_points_);
  }

  static bool poseFresh(const PoseCache &pose, const ros::Time &stamp, double max_age_sec)
  {
    if (!pose.valid)
      return false;
    if (max_age_sec <= 0.0)
      return true;
    return std::abs((stamp - pose.stamp).toSec()) <= max_age_sec;
  }

  static double yawFromQuaternion(const geometry_msgs::Quaternion &q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static double normalizeAngle(double angle)
  {
    while (angle > pi())
      angle -= twoPi();
    while (angle < -pi())
      angle += twoPi();
    return angle;
  }

  static double deg2rad(double deg)
  {
    return deg * pi() / 180.0;
  }

  static double rad2deg(double rad)
  {
    return rad * 180.0 / pi();
  }

  static double pi()
  {
    return 3.14159265358979323846;
  }

  static double twoPi()
  {
    return 2.0 * pi();
  }

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

  double vehicle_box_length_{0.75};
  double vehicle_box_width_{0.55};
  double vehicle_box_z_min_rel_{-0.75};
  double vehicle_box_z_max_rel_{-0.25};
  double filter_min_robot_distance_{0.1};
  double filter_visibility_range_{12.0};
  double filter_horizontal_fov_rad_{6.283185307179586};
  double filter_visibility_z_min_rel_{-2.0};
  double filter_visibility_z_max_rel_{2.0};
  double filter_visibility_bearing_tolerance_rad_{0.06981317007977318};
  double filter_visibility_occlusion_margin_{0.8};
  double filter_visibility_support_range_margin_{1.0};
  int filter_visibility_min_points_{1};
  bool filter_require_scan_support_{true};
  bool filter_require_self_pose_{true};

  int require_min_others_{1};
  double max_pose_age_sec_{1.0};
  double pose_history_duration_sec_{2.0};
  double pose_window_padding_sec_{0.05};
  double first_cloud_pose_window_sec_{0.2};
  double bbox_filter_history_window_sec_{0.5};
  int bbox_filter_max_samples_per_robot_{8};
  int self_id_{0};
  double discovery_hz_{2.0};

  ros::Subscriber sub_cloud_;
  ros::Subscriber sub_self_odom_;
  ros::Publisher pub_filtered_;

  std::atomic<bool> running_{false};
  std::thread discovery_thread_;

  std::mutex mtx_;

  // 已订阅的话题集合：防止重复订阅
  std::unordered_set<std::string> subscribed_odom_topics_;

  // 保存 subscriber，否则析构会退订
  std::unordered_map<std::string, ros::Subscriber> odom_subs_;

  // rid -> pose history cache
  std::unordered_map<int, std::deque<PoseCache>> pose_histories_;
  std::deque<PoseCache> self_pose_history_;
  PoseCache self_pose_;
  ros::Time last_cloud_stamp_;
  bool have_last_cloud_stamp_{false};
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