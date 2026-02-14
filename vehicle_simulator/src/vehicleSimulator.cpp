#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <random>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/filters/impl/filter.hpp> // 必须
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>

// 定义自定义点结构
struct PointXYZIRT
{
  PCL_ADD_POINT4D; // Quad-word XYZ
  float intensity;
  std::uint16_t ring; // 使用 std::uint16_t 避免 warning
  float time;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// 注册点类型结构（必须与结构字段一致）
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(float, time, time))

using namespace std;

const double PI = 3.1415926;

bool use_gazebo_time = false;     // 使用仿真时间
double cameraOffsetZ = 0;         // 相机z轴方向偏离距离
double sensorOffsetX = 0;         // 雷达传感器x轴方向偏离距离
double sensorOffsetY = 0;         // 雷达传感器y轴方向偏离距离
double vehicleHeight = 0.75;      // 车辆的高度
double terrainVoxelSize = 0.05;   // 地形体素网格大小
double groundHeightThre = 0.1;    // 地面高度距离阈值
bool adjustZ = false;             // Z轴调整标志，主要用于地形识别后的平滑
double terrainRadiusZ = 0.5;      // z轴上车辆周围的阈值
int minTerrainPointNumZ = 10;     // 最小的z轴地面点数
double smoothRateZ = 0.2;         // 控制高度方向平滑的速率
bool adjustIncl = false;          // 调整地面点云倾斜的标志
double terrainRadiusIncl = 1.5;   // 地面点云倾斜角度的阈值
int minTerrainPointNumIncl = 500; // 地面点云倾斜调整的最小点数阈值
double smoothRateIncl = 0.2;      // 倾斜角度平滑的速率
double InclFittingThre = 0.2;     // I倾斜角度滤波的阈值
double maxIncl = 30.0;            // 最大的倾斜角度

const int systemDelay = 5; // 系统延迟时间
int systemInitCount = 0;   // 系统初始化计数
bool systemInited = false; // 系统初始化标志

static double last_ax = 0.0, last_ay = 0.0, last_az = 0.0;

pcl::PointCloud<PointXYZIRT>::Ptr velodyneData(new pcl::PointCloud<PointXYZIRT>());
pcl::PointCloud<PointXYZIRT>::Ptr scanData(new pcl::PointCloud<PointXYZIRT>());               // 激光雷达的扫描点云
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());     // 地面点云
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudIncl(new pcl::PointCloud<pcl::PointXYZI>()); // 地面倾斜点云
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());  // 地面点云下采样

std::vector<int> scanInd; // 扫描的ID号

ros::Time odomTime; // 时间

float vehicleX = 0;     // 车辆x轴起始位置
float vehicleY = 0;     // 车辆y轴起始位置
float vehicleZ = 0;     // 车辆z轴起始位置
float vehicleRoll = 0;  // 车辆Roll角
float vehiclePitch = 0; // 车辆Pitch角
float vehicleYaw = 0;   // 车辆Yaw角

float vehicleYawRate = 0; // 车辆Yaw角变化率
float vehicleSpeed = 0;   // 车辆速度

float terrainZ = 0;     // 地面z轴高度
float terrainRoll = 0;  // 地面Roll角
float terrainPitch = 0; // 地面Pitch角

const int stackNum = 400;
float vehicleXStack[stackNum];
float vehicleYStack[stackNum];
float vehicleZStack[stackNum];
float vehicleRollStack[stackNum];
float vehiclePitchStack[stackNum];
float vehicleYawStack[stackNum];
float terrainRollStack[stackNum];
float terrainPitchStack[stackNum];
double odomTimeStack[stackNum];
int odomSendIDPointer = -1;
int odomRecIDPointer = 0;

std::string sensor_frame;
std::string robot_model_name;
std::string camera_model_name;
std::string lidar_model_name;
std::string vehicle_frame;
std::string lidar_frame;

pcl::VoxelGrid<pcl::PointXYZI> terrainDwzFilter;

ros::Publisher *pubScanPointer = NULL;
ros::Publisher *pubCorrectedScan = NULL;

bool accel_initialized = false; // 初始两帧跳过计算
double last_x = 0.0, last2_x = 0.0;
double last_y = 0.0, last2_y = 0.0;
double last_z = 0.0, last2_z = 0.0;
const double dt = 0.005;                // 200Hz
const double dt2_inv = 1.0 / (dt * dt); // 二阶差分系数

// 用于模拟高斯白噪声
std::default_random_engine rand_gen;
std::normal_distribution<double> noise_gyro(0.0, 0.0005); // 角速度噪声 std=0.005 rad/s
std::normal_distribution<double> noise_accel(0.0, 0.001); // 加速度噪声 std=0.001 m/s^2

void scanHandler(const sensor_msgs::PointCloud2::ConstPtr &scanIn)
{
  // 当系统未初始化
  if (!systemInited)
  {
    systemInitCount++;
    // 当计数延迟大于系统延迟时
    if (systemInitCount > systemDelay)
    {
      systemInited = true;
    }
    return;
  }

  double scanTime = scanIn->header.stamp.toSec();
  // 当里程计信息不可用
  if (odomSendIDPointer < 0)
  {
    return;
  }
  /*循环处理里程计数据
   *通过循环找到时间戳最近的正确的里程计信息
   */
  while (odomTimeStack[(odomRecIDPointer + 1) % stackNum] < scanTime &&
         odomRecIDPointer != (odomSendIDPointer + 1) % stackNum)
  {
    odomRecIDPointer = (odomRecIDPointer + 1) % stackNum;
  }

  // 获得里程计信息和姿态
  double odomRecTime = odomTime.toSec();
  float vehicleRecX = vehicleX;
  float vehicleRecY = vehicleY;
  float vehicleRecZ = vehicleZ;
  float vehicleRecRoll = vehicleRoll;
  float vehicleRecPitch = vehiclePitch;
  float vehicleRecYaw = vehicleYaw;
  float terrainRecRoll = terrainRoll;
  float terrainRecPitch = terrainPitch;

  // 使用仿真时间则从栈中获得时间
  if (use_gazebo_time)
  {
    odomRecTime = odomTimeStack[odomRecIDPointer];
    vehicleRecX = vehicleXStack[odomRecIDPointer];
    vehicleRecY = vehicleYStack[odomRecIDPointer];
    vehicleRecZ = vehicleZStack[odomRecIDPointer];
    vehicleRecRoll = vehicleRollStack[odomRecIDPointer];
    vehicleRecPitch = vehiclePitchStack[odomRecIDPointer];
    vehicleRecYaw = vehicleYawStack[odomRecIDPointer];
    terrainRecRoll = terrainRollStack[odomRecIDPointer];
    terrainRecPitch = terrainPitchStack[odomRecIDPointer];
  }

  float sinTerrainRecRoll = sin(terrainRecRoll);
  float cosTerrainRecRoll = cos(terrainRecRoll);
  float sinTerrainRecPitch = sin(terrainRecPitch);
  float cosTerrainRecPitch = cos(terrainRecPitch);

  scanData->clear();
  pcl::fromROSMsg(*scanIn, *scanData);

  std::vector<int> index; // 存储有效点索引
  pcl::removeNaNFromPointCloud(*scanData, *scanData, index);

  // 坐标系转换
  // 定义类型为 PointXYZIRT
  pcl::PointCloud<PointXYZIRT>::Ptr scanDataXYZIRT(new pcl::PointCloud<PointXYZIRT>());

  // 遍历原始点云并转换坐标
  int scanDataSize = scanData->points.size();
  for (int i = 0; i < scanDataSize; i++)
  {
    const PointXYZIRT &pt_in = scanData->points[i]; // 原始点
    PointXYZIRT pt_out;

    // === 坐标变换 ===
    // 绕 x 轴旋转地面 Roll 角
    float pointX1 = pt_in.x;
    float pointY1 = pt_in.y * cosTerrainRecRoll - pt_in.z * sinTerrainRecRoll;
    float pointZ1 = pt_in.y * sinTerrainRecRoll + pt_in.z * cosTerrainRecRoll;

    // 绕 y 轴旋转地面 Pitch 角
    float pointX2 = pointX1 * cosTerrainRecPitch + pointZ1 * sinTerrainRecPitch;
    float pointY2 = pointY1;
    float pointZ2 = -pointX1 * sinTerrainRecPitch + pointZ1 * cosTerrainRecPitch;

    // 平移至车辆位置
    pt_out.x = pointX2 + vehicleRecX;
    pt_out.y = pointY2 + vehicleRecY;
    pt_out.z = pointZ2 + vehicleRecZ;

    // === 保留原始属性 ===
    pt_out.intensity = pt_in.intensity;
    pt_out.ring = pt_in.ring;
    pt_out.time = pt_in.time;

    scanDataXYZIRT->push_back(pt_out);
  }

  // === 发布 PointCloud2 消息 ===
  sensor_msgs::PointCloud2 scanData2;
  pcl::toROSMsg(*scanDataXYZIRT, scanData2);
  scanData2.header.stamp = ros::Time().fromSec(odomRecTime);
  scanData2.header.frame_id = "map";
  pubScanPointer->publish(scanData2);
}

void terrainCloudHandler(const sensor_msgs::PointCloud2ConstPtr &terrainCloud2)
{
  // 是否需要对地形参数进行调整
  if (!adjustZ && !adjustIncl)
  {
    return;
  }

  // 读取地面点云
  terrainCloud->clear();
  pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

  pcl::PointXYZI point;
  terrainCloudIncl->clear();
  int terrainCloudSize = terrainCloud->points.size();
  double elevMean = 0;
  int elevCount = 0;
  bool terrainValid = true;
  for (int i = 0; i < terrainCloudSize; i++)
  {
    point = terrainCloud->points[i];

    float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));

    // 点云距离是否小于地面高度区域阈值
    if (dis < terrainRadiusZ)
    {
      // 点云高度是否小于地面高度阈值
      if (point.intensity < groundHeightThre)
      {
        elevMean += point.z;
        elevCount++;
      }
      else
      {
        terrainValid = false;
      }
    }

    // 距离是否小于地面倾斜判定范围阈值 或者 点云的高度小于地面的阈值
    if (dis < terrainRadiusIncl && point.intensity < groundHeightThre)
    {
      terrainCloudIncl->push_back(point);
    }
  }

  if (elevCount >= minTerrainPointNumZ)
    elevMean /= elevCount;
  else
    terrainValid = false;

  // 如果地形有效且需要调整高度
  if (terrainValid && adjustZ)
  {
    // 线性化平滑地面z轴高度
    terrainZ = (1.0 - smoothRateZ) * terrainZ + smoothRateZ * elevMean;
  }

  // 对车辆周围的地面点云进行下采样
  terrainCloudDwz->clear();
  terrainDwzFilter.setInputCloud(terrainCloudIncl);
  terrainDwzFilter.filter(*terrainCloudDwz);
  int terrainCloudDwzSize = terrainCloudDwz->points.size();

  // 是否有足够数量的内点（有效的地形点）用于进行倾斜角度的估计 或者  地面无效
  if (terrainCloudDwzSize < minTerrainPointNumIncl || !terrainValid)
  {
    return;
  }

  /*迭代 RANSAC 进行地形参数估计：
   *通过 RANSAC 算法估计地形的俯仰和横滚角度。
   *RANSAC 是一种随机抽样一致性算法，用于估计模型参数（在这里是俯仰和横滚角度），并剔除异常值。
   *迭代次数为5次，每次迭代中计算估计参数 matX。
   **/
  cv::Mat matA(terrainCloudDwzSize, 2, CV_32F, cv::Scalar::all(0));
  cv::Mat matAt(2, terrainCloudDwzSize, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtA(2, 2, CV_32F, cv::Scalar::all(0));
  cv::Mat matB(terrainCloudDwzSize, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtB(2, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matX(2, 1, CV_32F, cv::Scalar::all(0));

  /*这段代码的目的是执行一个 RANSAC（随机抽样一致性）迭代，通过拟合地形点云数据
   *来估计地形的俯仰和横滚参数，并识别和剔除异常值。在每次迭代中，根据当前的地形参数
   *（terrainPitch 和 terrainRoll）计算残差，如果残差大于阈值，则将该点标记为异
   *常值，然后在下一次迭代中将其忽略，直到满足停止条件（5次迭代或没有新的异常值）。最
   *终，matX 中的 terrainPitch 和 terrainRoll 参数将是拟合的结果。这种方法可以提
   *高地形参数的估计精度，并剔除地形点云中的异常值。
   **/
  int inlierNum = 0;
  matX.at<float>(0, 0) = terrainPitch;
  matX.at<float>(1, 0) = terrainRoll;
  for (int iterCount = 0; iterCount < 5; iterCount++)
  {
    int outlierCount = 0;
    for (int i = 0; i < terrainCloudDwzSize; i++)
    {
      point = terrainCloudDwz->points[i];

      matA.at<float>(i, 0) = -point.x + vehicleX;
      matA.at<float>(i, 1) = point.y - vehicleY;
      matB.at<float>(i, 0) = point.z - elevMean;

      if (fabs(matA.at<float>(i, 0) * matX.at<float>(0, 0) + matA.at<float>(i, 1) * matX.at<float>(1, 0) -
               matB.at<float>(i, 0)) > InclFittingThre &&
          iterCount > 0)
      {
        matA.at<float>(i, 0) = 0;
        matA.at<float>(i, 1) = 0;
        matB.at<float>(i, 0) = 0;
        outlierCount++;
      }
    }

    /*这段代码的主要目的是通过拟合地形点云数据来估计地形的俯仰和横滚角度，并在
     *一些条件下判断地形是否有效。如果地形有效，它还会对地形角度进行平滑处理，以
     *减小估计的误差。这样可以提高地形参数的估计精度。
     */
    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR); // 使用QR分解求解线性方程组

    if (inlierNum == terrainCloudDwzSize - outlierCount)
      break;
    inlierNum = terrainCloudDwzSize - outlierCount;
  }

  /*这些条件用于对估计的地形参数进行限制和检查，以确保地形估计的合理性和准确性。
   *内点的数量是否小于指定的最小地形点数
   *估计的俯仰角度是否超过了指定的最大俯仰角度
   *估计的横滚角度是否超过了指定的最大横滚角度
   */
  if (inlierNum < minTerrainPointNumIncl || fabs(matX.at<float>(0, 0)) > maxIncl * PI / 180.0 ||
      fabs(matX.at<float>(1, 0)) > maxIncl * PI / 180.0)
  {
    terrainValid = false;
  }

  /*这段代码用于更新估计得到的地形俯仰角度terrainPitch 和横滚角度 terrainRoll，并且应用了平滑处理 */
  if (terrainValid && adjustIncl)
  {
    terrainPitch = (1.0 - smoothRateIncl) * terrainPitch + smoothRateIncl * matX.at<float>(0, 0);
    terrainRoll = (1.0 - smoothRateIncl) * terrainRoll + smoothRateIncl * matX.at<float>(1, 0);
  }
}

void speedHandler(const geometry_msgs::TwistStamped::ConstPtr &speedIn)
{
  vehicleSpeed = speedIn->twist.linear.x;    // 获取速度
  vehicleYawRate = speedIn->twist.angular.z; // 获取角速度
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vehicleSimulator");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("use_gazebo_time", use_gazebo_time);
  nhPrivate.getParam("cameraOffsetZ", cameraOffsetZ);
  nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
  nhPrivate.getParam("vehicleHeight", vehicleHeight);
  nhPrivate.getParam("vehicleX", vehicleX);
  nhPrivate.getParam("vehicleY", vehicleY);
  nhPrivate.getParam("vehicleZ", vehicleZ);
  nhPrivate.getParam("terrainZ", terrainZ);
  nhPrivate.getParam("vehicleYaw", vehicleYaw);
  nhPrivate.getParam("terrainVoxelSize", terrainVoxelSize);
  nhPrivate.getParam("groundHeightThre", groundHeightThre);
  nhPrivate.getParam("adjustZ", adjustZ);
  nhPrivate.getParam("terrainRadiusZ", terrainRadiusZ);
  nhPrivate.getParam("minTerrainPointNumZ", minTerrainPointNumZ);
  nhPrivate.getParam("adjustIncl", adjustIncl);
  nhPrivate.getParam("terrainRadiusIncl", terrainRadiusIncl);
  nhPrivate.getParam("minTerrainPointNumIncl", minTerrainPointNumIncl);
  nhPrivate.getParam("InclFittingThre", InclFittingThre);
  nhPrivate.getParam("maxIncl", maxIncl);

  nhPrivate.getParam("sensor_frame", sensor_frame);
  nhPrivate.getParam("robot_model_name", robot_model_name);
  nhPrivate.getParam("camera_model_name", camera_model_name);
  nhPrivate.getParam("lidar_model_name", lidar_model_name);
  nhPrivate.getParam("vehicle_frame", vehicle_frame);
  nhPrivate.getParam("lidar_frame", lidar_frame);

  ros::Subscriber subScan = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 2, scanHandler);

  ros::Subscriber subTerrainCloud = nh.subscribe<sensor_msgs::PointCloud2>("terrain_map", 2, terrainCloudHandler);

  ros::Subscriber subSpeed = nh.subscribe<geometry_msgs::TwistStamped>("cmd_vel", 5, speedHandler);

  ros::Publisher pubVehicleOdom = nh.advertise<nav_msgs::Odometry>("state_estimation", 5);

  nav_msgs::Odometry odomData;
  odomData.header.frame_id = "map";
  odomData.child_frame_id = sensor_frame;

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform odomTrans;
  odomTrans.frame_id_ = "map";
  odomTrans.child_frame_id_ = sensor_frame;

  ros::Publisher pubModelState = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 5);
  gazebo_msgs::ModelState cameraState;
  cameraState.model_name = camera_model_name;
  gazebo_msgs::ModelState lidarState;
  lidarState.model_name = lidar_model_name;
  gazebo_msgs::ModelState robotState;
  robotState.model_name = robot_model_name;

  ros::Publisher pubScan = nh.advertise<sensor_msgs::PointCloud2>("registered_scan", 2);
  pubScanPointer = &pubScan;

  terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);

  printf("\nSimulation started.\n\n");

  ros::Rate rate(200);
  bool status = ros::ok();
  while (status)
  {
    ros::spinOnce();

    float vehicleRecRoll = vehicleRoll;
    float vehicleRecPitch = vehiclePitch;
    float vehicleRecZ = vehicleZ;

    vehicleRoll = terrainRoll * cos(vehicleYaw) + terrainPitch * sin(vehicleYaw);
    vehiclePitch = -terrainRoll * sin(vehicleYaw) + terrainPitch * cos(vehicleYaw);
    vehicleYaw += 0.005 * vehicleYawRate;
    if (vehicleYaw > PI)
      vehicleYaw -= 2 * PI;
    else if (vehicleYaw < -PI)
      vehicleYaw += 2 * PI;

    vehicleX += 0.005 * cos(vehicleYaw) * vehicleSpeed +
                0.005 * vehicleYawRate * (-sin(vehicleYaw) * sensorOffsetX - cos(vehicleYaw) * sensorOffsetY);
    vehicleY += 0.005 * sin(vehicleYaw) * vehicleSpeed +
                0.005 * vehicleYawRate * (cos(vehicleYaw) * sensorOffsetX - sin(vehicleYaw) * sensorOffsetY);
    vehicleZ = terrainZ + vehicleHeight;

    ros::Time odomTimeRec = odomTime;
    odomTime = ros::Time::now();
    if (odomTime == odomTimeRec)
      odomTime += ros::Duration(0.005);

    odomSendIDPointer = (odomSendIDPointer + 1) % stackNum;
    odomTimeStack[odomSendIDPointer] = odomTime.toSec();
    vehicleXStack[odomSendIDPointer] = vehicleX;
    vehicleYStack[odomSendIDPointer] = vehicleY;
    vehicleZStack[odomSendIDPointer] = vehicleZ;
    vehicleRollStack[odomSendIDPointer] = vehicleRoll;
    vehiclePitchStack[odomSendIDPointer] = vehiclePitch;
    vehicleYawStack[odomSendIDPointer] = vehicleYaw;
    terrainRollStack[odomSendIDPointer] = terrainRoll;
    terrainPitchStack[odomSendIDPointer] = terrainPitch;

    // publish 200Hz odometry messages
    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(vehicleRoll, vehiclePitch, vehicleYaw);

    odomData.header.stamp = odomTime;
    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = vehicleX;
    odomData.pose.pose.position.y = vehicleY;
    odomData.pose.pose.position.z = vehicleZ;
    odomData.twist.twist.angular.x = 200.0 * (vehicleRoll - vehicleRecRoll);   // 200 是0.005秒的倒数
    odomData.twist.twist.angular.y = 200.0 * (vehiclePitch - vehicleRecPitch); // 200 是0.005秒的倒数
    odomData.twist.twist.angular.z = vehicleYawRate;
    odomData.twist.twist.linear.x = vehicleSpeed;
    odomData.twist.twist.linear.z = 200.0 * (vehicleZ - vehicleRecZ); // 200 是0.005秒的倒数
    pubVehicleOdom.publish(odomData);

    // publish 200Hz tf messages
    odomTrans.stamp_ = odomTime;
    odomTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
    odomTrans.setOrigin(tf::Vector3(vehicleX, vehicleY, vehicleZ));
    tfBroadcaster.sendTransform(odomTrans);

    // publish 200Hz Gazebo model state messages (this is for Gazebo simulation)
    cameraState.pose.orientation = geoQuat;
    cameraState.pose.position.x = vehicleX;
    cameraState.pose.position.y = vehicleY;
    cameraState.pose.position.z = vehicleZ + cameraOffsetZ;
    pubModelState.publish(cameraState);

    robotState.pose.orientation = geoQuat;
    robotState.pose.position.x = vehicleX;
    robotState.pose.position.y = vehicleY;
    robotState.pose.position.z = vehicleZ;
    pubModelState.publish(robotState);

    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(terrainRoll, terrainPitch, 0);

    lidarState.pose.orientation = geoQuat;
    lidarState.pose.position.x = vehicleX;
    lidarState.pose.position.y = vehicleY;
    lidarState.pose.position.z = vehicleZ;
    pubModelState.publish(lidarState);

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}

template void pcl::removeNaNFromPointCloud<PointXYZIRT>(
    const pcl::PointCloud<PointXYZIRT> &cloud_in,
    pcl::PointCloud<PointXYZIRT> &cloud_out,
    std::vector<int> &indices);
