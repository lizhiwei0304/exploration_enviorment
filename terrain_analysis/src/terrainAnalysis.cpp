#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

const double PI = 3.1415926;

double scanVoxelSize = 0.05;      // 扫描体素大小: 5cm
double decayTime = 2.0;           // 时间阈值：2.0s
double noDecayDis = 4.0;          // 车辆初始距离阈值：4.0m
double clearingDis = 8.0;         // 清除距离：8.0m
bool clearingCloud = false;       // 清除点云：否-false；是-true
bool useSorting = true;           // 使用排序：是
double quantileZ = 0.25;          // Z轴分辩数：0.25m
bool considerDrop = false;        // 考虑下降：否
bool limitGroundLift = false;     // 地面升高高度限制
double maxGroundLift = 0.15;      // 地面上升最大距离 0.15m
bool clearDyObs = false;          // 清楚障碍标志位
double minDyObsDis = 0.3;         // 最小动态障碍物距离阈值
double minDyObsAngle = 0;         // 通过动态障碍物的最小角度
double minDyObsRelZ = -0.5;       // 通过动态障碍物最小的Ｚ轴相对高度
double minDyObsVFOV = -16.0;      // 左侧最大转向角
double maxDyObsVFOV = 16.0;       // 右侧最大转向角
int minDyObsPointNum = 1;         // 最小障碍物点的数量
bool noDataObstacle = false;      // 无障碍物数据
int noDataBlockSkipNum = 0;       // 无障碍物阻塞跳过的点数
int minBlockPointNum = 10;        // 最小阻塞的点数
double vehicleHeight = 1.5;       // 车辆的高度
int voxelPointUpdateThre = 100;   // 同一个位置的雷达点数阈值
double voxelTimeUpdateThre = 2.0; // 同一个位置的雷达点时间阈值
double minRelZ = -1.5;            // Z轴最小的相对距离
double maxRelZ = 0.2;             // Z轴最小的相对距离
double disRatioZ = 0.2;           // 点云处理的高度与距离的比例-与激光雷达性能相关

// 地面体素参数
float terrainVoxelSize = 1.0;                                      // 地面体素网格的大小
int terrainVoxelShiftX = 0;                                        // 地面体素网格翻转时的X位置
int terrainVoxelShiftY = 0;                                        // 地面体素网格翻转时的Y位置
const int terrainVoxelWidth = 21;                                  // 地面体素的宽度
int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;           // 地面体素的宽度  10
const int terrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth; // 地面体素的大小  21×21

// 规划体素参数
float planarVoxelSize = 0.2;                                    // 平面体素网格的尺寸大小 0.2m
const int planarVoxelWidth = 51;                                // 点云存储的格子大小
int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2;          // 平面体素的宽度  25
const int planarVoxelNum = planarVoxelWidth * planarVoxelWidth; // 平面体素的大小  51×51

pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    terrainCloudElev(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[terrainVoxelNum]; // 每个像素对应存储一个点云指针

int terrainVoxelUpdateNum[terrainVoxelNum] = {0};    // 地面体素更新数量
float terrainVoxelUpdateTime[terrainVoxelNum] = {0}; // 地面体素更新时间
float planarVoxelElev[planarVoxelNum] = {0};         // 地面体素网格高程的最小值
int planarVoxelEdge[planarVoxelNum] = {0};           // 地面体素网格边缘的信息
int planarVoxelDyObs[planarVoxelNum] = {0};          // 地面体素网格动态障碍物信息
vector<float> planarPointElev[planarVoxelNum];       // 存储了地面体素网格附近一个平面网格的所有点云的高程信息

double laserCloudTime = 0;  // 雷达第一帧数据时间
bool newlaserCloud = false; // 雷达数据接收标志位

double systemInitTime = 0; // 系统初始化时间，根据第一帧点云信息的时间设定
bool systemInited = false; // 系统初始化标志位 false-未初始化；true-已经初始化
int noDataInited = 0;      // 车辆初始位置的标志位 0-未赋值，将收到的第一个车辆位置赋值；1-表示已经初始化；2-车辆初始距离误差大于初始阈值

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float vehicleXRec = 0, vehicleYRec = 0;

float sinVehicleRoll = 0, cosVehicleRoll = 0;
float sinVehiclePitch = 0, cosVehiclePitch = 0;
float sinVehicleYaw = 0, cosVehicleYaw = 0;

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter; // 点云三维体素化下采样

// state estimation callback function
void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;

  sinVehicleRoll = sin(vehicleRoll);
  cosVehicleRoll = cos(vehicleRoll);
  sinVehiclePitch = sin(vehiclePitch);
  cosVehiclePitch = cos(vehiclePitch);
  sinVehicleYaw = sin(vehicleYaw);
  cosVehicleYaw = cos(vehicleYaw);
  /*这段代码的主要目的是在车辆的位置发生较大变化时，通过距离测量来触发数据初始化。
   *在初始阶段，数据会被初始化并记录，然后当车辆移动一定距离后，数据初始化完成，
   *noDataInited 的值被设置为2。这可能用于控制程序中的某些逻辑或行为，
   *以确保在数据准备好之前不进行某些操作。
   **/
  if (noDataInited == 0)
  {
    vehicleXRec = vehicleX;
    vehicleYRec = vehicleY;
    noDataInited = 1;
  }
  if (noDataInited == 1)
  {
    float dis = sqrt((vehicleX - vehicleXRec) * (vehicleX - vehicleXRec) + (vehicleY - vehicleYRec) * (vehicleY - vehicleYRec));
    if (dis >= noDecayDis)
      noDataInited = 2;
  }
}

// lidar点云回调函数
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloud2)
{
  laserCloudTime = laserCloud2->header.stamp.toSec();

  if (!systemInited)
  {
    systemInitTime = laserCloudTime;
    systemInited = true;
  }

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloud2, *laserCloud);

  pcl::PointXYZI point;
  laserCloudCrop->clear();
  int laserCloudSize = laserCloud->points.size();
  for (int i = 0; i < laserCloudSize; i++)
  {
    point = laserCloud->points[i];

    float pointX = point.x;
    float pointY = point.y;
    float pointZ = point.z;

    float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
    if (pointZ - vehicleZ > minRelZ - disRatioZ * dis && pointZ - vehicleZ < maxRelZ + disRatioZ * dis && dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1))
    {
      point.x = pointX;
      point.y = pointY;
      point.z = pointZ;
      point.intensity = laserCloudTime - systemInitTime;
      laserCloudCrop->push_back(point);
    }
  }

  newlaserCloud = true;
}

// joystick callback function
void joystickHandler(const sensor_msgs::Joy::ConstPtr &joy)
{
  if (joy->buttons[5] > 0.5)
  {
    noDataInited = 0;
    clearingCloud = true;
  }
}

// cloud clearing callback function
/*总之，这段代码的主要目的是在接收到浮点数消息时触发一些操作，包括将
 *noDataInited 重置为0、记录距离值到 clearingDis 变量，并设置
 *clearingCloud 为 true。具体操作的含义和实现可能取决于整个程序的
 *上下文和设计。浮点数消息中的数据值可能用于控制何时执行激光云数据的清除或重置操作。
 */
void clearingHandler(const std_msgs::Float32::ConstPtr &dis)
{
  noDataInited = 0;
  clearingDis = dis->data;
  clearingCloud = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "terrainAnalysis");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("scanVoxelSize", scanVoxelSize);
  nhPrivate.getParam("decayTime", decayTime);
  nhPrivate.getParam("noDecayDis", noDecayDis);
  nhPrivate.getParam("clearingDis", clearingDis);
  nhPrivate.getParam("useSorting", useSorting);
  nhPrivate.getParam("quantileZ", quantileZ);
  nhPrivate.getParam("considerDrop", considerDrop);
  nhPrivate.getParam("limitGroundLift", limitGroundLift);
  nhPrivate.getParam("maxGroundLift", maxGroundLift);
  nhPrivate.getParam("clearDyObs", clearDyObs);
  nhPrivate.getParam("minDyObsDis", minDyObsDis);
  nhPrivate.getParam("minDyObsAngle", minDyObsAngle);
  nhPrivate.getParam("minDyObsRelZ", minDyObsRelZ);
  nhPrivate.getParam("minDyObsVFOV", minDyObsVFOV);
  nhPrivate.getParam("maxDyObsVFOV", maxDyObsVFOV);
  nhPrivate.getParam("minDyObsPointNum", minDyObsPointNum);
  nhPrivate.getParam("noDataObstacle", noDataObstacle);
  nhPrivate.getParam("noDataBlockSkipNum", noDataBlockSkipNum);
  nhPrivate.getParam("minBlockPointNum", minBlockPointNum);
  nhPrivate.getParam("vehicleHeight", vehicleHeight);
  nhPrivate.getParam("voxelPointUpdateThre", voxelPointUpdateThre);
  nhPrivate.getParam("voxelTimeUpdateThre", voxelTimeUpdateThre);
  nhPrivate.getParam("minRelZ", minRelZ);
  nhPrivate.getParam("maxRelZ", maxRelZ);
  nhPrivate.getParam("disRatioZ", disRatioZ);

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("state_estimation", 5, odometryHandler);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("registered_scan", 5, laserCloudHandler);
  // ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("registered_scan_filted", 5, laserCloudHandler);


  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy>("joy", 5, joystickHandler);

  ros::Subscriber subClearing = nh.subscribe<std_msgs::Float32>("map_clearing", 5, clearingHandler);

  ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("terrain_map", 2);

  for (int i = 0; i < terrainVoxelNum; i++)
  {
    terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }

  downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status)
  {
    ros::spinOnce();

    /*对地形点云进行分割，分为不同的地形体素（terrain voxel）。
     *对每个地形体素进行更新和处理，可能包括下采样、清除、滤波等操作。
     *对地形点云进行分析，估计地面高度和计算每个点的相对高度。
     *将处理后的地形点云发布到指定的ROS话题中。
     */
    if (newlaserCloud)
    {
      newlaserCloud = false;

      /*点云滚动更新
       *这段代码片段是地面体素的滚动更新，其目的是根据车辆的当前位置调整地面体素
       *的布局。这种滚动更新确保车辆始终位于地面体素网格的中心。地面体素网格以车
       *辆当前位置为中心，并根据车辆的移动而滚动。这有助于更好地跟踪地形的变化。
       */
      float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;

      // 计算车辆当前位置相对于地面体素中心的偏移
      // 车辆位置X-地面体素中心X < 负的一个体素网格大小，地面体素向车辆位置X的方向移动
      while (vehicleX - terrainVoxelCenX < -terrainVoxelSize)
      {
        for (int indY = 0; indY < terrainVoxelWidth; indY++)
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY];
          for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--)
          {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] = terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
          }
          terrainVoxelCloud[indY] = terrainVoxelCloudPtr;
          terrainVoxelCloud[indY]->clear();
        }
        terrainVoxelShiftX--;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      }

      // 车辆位置X-地面体素中心X > 正的一个体素网格大小，地面体素向车辆位置X的相反方向滚动
      while (vehicleX - terrainVoxelCenX > terrainVoxelSize)
      {
        for (int indY = 0; indY < terrainVoxelWidth; indY++)
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud[indY];
          for (int indX = 0; indX < terrainVoxelWidth - 1; indX++)
          {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] = terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
          }
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]->clear();
        }
        terrainVoxelShiftX++;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      }

      // 车辆位置Y-地面体素中心Y < 负的一个体素网格大小，地面体素沿Y轴方向移动
      while (vehicleY - terrainVoxelCenY < -terrainVoxelSize)
      {
        for (int indX = 0; indX < terrainVoxelWidth; indX++)
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)];
          for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--)
          {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] = terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
          }
          terrainVoxelCloud[terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * indX]->clear();
        }
        terrainVoxelShiftY--;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      }

      // 车辆位置Y-地面体素中心Y > 正的一个体素网格大小，地面体素沿Y轴反方向滚动
      while (vehicleY - terrainVoxelCenY > terrainVoxelSize)
      {
        for (int indX = 0; indX < terrainVoxelWidth; indX++)
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud[terrainVoxelWidth * indX];
          for (int indY = 0; indY < terrainVoxelWidth - 1; indY++)
          {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] = terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
          }
          terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]->clear();
        }
        terrainVoxelShiftY++;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      }

      // 激光雷达的点云栈
      pcl::PointXYZI point;
      int laserCloudCropSize = laserCloudCrop->points.size();

      // 用于将激光点云数据分配给地面体素，以便估计地面高度并计算每个点的相对高度
      for (int i = 0; i < laserCloudCropSize; i++)
      {
        point = laserCloudCrop->points[i];

        // 计算点云所在的体素
        int indX = int((point.x - vehicleX + terrainVoxelSize / 2) / terrainVoxelSize) + terrainVoxelHalfWidth;
        int indY = int((point.y - vehicleY + terrainVoxelSize / 2) / terrainVoxelSize) + terrainVoxelHalfWidth;

        // 如果是负半轴上，值应该减少1 
        if (point.x - vehicleX + terrainVoxelSize / 2 < 0)
          indX--;
        if (point.y - vehicleY + terrainVoxelSize / 2 < 0)
          indY--;

        if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 && indY < terrainVoxelWidth)
        {
          // 填充地面点云
          terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point);
          terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;
        }
      }

      // 对激光雷达数据进行处理，并重置地面体素网格
      for (int ind = 0; ind < terrainVoxelNum; ind++)
      {
        /**
         * @brief 处理激光雷达数据，重置地面体素网格
         * 判断条件1： 同一个位置的雷达点数 > 100
         * 判断条件2： id数据的时间差大于时间阈值
         * 判断条件3： 清除激光雷达数据标志位为true
         */
        if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre || laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >= voxelTimeUpdateThre || clearingCloud)
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud[ind];

          laserCloudDwz->clear();
          downSizeFilter.setInputCloud(terrainVoxelCloudPtr);
          downSizeFilter.filter(*laserCloudDwz);

          terrainVoxelCloudPtr->clear();
          int laserCloudDwzSize = laserCloudDwz->points.size();
          for (int i = 0; i < laserCloudDwzSize; i++)
          {
            point = laserCloudDwz->points[i];
            float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));

            /* 对于激光雷达数据的滤波
             * 在体素栅格中，需要被进行地面分割的点云满足以下要求，这些点云会被放入terrainCloud,用于地面分割
             * 点云高度大于最小阈值
             * 点云高度小于最大阈值
             * 当前点云的时间与要处理的点云时间差小于阈值 decayTime，或者距离小于 noDecayDis
             * 此时不会清除距离外的点云，或者不在需要被清除的距离之内
             */
            if (point.z - vehicleZ > minRelZ - disRatioZ * dis && point.z - vehicleZ < maxRelZ + disRatioZ * dis && (laserCloudTime - systemInitTime - point.intensity < decayTime 
                || dis < noDecayDis) && !(dis < clearingDis && clearingCloud))
            {
              terrainVoxelCloudPtr->push_back(point);
            }
          }

          terrainVoxelUpdateNum[ind] = 0;
          terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;
        }
      }

      terrainCloud->clear();

      // 加载中心附近的11x11组点云到terrainCloud
      for (int indX = terrainVoxelHalfWidth - 5; indX <= terrainVoxelHalfWidth + 5; indX++)
      {
        for (int indY = terrainVoxelHalfWidth - 5; indY <= terrainVoxelHalfWidth + 5; indY++)
        {
          *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];
        }
      }

      // 估算地面并计算每个点的相对于估计地面的高程
      for (int i = 0; i < planarVoxelNum; i++)
      {
        planarVoxelElev[i] = 0;
        planarVoxelEdge[i] = 0;
        planarVoxelDyObs[i] = 0;
        planarPointElev[i].clear();
      }
      
      int terrainCloudSize = terrainCloud->points.size();
      for (int i = 0; i < terrainCloudSize; i++)
      {
        // 地面点云的点
        point = terrainCloud->points[i];

        // 计算该点所在的体素网格的位置
        int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;
        int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;

        // 如果在负半轴的话减1
        if (point.x - vehicleX + planarVoxelSize / 2 < 0)
          indX--;
        if (point.y - vehicleY + planarVoxelSize / 2 < 0)
          indY--;

        // 检测点的高度是不是在[minRelZ, maxRelZ]之间，然后检测[indX,indY]周围的3x3的体素网格，填充到planarPointElev
        if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ)
        { 
          // 3个体素
          for (int dX = -1; dX <= 1; dX++)
          {
            // 3个体素
            for (int dY = -1; dY <= 1; dY++)
            {
              if (indX + dX >= 0 && indX + dX < planarVoxelWidth && indY + dY >= 0 && indY + dY < planarVoxelWidth)
              {
                planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY].push_back(point.z);
              }
            }
          }
        }

        // 判断是否清除动态障碍物
        if (clearDyObs)
        {
          // 计算相对位置
          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 && indY < planarVoxelWidth)
          {
            float pointX1 = point.x - vehicleX;
            float pointY1 = point.y - vehicleY;
            float pointZ1 = point.z - vehicleZ;

            float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);

            // 如果相对位置大于最小动态障碍物距离
            if (dis1 > minDyObsDis)
            {
              // 计算动态障碍物到车辆的角度
              float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / PI;
              // 如果相对位置大于最小动态障碍物角度
              if (angle1 > minDyObsAngle)
              {
                // 将点旋转到车辆坐标系
                // 绕z轴旋转Yaw角度
                float pointX2 = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
                float pointY2 = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
                float pointZ2 = pointZ1;

                // 绕Y轴旋转Pitch角度
                float pointX3 = pointX2 * cosVehiclePitch - pointZ2 * sinVehiclePitch;
                float pointY3 = pointY2;
                float pointZ3 = pointX2 * sinVehiclePitch + pointZ2 * cosVehiclePitch;

                // 绕x轴旋转Roll角度
                float pointX4 = pointX3;
                float pointY4 = pointY3 * cosVehicleRoll + pointZ3 * sinVehicleRoll;
                float pointZ4 = -pointY3 * sinVehicleRoll + pointZ3 * cosVehicleRoll;

                // 计算点在车辆坐标系中的极坐标，包括距离 dis4 和高度角 angle4
                float dis4 = sqrt(pointX4 * pointX4 + pointY4 * pointY4);
                float angle4 = atan2(pointZ4, dis4) * 180.0 / PI;

                // 如果 angle4 大于 minDyObsVFOV（最小垂直视场角）并且小于 maxDyObsVFOV（最大垂直视场角），则该点被标记为动态障碍物
                if (angle4 > minDyObsVFOV && angle4 < maxDyObsVFOV)
                {
                  // 动态障碍物增加
                  planarVoxelDyObs[planarVoxelWidth * indX + indY]++;
                }
              }
            }
            else
            {
              planarVoxelDyObs[planarVoxelWidth * indX + indY] += minDyObsPointNum;
            }
          }
        }
      }

      // 同上
      if (clearDyObs)
      {
        for (int i = 0; i < laserCloudCropSize; i++)
        {
          point = laserCloudCrop->points[i];

          int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;
          int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
            indY--;

          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 && indY < planarVoxelWidth)
          {
            float pointX1 = point.x - vehicleX;
            float pointY1 = point.y - vehicleY;
            float pointZ1 = point.z - vehicleZ;

            float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
            float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / PI;
            if (angle1 > minDyObsAngle)
            {
              planarVoxelDyObs[planarVoxelWidth * indX + indY] = 0;
            }
          }
        }
      }

      /*首先，代码检查是否启用了排序（useSorting标志）。
        如果启用了排序，它会遍历每个平面体素（planarVoxelNum）：
          a. 获取每个平面体素中保存的点的数量（planarPointElevSize）。
          b. 如果该平面体素内有点，它将对保存的高程值进行排序。
          c. 根据分位数（quantileZ）选择一个高程值，该分位数定义了估算高程的分位点。
          d. 如果选定的高程值大于某个阈值（maxGroundLift）加上最低点的高程值，并且开启了限制地面高程的标志（limitGroundLift），
             则将平面体素的高程设置为最低点的高程值加上阈值，否则将其设置为选定的高程值。

        如果没有启用排序，它将继续遍历每个平面体素：
          a. 获取每个平面体素中保存的点的数量（planarPointElevSize）。
          b. 然后它会查找该平面体素内的最低点（高程最小的点）。
          c. 并将平面体素的高程设置为找到的最低点的高程。
       */ 
      if (useSorting)
      {
        for (int i = 0; i < planarVoxelNum; i++)
        {
          int planarPointElevSize = planarPointElev[i].size();

          if (planarPointElevSize > 0)
          {
            sort(planarPointElev[i].begin(), planarPointElev[i].end());

            int quantileID = int(quantileZ * planarPointElevSize);
            if (quantileID < 0)
              quantileID = 0;
            else if (quantileID >= planarPointElevSize)
              quantileID = planarPointElevSize - 1;

            if (planarPointElev[i][quantileID] > planarPointElev[i][0] + maxGroundLift && limitGroundLift)
            {
              planarVoxelElev[i] = planarPointElev[i][0] + maxGroundLift;
            }
            else
            {
              planarVoxelElev[i] = planarPointElev[i][quantileID];
            }
          }
        }
      }
      else
      {
        for (int i = 0; i < planarVoxelNum; i++)
        {
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize > 0)
          {
            float minZ = 1000.0;
            int minID = -1;
            for (int j = 0; j < planarPointElevSize; j++)
            {
              if (planarPointElev[i][j] < minZ)
              {
                minZ = planarPointElev[i][j];
                minID = j;
              }
            }

            if (minID != -1)
            {
              planarVoxelElev[i] = planarPointElev[i][minID];
            }
          }
        }
      }

      /*这段代码用于从点云中提取地面点，并进行一些条件检查来筛选符合地面条件的点。
       *它通常用于环境建模和地面估计中，以便更好地识别地面上的特征和物体。
       */
      terrainCloudElev->clear();
      int terrainCloudElevSize = 0;
      for (int i = 0; i < terrainCloudSize; i++)
      {
        point = terrainCloud->points[i];
        if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ)
        {
          int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;
          int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
            indY--;

          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 && indY < planarVoxelWidth)
          {
            // 平面体素中动态障碍的数量如果低于 minDyObsPointNum 或者 没有清除动态障碍的操作
            if (planarVoxelDyObs[planarVoxelWidth * indX + indY] < minDyObsPointNum || !clearDyObs)
            {
              // 计算当前点与平面体素估算的高程之间的差值
              float disZ = point.z - planarVoxelElev[planarVoxelWidth * indX + indY]; // 设置的一定阈值或者最小值加阈值

              // 是否考虑下降
              if (considerDrop)
                disZ = fabs(disZ);

              // 表示平面体素内的点的数量
              int planarPointElevSize = planarPointElev[planarVoxelWidth * indX + indY].size();
              // 高程差值（或绝对值）在 0 和车辆高度（vehicleHeight）之间。
              // 平面体素内的点数不低于 minBlockPointNum。
              if (disZ >= 0 && disZ < vehicleHeight && planarPointElevSize >= minBlockPointNum)
              {
                // 该点被判定为地面点
                terrainCloudElev->push_back(point);
                terrainCloudElev->points[terrainCloudElevSize].intensity = disZ; // 设置为高程差

                terrainCloudElevSize++;
              }
            }
          }
        }
      }

      /*这部分代码的目的是在检测到没有障碍物数据（noDataObstacle 为真）
       *且初始化完成（noDataInited == 2）的情况下，处理平面体素数组中
       *的边缘体素。
       */
      if (noDataObstacle && noDataInited == 2)
      {
        // 遍历平面体素数组中的每个体素
        for (int i = 0; i < planarVoxelNum; i++)
        {
          /*如果点数小于 minBlockPointNum，则将 planarVoxelEdge[i] 设置为 1，表示这是一个边缘体素*/
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize < minBlockPointNum)
          {
            planarVoxelEdge[i] = 1;
          }
        }

        // 进一步处理边缘体素，多次执行以扩展边缘体素的范围。
        for (int noDataBlockSkipCount = 0; noDataBlockSkipCount < noDataBlockSkipNum; noDataBlockSkipCount++)
        {
          for (int i = 0; i < planarVoxelNum; i++)
          {
            // 遍历平面体素数组中的每个平面体素，并检查是否是边缘体素
            if (planarVoxelEdge[i] >= 1)
            {
              int indX = int(i / planarVoxelWidth);
              int indY = i % planarVoxelWidth;
              bool edgeVoxel = false;

              /*对于每个可能的边缘体素，它查找其周围的体素，检查是否有比当前体素 planarVoxelEdge[i] 更小的
               *planarVoxelEdge 值。如果没有找到这样的体素，说明该体素是边缘体素的外边缘，它将 planarVoxelEdge[i] 
               *递增
               */
              for (int dX = -1; dX <= 1; dX++)
              {
                for (int dY = -1; dY <= 1; dY++)
                {
                  if (indX + dX >= 0 && indX + dX < planarVoxelWidth && indY + dY >= 0 && indY + dY < planarVoxelWidth)
                  {
                    if (planarVoxelEdge[planarVoxelWidth * (indX + dX) + indY + dY] < planarVoxelEdge[i])
                    {
                      edgeVoxel = true;
                    }
                  }
                }
              }

              if (!edgeVoxel)
                planarVoxelEdge[i]++;
            }
          }
        }

        /*在处理完边缘体素后，它再次遍历平面体素数组，检查每个体素的 planarVoxelEdge 值。
         *如果 planarVoxelEdge[i] 大于 noDataBlockSkipNum，表示这是一个边缘体素。
         */
        for (int i = 0; i < planarVoxelNum; i++)
        {
          if (planarVoxelEdge[i] > noDataBlockSkipNum)
          {
            /**
             * 对于每个被标记为边缘体素的体素，根据其索引计算出其三维坐标（point.x, point.y, point.z，
             * 并将高程（intensity）设置为车辆高度。然后，根据体素的坐标，生成一个方形区域的四个点，这四
             * 个点代表了边缘体素的范围，并将这些点添加到地面点云 terrainCloudElev 中。
             */
            int indX = int(i / planarVoxelWidth);
            int indY = i % planarVoxelWidth;

            point.x = planarVoxelSize * (indX - planarVoxelHalfWidth) + vehicleX;
            point.y = planarVoxelSize * (indY - planarVoxelHalfWidth) + vehicleY;
            point.z = vehicleZ;
            point.intensity = vehicleHeight;

            point.x -= planarVoxelSize / 4.0;
            point.y -= planarVoxelSize / 4.0;
            terrainCloudElev->push_back(point);

            point.x += planarVoxelSize / 2.0;
            terrainCloudElev->push_back(point);

            point.y += planarVoxelSize / 2.0;
            terrainCloudElev->push_back(point);

            point.x -= planarVoxelSize / 2.0;
            terrainCloudElev->push_back(point);
          }
        }
      }

      clearingCloud = false;

      // 发布具有高程信息的点
      sensor_msgs::PointCloud2 terrainCloud2;
      pcl::toROSMsg(*terrainCloudElev, terrainCloud2);
      terrainCloud2.header.stamp = ros::Time().fromSec(laserCloudTime);
      terrainCloud2.header.frame_id = "map";
      pubLaserCloud.publish(terrainCloud2);
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
