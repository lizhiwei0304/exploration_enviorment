#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <queue>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

const double PI = 3.1415926;

double scanVoxelSize = 0.1;           // 扫描体素大小: 10cm
double decayTime = 10.0;              // 时间阈值：10.0s
double noDecayDis = 0;                // 车辆初始距离阈值：0.0m
double clearingDis = 30.0;            // 清除距离：30.0m
bool clearingCloud = false;           // 清除点云：否-false；是-true
bool useSorting = false;              // 使用排序：否
double quantileZ = 0.25;              // Z轴分辩数：0.25m
double vehicleHeight = 1.5;           // 车辆的高度
int voxelPointUpdateThre = 100;       // 同一个体素网格的雷达点数阈值
double voxelTimeUpdateThre = 2.0;     // 同一个体素网格的雷达点时间阈值
double lowerBoundZ = -1.5;            // Z轴最小的相对距离
double upperBoundZ = 1.0;             // Z轴最小的相对距离
double disRatioZ = 0.1;               // 点云处理的高度与距离的比例-与激光雷达性能相关
bool checkTerrainConn = true;         // 是否检测点云连通性
double terrainUnderVehicle = -0.75;   // 地面位于车辆下方-0.75m
double terrainConnThre = 0.5;         // 地面连通性阈值： 0.5m
double ceilingFilteringThre = 2.0;    // 天花板滤波阈值
double localTerrainMapRadius = 4.0;   // 局部地图地面点云地图范围

// 平面体素参数
float terrainVoxelSize = 2.0;                                       // 地面体素网格的尺寸： 2.0m
int terrainVoxelShiftX = 0;                                         // 地面体素网格翻转时的X位置
int terrainVoxelShiftY = 0;                                         // 地面体素网格翻转时的Y位置
const int terrainVoxelWidth = 41;                                   // 地面体素的宽度：   41
int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;            // 地面体素的半宽度： 20
const int terrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth;  // 地面体素的多少：   41×41 = 1681

// 规划体素参数
float planarVoxelSize = 0.4;                                        // 规划区域体素网格的尺寸大小 0.4m
const int planarVoxelWidth = 101;                                   // 体素网格的宽度：101
int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2;              // 体素网格的半宽：50
const int planarVoxelNum = planarVoxelWidth * planarVoxelWidth;     // 体素网格的多少：10201

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudElev(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudLocal(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[terrainVoxelNum];

int terrainVoxelUpdateNum[terrainVoxelNum] = {0};       // 每个体素网格网格点的点云更新数量
float terrainVoxelUpdateTime[terrainVoxelNum] = {0};    // 每个体素网格网格点的更新时间
float planarVoxelElev[planarVoxelNum] = {0};            // 每个规划网格区域的高程信息
int planarVoxelConn[planarVoxelNum] = {0};              // 每个规划网格区域的连通性信息
vector<float> planarPointElev[planarVoxelNum];          // 每个规划网格的边缘信息
queue<int> planarVoxelQueue;                            // 规划体素的优先级队列

double laserCloudTime = 0;
bool newlaserCloud = false;

double systemInitTime = 0;
bool systemInited = false;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;          // 设置pcl的下采样参数
pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;                // KdTree的快速近似最近邻搜索

// 车辆状态估计回调函数
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
}

// lidar的点云回调函数
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
    if (pointZ - vehicleZ > lowerBoundZ - disRatioZ * dis && pointZ - vehicleZ < upperBoundZ + disRatioZ * dis && dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1))
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

// 局部地面点云的回调函数
void terrainCloudLocalHandler(const sensor_msgs::PointCloud2ConstPtr &terrainCloudLocal2)
{
  terrainCloudLocal->clear();
  pcl::fromROSMsg(*terrainCloudLocal2, *terrainCloudLocal);
}

// 手柄信息的回调函数
void joystickHandler(const sensor_msgs::Joy::ConstPtr &joy)
{
  if (joy->buttons[5] > 0.5)
  {
    clearingCloud = true;
  }
}

// 点云清除回调函数
void clearingHandler(const std_msgs::Float32::ConstPtr &dis)
{
  clearingDis = dis->data;
  clearingCloud = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "terrainAnalysisExt");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("scanVoxelSize", scanVoxelSize);
  nhPrivate.getParam("decayTime", decayTime);
  nhPrivate.getParam("noDecayDis", noDecayDis);
  nhPrivate.getParam("clearingDis", clearingDis);
  nhPrivate.getParam("useSorting", useSorting);
  nhPrivate.getParam("quantileZ", quantileZ);
  nhPrivate.getParam("vehicleHeight", vehicleHeight);
  nhPrivate.getParam("voxelPointUpdateThre", voxelPointUpdateThre);
  nhPrivate.getParam("voxelTimeUpdateThre", voxelTimeUpdateThre);
  nhPrivate.getParam("lowerBoundZ", lowerBoundZ);
  nhPrivate.getParam("upperBoundZ", upperBoundZ);
  nhPrivate.getParam("disRatioZ", disRatioZ);
  nhPrivate.getParam("checkTerrainConn", checkTerrainConn);
  nhPrivate.getParam("terrainUnderVehicle", terrainUnderVehicle);
  nhPrivate.getParam("terrainConnThre", terrainConnThre);
  nhPrivate.getParam("ceilingFilteringThre", ceilingFilteringThre);
  nhPrivate.getParam("localTerrainMapRadius", localTerrainMapRadius);

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("state_estimation", 5, odometryHandler);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("registered_scan", 5, laserCloudHandler);
  // ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("registered_scan_filted", 5, laserCloudHandler);

  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy>("joy", 5, joystickHandler);

  ros::Subscriber subClearing = nh.subscribe<std_msgs::Float32>("cloud_clearing", 5, clearingHandler);

  ros::Subscriber subTerrainCloudLocal = nh.subscribe<sensor_msgs::PointCloud2>("terrain_map", 2, terrainCloudLocalHandler);

  ros::Publisher pubTerrainCloud = nh.advertise<sensor_msgs::PointCloud2>("terrain_map_ext", 2);

  for (int i = 0; i < terrainVoxelNum; i++)
  {
    terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }

  downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);

  std::vector<int> pointIdxNKNSearch;         // FLANN库中的一个概念，代表“最近的 K 个近似邻居搜索”
  std::vector<float> pointNKNSquaredDistance;

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status)
  {
    ros::spinOnce();

    if (newlaserCloud)
    {
      newlaserCloud = false;

      // 地面点云翻滚更新，利用记录下来的偏移量terrainVoxelShiftX和terrainVoxelShiftY来更新地面中心与车辆位置相同
      float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;

      while (vehicleX - terrainVoxelCenX < -terrainVoxelSize)
      {
        for (int indY = 0; indY < terrainVoxelWidth; indY++)
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY];
          for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--)
          {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
          }
          terrainVoxelCloud[indY] = terrainVoxelCloudPtr;
          terrainVoxelCloud[indY]->clear();
        }
        terrainVoxelShiftX--;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      }

      while (vehicleX - terrainVoxelCenX > terrainVoxelSize)
      {
        for (int indY = 0; indY < terrainVoxelWidth; indY++)
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud[indY];
          for (int indX = 0; indX < terrainVoxelWidth - 1; indX++)
          {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
          }
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]->clear();
        }
        terrainVoxelShiftX++;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      }

      while (vehicleY - terrainVoxelCenY < -terrainVoxelSize)
      {
        for (int indX = 0; indX < terrainVoxelWidth; indX++)
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)];
          for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--)
          {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
          }
          terrainVoxelCloud[terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * indX]->clear();
        }
        terrainVoxelShiftY--;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      }

      while (vehicleY - terrainVoxelCenY > terrainVoxelSize)
      {
        for (int indX = 0; indX < terrainVoxelWidth; indX++)
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud[terrainVoxelWidth * indX];
          for (int indY = 0; indY < terrainVoxelWidth - 1; indY++)
          {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
          }
          terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]->clear();
        }
        terrainVoxelShiftY++;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      }

      // 原始lidar点云栈，将原始点云的点填充到变量中
      pcl::PointXYZI point;
      int laserCloudCropSize = laserCloudCrop->points.size();
      for (int i = 0; i < laserCloudCropSize; i++)
      {
        point = laserCloudCrop->points[i];

        int indX = int((point.x - vehicleX + terrainVoxelSize / 2) / terrainVoxelSize) + terrainVoxelHalfWidth;
        int indY = int((point.y - vehicleY + terrainVoxelSize / 2) / terrainVoxelSize) + terrainVoxelHalfWidth;

        if (point.x - vehicleX + terrainVoxelSize / 2 < 0)
          indX--;
        if (point.y - vehicleY + terrainVoxelSize / 2 < 0)
          indY--;

        if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 && indY < terrainVoxelWidth)
        {
          terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point);
          terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;
        }
      }
      /*
      开始
      循环（对每个地形体素）：
          |
          |--- 条件检查：
          |     |
          |     |-- terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre
          |     |-- laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >= voxelTimeUpdateThre
          |     |-- clearingCloud
          |
          |--- 如果任一条件为真：
          |     |
          |     |-- 获取当前地形体素的点云数据指针 terrainVoxelCloudPtr
          |     |
          |     |-- 创建新的点云容器 laserCloudDwz 并清空
          |     |
          |     |-- 使用下采样滤波器对 terrainVoxelCloudPtr 进行下采样，将结果存储在 laserCloudDwz 中
          |     |
          |     |-- 清空当前地形体素的点云数据 terrainVoxelCloudPtr
          |     |
          |     |-- 获取 laserCloudDwz 的点数 laserCloudDwzSize
          |     |
          |     |-- 循环（对每个点 in laserCloudDwz）：
          |     |     |
          |     |     |-- 计算点到车辆位置的距离 dis
          |     |     |
          |     |     |-- 条件检查：
          |     |     |     |
          |     |     |     |-- 检查点的高度是否在范围内
          |     |     |     |-- 检查点的时间戳是否在范围内
          |     |     |     |-- 检查是否需要清除点云数据
          |     |     |
          |     |     |-- 如果上述条件满足：
          |     |     |     |
          |     |     |     |-- 将点添加到当前地形体素的点云数据 terrainVoxelCloudPtr 中
          |     |     |
          |     |
          |     |-- 重置 terrainVoxelUpdateNum[ind] 为0
          |     |
          |     |-- 更新 terrainVoxelUpdateTime[ind] 为 laserCloudTime - systemInitTime
          |
          |
      结束

      */
      for (int ind = 0; ind < terrainVoxelNum; ind++)
      {
        if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre || laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >= voxelTimeUpdateThre || clearingCloud)
        {
          // 将体素ID为ind的点云提取出来
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
            if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis &&  point.z - vehicleZ < upperBoundZ + disRatioZ * dis &&
                (laserCloudTime - systemInitTime - point.intensity < decayTime || dis < noDecayDis) && !(dis < clearingDis && clearingCloud))
            {
              terrainVoxelCloudPtr->push_back(point);
            }
          }

          terrainVoxelUpdateNum[ind] = 0;
          terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;
        }
      }

      // 添加新的地面点云，之前的来自有terrain_map话题
      terrainCloud->clear();
      for (int indX = terrainVoxelHalfWidth - 10; indX <= terrainVoxelHalfWidth + 10; indX++)
      {
        for (int indY = terrainVoxelHalfWidth - 10; indY <= terrainVoxelHalfWidth + 10; indY++)
        {
          *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];
        }
      }

      // 估算地面和每个点的高程信息
      for (int i = 0; i < planarVoxelNum; i++)
      {
        planarVoxelElev[i] = 0;
        planarVoxelConn[i] = 0;
        planarPointElev[i].clear();
      }

      /*开始
      循环（对每个地形点 in terrainCloud）:
          |
          |-- 获取当前地形点的坐标 point
          |
          |-- 计算点到车辆位置的距离 dis
          |
          |-- 条件检查：
          |     |
          |     |-- 检查点的高度是否在范围内
          |
          |-- 如果上述条件满足:
          |     |
          |     |-- 计算地形点在平面体素网格中的索引 indX 和 indY
          |     |
          |     |-- 条件检查：
          |     |     |
          |     |     |-- 检查是否需要向下取整索引 indX 和 indY
          |     |
          |     |-- 循环（对每个相邻的体素 dX 和 dY）:
          |     |     |
          |     |     |-- 条件检查：
          |     |     |     |
          |     |     |     |-- 检查体素索引是否在范围内
          |     |     |
          |     |     |-- 如果上述条件满足:
          |     |     |     |
          |     |     |     |-- 将当前地形点的高度值添加到平面体素网格的 elev 列表中
          |
          |
      结束
      */

      int terrainCloudSize = terrainCloud->points.size();
      for (int i = 0; i < terrainCloudSize; i++)
      {
        point = terrainCloud->points[i];
        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));
        if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis && point.z - vehicleZ < upperBoundZ + disRatioZ * dis)
        {
          int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;
          int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
            indY--;

          for (int dX = -1; dX <= 1; dX++)
          {
            for (int dY = -1; dY <= 1; dY++)
            {
              if (indX + dX >= 0 && indX + dX < planarVoxelWidth && indY + dY >= 0 && indY + dY < planarVoxelWidth)
              {
                // 存储着符合上述条件的[indX， indY]的周围九个点的z轴高度
                planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY].push_back(point.z);
              }
            }
          }
        }
      }
      /*开始
      条件检查：
      |
      |-- 如果 useSorting 为真:
      |     |
      |     |-- 循环（对每个平面体素 i）:
      |     |     |
      |     |     |-- 获取当前平面体素的高度列表 planarPointElev[i] 的大小 planarPointElevSize
      |     |     |
      |     |     |-- 条件检查：
      |     |     |     |
      |     |     |     |-- 如果 planarPointElevSize 大于0:
      |     |     |     |     |
      |     |     |     |     |-- 对 planarPointElev[i] 中的高度值列表进行排序
      |     |     |     |
      |     |     |     |-- 计算高度列表的分位数索引 quantileID
      |     |     |     |
      |     |     |     |-- 条件检查：
      |     |     |     |     |
      |     |     |     |     |-- 纠正 quantileID 以确保它在有效范围内
      |     |     |     |
      |     |     |     |-- 将平面体素的高度 planarVoxelElev[i] 设置为对应分位数的高度值
      |     |
      |
      |-- 否则（useSorting 为假）:
      |     |
      |     |-- 循环（对每个平面体素 i）:
      |     |     |
      |     |     |-- 获取当前平面体素的高度列表 planarPointElev[i] 的大小 planarPointElevSize
      |     |     |
      |     |     |-- 条件检查：
      |     |     |     |
      |     |     |     |-- 如果 planarPointElevSize 大于0:
      |     |     |     |     |
      |     |     |     |     |-- 初始化最小高度值 minZ 为一个大值，最小高度值索引 minID 为-1
      |     |     |     |
      |     |     |     |-- 循环（对高度列表中的每个高度值 j）:
      |     |     |     |     |
      |     |     |     |     |-- 条件检查：
      |     |     |     |     |     |
      |     |     |     |     |     |-- 如果当前高度值小于 minZ:
      |     |     |     |     |     |     |
      |     |     |     |     |     |     |-- 更新 minZ 为当前高度值，minID 为 j
      |     |     |     |     |
      |     |     |     |
      |     |     |     |-- 条件检查：
      |     |     |     |     |
      |     |     |     |     |-- 如果 minID 不为-1:
      |     |     |     |     |     |
      |     |     |     |     |     |-- 将平面体素的高度 planarVoxelElev[i] 设置为最小高度值 minZ
      |
      结束
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

            planarVoxelElev[i] = planarPointElev[i][quantileID];
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


      // 检查地形的连通性以去除天花板
      // 如果 checkTerrainConn 为真:
      if (checkTerrainConn)
      {
        // 获取中心平面体素的索引 ind
        int ind = planarVoxelWidth * planarVoxelHalfWidth + planarVoxelHalfWidth; // 5100

        // 如果中心平面体素的高度值列表 planarPointElev[ind] 为空:
        if (planarPointElev[ind].size() == 0)
          // 将中心平面体素的高度值 planarVoxelElev[ind] 设置为车辆位置高度加上地形下方高度 terrainUnderVehicle,其实相对于车辆坐标来说就是中心位置减去一半车高
          planarVoxelElev[ind] = vehicleZ + terrainUnderVehicle;

        // 将中心平面体素的索引 ind 添加到平面体素队列 planarVoxelQueue 中
        planarVoxelQueue.push(ind);

        // 设置中心平面体素的连接状态为1，表示已处理
        planarVoxelConn[ind] = 1;

        // 循环（当平面体素队列不为空时）
        while (!planarVoxelQueue.empty())
        {
          // 获取队列的前端平面体素索引 front
          int front = planarVoxelQueue.front();

          // 设置前端平面体素的连接状态为2，表示已处理
          planarVoxelConn[front] = 2;

          // 从平面体素队列中移除前端平面体素
          planarVoxelQueue.pop();

          // 计算前端平面体素的二维索引 indX 和 indY
          int indX = int(front / planarVoxelWidth);
          int indY = front % planarVoxelWidth;

          // 循环（对前端平面体素周围的平面体素）
          for (int dX = -10; dX <= 10; dX++)
          {
            for (int dY = -10; dY <= 10; dY++)
            {
              // 如果邻近平面体素的二维索引有效
              if (indX + dX >= 0 && indX + dX < planarVoxelWidth && indY + dY >= 0 && indY + dY < planarVoxelWidth)
              {
                // 计算邻近平面体素的索引 ind
                ind = planarVoxelWidth * (indX + dX) + indY + dY;

                // 如果邻近平面体素未连接且具有高度值列表 planarPointElev[ind]
                if (planarVoxelConn[ind] == 0 && planarPointElev[ind].size() > 0)
                {
                  // 如果前端平面体素与邻近平面体素的高度值差小于 terrainConnThre:
                  if (fabs(planarVoxelElev[front] - planarVoxelElev[ind]) < terrainConnThre)
                  {
                    planarVoxelQueue.push(ind); // 将邻近平面体素添加到平面体素队列 planarVoxelQueue 中
                    planarVoxelConn[ind] = 1;   // 设置邻近平面体素的连接状态为1
                  }
                  else if (fabs(planarVoxelElev[front] - planarVoxelElev[ind]) > ceilingFilteringThre)
                  {
                    // 设置邻近平面体素的连接状态为-1
                    planarVoxelConn[ind] = -1;
                  }
                }
              }
            }
          }
        }
      }

      // 计算超出本地地形地图半径的地形地图
      terrainCloudElev->clear();
      int terrainCloudElevSize = 0;
      for (int i = 0; i < terrainCloudSize; i++)
      {
        point = terrainCloud->points[i];
        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));
        if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis && point.z - vehicleZ < upperBoundZ + disRatioZ * dis && dis > localTerrainMapRadius)
        {
          int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;
          int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
            indY--;

          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 && indY < planarVoxelWidth)
          {
            int ind = planarVoxelWidth * indX + indY;
            float disZ = fabs(point.z - planarVoxelElev[ind]);
            if (disZ < vehicleHeight && (planarVoxelConn[ind] == 2 || !checkTerrainConn))
            {
              terrainCloudElev->push_back(point);
              terrainCloudElev->points[terrainCloudElevSize].x = point.x;
              terrainCloudElev->points[terrainCloudElevSize].y = point.y;
              terrainCloudElev->points[terrainCloudElevSize].z = point.z;
              terrainCloudElev->points[terrainCloudElevSize].intensity = disZ;

              terrainCloudElevSize++;
            }
          }
        }
      }

      // 合并本地地形地图半径内的地形地图
      int terrainCloudLocalSize = terrainCloudLocal->points.size();
      for (int i = 0; i < terrainCloudLocalSize; i++)
      {
        point = terrainCloudLocal->points[i];
        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));
        if (dis <= localTerrainMapRadius)
        {
          terrainCloudElev->push_back(point);
        }
      }

      clearingCloud = false;

      // 发布带着高程信息的点云
      sensor_msgs::PointCloud2 terrainCloud2;
      pcl::toROSMsg(*terrainCloudElev, terrainCloud2);
      terrainCloud2.header.stamp = ros::Time().fromSec(laserCloudTime);
      terrainCloud2.header.frame_id = "map";
      pubTerrainCloud.publish(terrainCloud2);
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
