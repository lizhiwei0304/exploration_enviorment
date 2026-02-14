/*
 * @Author: Lee lizw_0304@163.com
 * @Date: 2024-05-29 14:11:12
 * @LastEditors: Lee lizw_0304@163.com
 * @LastEditTime: 2024-06-09 11:48:11
 * @FilePath: /waypoint_rviz_plugin/include/waypoint_tool.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef WAYPOINT_RVIZ_PLUGIN_WAYPOINT_TOOL_H
#define WAYPOINT_RVIZ_PLUGIN_WAYPOINT_TOOL_H

#include <sstream>
#include <ros/ros.h>
#include <QObject>

#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"
#include "rviz/default_plugin/tools/pose_tool.h"

namespace rviz
{
class StringProperty;

class WaypointTool : public PoseTool
{
  Q_OBJECT
public:
  WaypointTool();
  virtual ~WaypointTool()
  {
  }
  virtual void onInitialize();

protected:
  virtual void odomHandler(const nav_msgs::Odometry::ConstPtr& odom);
  virtual void onPoseSet(double x, double y, double theta);

private Q_SLOTS:
  void updateTopic();

private:
  float vehicle_z;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher pub_joy_;

  StringProperty* topic_property_;
};
}

#endif  // WAYPOINT_RVIZ_PLUGIN_WAYPOINT_TOOL_H
