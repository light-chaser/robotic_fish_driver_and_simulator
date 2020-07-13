//于rviz中标记障碍物所在位置为一球体，半径R=0.5,于该球体范围内的其他障碍物视为一体//
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Range.h>
#include"../../../devel/include/cpg_driver_simulation/obstacle.h"
//tf2_ros::TransformListener listener;
cpg_driver_simulation::obstacle ost;
void posecallback(const cpg_driver_simulation::obstacle &range)
{
  ost=range;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_tf");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("obstacles", 10, &posecallback);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("marker", 10);
  visualization_msgs::Marker marker;
  while(ros::ok()){
  marker.header.frame_id = "camera";
  marker.header.stamp = ros::Time::now();
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = marker.ADD;
  marker.pose.position.x = ost.x;
  marker.pose.position.y = ost.y;
  marker.pose.position.z = ost.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = ost.R;
  marker.scale.y = ost.R;
  marker.scale.z = ost.R;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker_pub.publish(marker);
  
  ros::spinOnce();
  }
}