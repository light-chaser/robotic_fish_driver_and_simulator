#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include<gazebo_msgs/ModelStates.h>


void poseCallback(const gazebo_msgs::ModelStates &pose){
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "base_link";
  transformStamped.transform.translation.x = pose.pose[1].position.x;
  transformStamped.transform.translation.y = pose.pose[1].position.y;
  transformStamped.transform.translation.z = pose.pose[1].position.z;

  transformStamped.transform.rotation.x = pose.pose[1].orientation.x;
  transformStamped.transform.rotation.y = pose.pose[1].orientation.y;
  transformStamped.transform.rotation.z = pose.pose[1].orientation.z;
  transformStamped.transform.rotation.w = pose.pose[1].orientation.w;

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_broadcaster");

  ros::NodeHandle private_node("naro");
 
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("gazebo/model_states", 10, &poseCallback);

  ros::spin();
  return 0;
};