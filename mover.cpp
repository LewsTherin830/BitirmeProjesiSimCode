#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <cmath>
#include <vector>
#include <list>
#include <string>
#include <boost/thread/mutex.hpp>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>

using namespace std;
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

// master Pos
float masterX;
float masterY;
// master angle
double masterYaw,masterPitch,masterRoll;
// master pos+angle publisher x -> x y -> y z -> angle
ros::Publisher masterPosPublisher;


int main(int argc, char**argv) {
  ros::init(argc, argv, "Master");
  ros::NodeHandle nh;
  // master listens its pos + LIDAR
  ros::Subscriber p_listener = nh.subscribe("/odom",1,odomCallback);
  ros::Subscriber scan = nh.subscribe("/scan",1,scanCallback);
  // master publishes its pos+angle to slaves
  masterPosPublisher = nh.advertise<const geometry_msgs::Pose>("masterOdom",1);

  // we work at 5Hz, can be increased or decreased later 
  ros::Rate looprate(5);
  while(nh.ok()){
    ros::spinOnce();
    ROS_INFO("My Pos: (%f , %f)",masterX, masterY);
    ROS_INFO("My Yaw: %f",masterYaw);
    ROS_INFO("------");
    looprate.sleep();
  } 
  return 0;
} 

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  
  // create a array to store values
  geometry_msgs::Pose masterPos;
  
  // get the values for pos 
  masterX = msg->pose.pose.position.x;
  masterY = msg->pose.pose.position.y;

  // save the values 
  masterPos.position.x = masterX;
  masterPos.position.y = masterY;

  // get the values for angle and save them
  geometry_msgs::Quaternion orientation_q;
  orientation_q = msg->pose.pose.orientation;

  // perform necessary transformation  
  tf::Quaternion RQ2;
  tf::quaternionMsgToTF(orientation_q,RQ2);
  tf::Matrix3x3(RQ2).getRPY(masterRoll,masterPitch,masterYaw);
  
  // save the values 
  masterPos.position.z = masterYaw;
  
  // publish with slaves
  masterPosPublisher.publish(masterPos);

}

// to be used at later stages for LIDAR implementation
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

}
