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

using namespace std;
void masterListenerCallback(const geometry_msgs::Pose msg);
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void decisionCallback(const geometry_msgs::Pose msg);

double getDistance(double x1, double y1, double x2, double y2);
void moveGoal();

const double PI = 3.141592654;

float myX;
float myY;

float goalX;
float goalY;

double roll,pitch,yaw;

float masterPosX;
float masterPosY;
double masterYaw;

float goalPoseX, goalPoseY;

float distanceToGoal;
float angularSpeed;

ros::Publisher slaveSpeedPublisher;

//RIGHT slave
int main(int argc, char**argv) {
  ros::init(argc, argv, "Slave2");
  ros::NodeHandle nh;
  ros::Subscriber masterListener = nh.subscribe("masterOdom", 1, masterListenerCallback);
  ros::Subscriber myOdom = nh.subscribe("/robot1/odom", 1, odomCallback);
  ros::Subscriber decision = nh.subscribe("decision", 1, decisionCallback);
  slaveSpeedPublisher = nh.advertise<geometry_msgs::Twist>("/robot1/cmd_vel",1);
  ros::Rate loopRate(5);
  while(nh.ok()){
    ros::spinOnce();
    loopRate.sleep();
  }
  return 0;
} 

void masterListenerCallback(const geometry_msgs::Pose msg){
  //read the master data
  masterPosX = msg.position.x;
  masterPosY = msg.position.y;
  masterYaw = msg.position.z;
  //ROS_INFO("My Yaw: %f",yaw);
  //ROS_INFO("Goal Pos: (%f , %f)",goalX, goalY);
  //ROS_INFO("Distance to Goal: %f",distanceToGoal);
  //ROS_INFO("------");
  //ROS_INFO("My Pos: (%f , %f)",myX, myY);
  
  moveGoal();
  //publish odom data to slaves
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  //read the odom data
  myX = msg->pose.pose.position.x;
  myY = msg->pose.pose.position.y;

  geometry_msgs::Quaternion orientation_q;
  orientation_q = msg->pose.pose.orientation;

  tf::Quaternion RQ2;
  tf::quaternionMsgToTF(orientation_q,RQ2);
  tf::Matrix3x3(RQ2).getRPY(roll,pitch,yaw);
}

void decisionCallback(const geometry_msgs::Pose msg){
  goalPoseX = msg.position.x;
  if (goalPoseX == 0){
    goalPoseY = 2 * msg.position.y;
  }
  else{
    goalPoseY = msg.position.y;
  }
  
}

void moveGoal(){
  geometry_msgs::Twist vel_msg;
  vel_msg.linear.y = 0;
  vel_msg.linear.z = 0;
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;

  if(goalPoseX == 0 && goalPoseY == 0){
    goalPoseX = -2;
    goalPoseY = 0;
  }

  goalX = masterPosX - goalPoseX;
  goalY = masterPosY + goalPoseY;
  

  distanceToGoal = getDistance(myX, myY, goalX, goalY);
  // set speed for goal
  float linearSpeedGain;
  float angularSpeedGain;
  if(distanceToGoal > 1.5){
    linearSpeedGain = 0.4;
  }
  else if(distanceToGoal < 1.5 && distanceToGoal > 0.5){
    linearSpeedGain = 0.3;
  }
  else{
    linearSpeedGain = 0.2;
  }

  // go to goal
  if (distanceToGoal > 0.2) { 
    vel_msg.linear.x = linearSpeedGain;
    angularSpeed = atan2(goalY - myY, goalX - myX) - yaw;
    cout << "First Angular Speed:" << angularSpeed << "\n";
    if(angularSpeed < -3.14 ){
      angularSpeed = 6.28 - angularSpeed;
      angularSpeedGain = 0.20;
    }
    else{
      angularSpeedGain = 1;
    }
    
    vel_msg.angular.z = angularSpeedGain * angularSpeed;
    
    cout << "Yaw is: " << yaw << "\n"; 
    cout << "Angular speed is: " << (atan2(goalY - myY, goalX - myX) - yaw) << "\n";
    cout << "Y diff:" << goalY - myY << "\n";
    cout << "X diff:" << goalX - myX << "\n";
    cout << "------" << "\n";
  }
  else if (masterYaw >= 0 && masterYaw - yaw  > 0.1){
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0.5;
  }
  else if (masterYaw >= 0 && yaw - masterYaw > 0.1){
    vel_msg.linear.x = 0;
    vel_msg.angular.z = -0.5;
  } 
  else{
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
  }
  slaveSpeedPublisher.publish(vel_msg);
}

// distance calculator
double getDistance(double x1, double y1, double x2, double y2){ //x1-y1 and x2-y2 are points to calculate distance between them
  double distance = sqrt(pow((x2-x1),2) + pow((y2-y1),2)); 
  return distance;
}
