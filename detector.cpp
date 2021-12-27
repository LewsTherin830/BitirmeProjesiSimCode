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
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

double getDistance(double x1, double y1, double x2, double y2);
void moveGoal();
void avoidObject();
void stopTheSlave();

const double PI = 3.141592654;

float myX;
float myY;

double roll,pitch,yaw;

float masterPosX;
float masterPosY;

float goalPoseX, goalPoseY;
float goalX,goalY;

float distanceToGoal;
double masterYaw;

float frontDistance;
float sideDistance;
float distanceMoved = 0;

int stage = 1; 
bool keepTrackMaster = true;

float startingX;
float startingY;

float endingX;
float endingY;

float tempGoalY;
float k;
float angularSpeed;


//LEFT Slave
ros::Publisher slaveSpeedPublisher;

int main(int argc, char**argv) {
  ros::init(argc, argv, "Slave1");
  ros::NodeHandle nh;
  ros::Subscriber masterListener = nh.subscribe("masterOdom", 1, masterListenerCallback);
  ros::Subscriber myOdom = nh.subscribe("/robot2/odom", 1, odomCallback);
  ros::Subscriber formDecision = nh.subscribe("decision", 1, decisionCallback);
  ros::Subscriber laserScan = nh.subscribe("/robot2/scan",1,scanCallback);
  slaveSpeedPublisher = nh.advertise<geometry_msgs::Twist>("/robot2/cmd_vel",1);
  
  ros::Rate loopRate(5);
  while(nh.ok()){
    ros::spinOnce();
    loopRate.sleep();
  }
  return 0;
} 

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  // check the front distance to detect whether there is object or not
  frontDistance = msg->ranges[1]; //front range
  sideDistance = msg->ranges[100];
  if(frontDistance <= 1){
    keepTrackMaster = false;
  }
}


void masterListenerCallback(const geometry_msgs::Pose msg){
  //read the master data
  masterPosX = msg.position.x;
  masterPosY = msg.position.y;
  masterYaw = msg.position.z;
  //ROS_INFO("My Pos: (%f , %f)",myX, myY);
  //ROS_INFO("My Yaw: %f",yaw);
  //ROS_INFO("Goal Pos: (%f , %f)",goalX, goalY);
  //ROS_INFO("Distance to Goal: %f",distanceToGoal);
  //ROS_INFO("Front Distance: %f",frontDistance);
  //ROS_INFO("Side Distance: %f",sideDistance);
  //ROS_INFO("Distance Moved: %f",distanceMoved);
  //ROS_INFO("------");
  if(keepTrackMaster){
    moveGoal();
  }
  else{
    avoidObject();
  }
  
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
  goalPoseY = msg.position.y;
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
  
  float theta = masterYaw - 1.57;
  
  goalX = cos(theta)*goalPoseX - sin(theta)*goalPoseY + masterPosX;
  goalY = sin(theta)*goalPoseX + cos(theta)*goalPoseY + masterPosY;
  
  
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
    cout << "Yaw is:" << yaw << "\n"; 
    cout << "Angular speed is: " << vel_msg.angular.z << "\n";
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


void stopTheSlave(){
  geometry_msgs::Twist vel_msg;
  vel_msg.linear.y = 0;
  vel_msg.linear.z = 0;
  vel_msg.linear.x = 0;
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  vel_msg.angular.z = 0;
  slaveSpeedPublisher.publish(vel_msg);
}

void avoidObject(){
  if(stage == 1){
    // +z sola donuyo -z saga donuyo
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.linear.x = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    if (yaw > 0.1){
      vel_msg.angular.z = -0.3;
      slaveSpeedPublisher.publish(vel_msg);
    }
    else{
      vel_msg.angular.z = 0;
      slaveSpeedPublisher.publish(vel_msg);
      startingX = myX;
      startingY = myY;
      stage += 1;
    }
  }
  else if(stage ==2){
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;
    if(sideDistance != INFINITY){
      vel_msg.linear.x = 0.3;
      slaveSpeedPublisher.publish(vel_msg);
    }
    else{
      vel_msg.linear.x = 0;
      slaveSpeedPublisher.publish(vel_msg);
      endingX = myX;
      endingY = myY;
      distanceMoved = getDistance(startingX,startingY,endingX,endingY);
      stage +=1;
    }
  }
  else if(stage ==3){
    // +z sola donuyo -z saga donuyo
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.linear.x = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    if (yaw < 1.6){
      vel_msg.angular.z = 0.3;
      slaveSpeedPublisher.publish(vel_msg);
    }
    else{
      vel_msg.angular.z = 0;
      slaveSpeedPublisher.publish(vel_msg);
      tempGoalY = myY + 1.5;
      stage += 1;
    }
  }
  else if(stage == 4){
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;
    vel_msg.angular.x = 0;
    
    if(myY < tempGoalY){
      vel_msg.linear.x = 0.2;
      slaveSpeedPublisher.publish(vel_msg);
    }
    else{
      vel_msg.linear.x = 0;
      slaveSpeedPublisher.publish(vel_msg);
      keepTrackMaster = true;
      stage =1;
    }
  }

}

// distance calculator
double getDistance(double x1, double y1, double x2, double y2){ //x1-y1 and x2-y2 are points to calculate distance between them
  double distance = sqrt(pow((x2-x1),2) + pow((y2-y1),2)); 
  return distance;
}
