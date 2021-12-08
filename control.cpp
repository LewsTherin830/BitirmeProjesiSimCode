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

ros::Publisher masterDecisionPublisher;
int masterDecision;
int level;
string formation;
geometry_msgs::Pose decisionParameters;

int main(int argc, char**argv) {
  
  ros::init(argc, argv, "Control");
  ros::NodeHandle nh;
  masterDecisionPublisher = nh.advertise<const geometry_msgs::Pose>("decision",1);

  cout << "Enter desired formation:" <<  "\n" << "1-Arrow" << "\n" << "2-Reverse Arrow" << "\n" << "3-Line" << "\n" << "4-Follow Each Other" << "\n" << "5-Protect the Master" << "\n";
  cin >> masterDecision;
  cout << "\n";
  if(masterDecision != 5){
    cout << "Select the level:" << "\n" << "1-one meter distance" << "\n" <<"2-two meter distance" << "\n" << "3-three meter distance" << "\n";
    cin >> level;
  }
  float goalPoseX, goalPoseY;
  // parameters are according to left slave, they are adjusted for right slave in right slave node
  if(masterDecision == 1){
    goalPoseX = -1 * level;
    goalPoseY = -1 * level;
    formation = "Arrow";
  }
  else if(masterDecision == 2){
    goalPoseX = -1 * level;
    goalPoseY = 1 * level;
    formation = "Reverse Arrow";
  }
  else if(masterDecision == 3){
    goalPoseX = -1 * level;
    goalPoseY = 0 * level;
    formation = "Line";
  }
  else if(masterDecision == 4) {
    goalPoseX = 0 * level * 1.5;
    goalPoseY = -1 * level * 1.5;
    formation = "Follow Each Other";
  }
  else if(masterDecision == 5) {
    goalPoseX = -0.35;
    goalPoseY = 1.1; 
    formation = "Protect the Master";
  }

  decisionParameters.position.x = goalPoseX;
  decisionParameters.position.y = goalPoseY;
  masterDecisionPublisher.publish(decisionParameters);
  
  cout << "\n";
  cout << "\n";
  cout << "<<< CURRENT FORMATION: " << formation << " with the LEVEL " << level << " >>>" << "\n";
  cout << "\n";
  cout << "Run again to change the formation or to set different level" << "\n";
  cout << "-------------------" << "\n";
  return 0;
}