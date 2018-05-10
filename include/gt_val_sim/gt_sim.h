#ifndef GT_SIM_H
#define GT_SIM_H
// ROS Headers
#include <ros/ros.h>
// c++ Headers
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <vector>
// ROS Messages
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>


// third party headers
#include <hrl_kinematics/Kinematics.h>

using namespace Eigen;
using namespace std;
using std::cerr;
using std::endl;
using std::string;
using namespace hrl_kinematics;

class gt_sim{
private:
  // ROS Standard Variables
    ros::NodeHandle n;
    ros::Publisher ground_truth_odom_path_pub, ground_truth_com_path_pub, ground_truth_com_pub, ground_truth_odom_pub,joint_filt_pub;
    ros::Subscriber ground_truth_odom_sub, joint_state_sub;
   double  freq, joint_freq;
   Vector3d CoM_gt, CoM_enc;
   Affine3d Tib_gt;
   Quaterniond qib_gt;
   bool visualize_with_rviz,firstJoint,useJointKF,joint_inc,is_connected_, ground_truth_odom_inc;
   int number_of_joints;
//ROS Messages
  nav_msgs::Odometry ground_truth_odom_msg, ground_truth_com_odom_msg, CoM_odom_msg;
  nav_msgs::Path ground_truth_odom_path_msg, ground_truth_com_path_msg;
  geometry_msgs::PoseStamped pose_msg, pose_msg_, temp_pose_msg;
  sensor_msgs::JointState joint_state_msg,joint_filt_msg;
// get joint positions from state message
  std::map<std::string, double> joint_map;
  tf::Point com;
  tf::Transform tf_right_foot, tf_left_foot;

  Kinematics* kin;
  string base_link_frame,lfoot_frame, rfoot_frame;
  double jointFreq,mass, m;
/** Real odometry Data **/
  string ground_truth_odom_topic,joint_state_topic;
// functions
  void subscribe();
  void subscribeToJointState();
  void subscribeToGroundTruth();
  void ground_truth_odom(const nav_msgs::Odometry::ConstPtr& msg);
  void ground_truth_joint_state(const sensor_msgs::JointState::ConstPtr& msg);
  void ground_truth_compute_CoM();
  void publishBodyEstimates();
  void advertise();
  void init();

//  void gt_computeKinTFs();
  // Advertise to ROS Topics
 // void advertise();
public:
  // Constructor/Destructor
   gt_sim();
  ~gt_sim();
  void loadparams();
  bool connect(const ros::NodeHandle nh);
  void gt_compute();
  void chatterCallback(const std_msgs::String::ConstPtr& msg);
  void disconnect();
  bool connected();
};
#endif //GT_SIM_H
