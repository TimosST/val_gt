#include <gt_val_sim/gt_sim.h>

gt_sim::gt_sim()
{


}
gt_sim::~gt_sim()
{
  if (is_connected_)
    disconnect();
}
void gt_sim::disconnect() {
  if (!is_connected_)
    return;
  is_connected_ = false;
}

bool gt_sim::connected() {
  return is_connected_;
}

void gt_sim::loadparams() {
    ros::NodeHandle n_p("~");

    n_p.param<double>("imu_topic_freq",freq,1000.0);
    n_p.param<double>("mass", m, 5.14);
    n_p.param<double>("joint_topic_freq",joint_freq,1000.0);
    n_p.param<std::string>("joint_state_topic", joint_state_topic,"joint_states");
    n_p.param<std::string>("ground_truth_odom_topic", ground_truth_odom_topic,"ground_truth_valk");
    n_p.param<std::string>("base_link",base_link_frame,"base_link");
    n_p.param<std::string>("lfoot",lfoot_frame,"l_sole");
    n_p.param<std::string>("rfoot",rfoot_frame,"r_sole");

    n_p.param<bool>("estimateJoints", useJointKF,false);
}

// joint_map compute----------------------------------------------------------------------------------------------------------------------

void gt_sim::subscribeToJointState()
{
  joint_state_sub = n.subscribe(joint_state_topic,1000,&gt_sim::ground_truth_joint_state,this);
  ROS_INFO_STREAM(" subscribeToJointState....");
  cout << "Press enter to continue ...";
  cin.get();
}

void gt_sim::ground_truth_joint_state(const sensor_msgs::JointState::ConstPtr& msg)
{

  joint_state_msg = *msg;
  joint_inc = true;

    for (unsigned int i=0; i< joint_state_msg.name.size(); i++){
      joint_map.insert(make_pair(joint_state_msg.name[i], joint_state_msg.position[i]));
    }

}



void gt_sim::subscribeToGroundTruth()
{
  ground_truth_odom_sub = n.subscribe(ground_truth_odom_topic,1000,&gt_sim::ground_truth_odom,this);
}
void gt_sim::ground_truth_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
  ground_truth_odom_msg = *msg;
ground_truth_odom_msg.pose.pose.position.x += 0.087515562;
ground_truth_odom_msg.pose.pose.position.z -=0.109324413;
  ground_truth_odom_inc = true;
  Tib_gt.translation() = Vector3d(ground_truth_odom_msg.pose.pose.position.x,ground_truth_odom_msg.pose.pose.position.y,ground_truth_odom_msg.pose.pose.position.z);
  qib_gt = Quaterniond(ground_truth_odom_msg.pose.pose.orientation.w,ground_truth_odom_msg.pose.pose.orientation.x, ground_truth_odom_msg.pose.pose.orientation.y,ground_truth_odom_msg.pose.pose.orientation.z);
  Tib_gt.linear() = qib_gt.toRotationMatrix();

}

// subscribe to topics----------------------------------------------------------------------------------
  void gt_sim::subscribe()
  {

    firstJoint = true;
    subscribeToJointState();

    subscribeToGroundTruth();
    ros::Duration(1.0).sleep();
  }

// subscribe to topics----------------------------------------------------------------------------------


bool gt_sim::connect(const ros::NodeHandle nh) {
  // Initialize ROS nodes
  n = nh;
  // Load ROS Parameters
  loadparams();
  init();
  //Subscribe/Publish ROS Topics/Services

  subscribe();

  advertise();

  is_connected_ = true;
  ROS_INFO_STREAM("Val Ground Truth Node Initialize");
  return true;
}

// COM compute----------------------------------------------------------------------------------------------------------------------

void gt_sim::ground_truth_compute_CoM()
{
      kin->computeCOM(joint_map, com, mass, tf_right_foot, tf_left_foot);
      CoM_enc << com.x(), com.y(), com.z();
      CoM_gt = Tib_gt * CoM_enc;
}



void gt_sim::publishBodyEstimates() {
        ground_truth_odom_path_msg.header.stamp = ros::Time::now();
        ground_truth_odom_path_msg.header.frame_id = "ground_truth_odom";

        temp_pose_msg.pose = ground_truth_odom_msg.pose.pose;
        ground_truth_odom_path_msg.poses.push_back(temp_pose_msg);
        ground_truth_odom_path_pub.publish(ground_truth_odom_path_msg);

        ground_truth_com_path_msg.header.stamp = ros::Time::now();
        ground_truth_com_path_msg.header.frame_id = "ground_truth_com";
        temp_pose_msg.pose.position.x = CoM_gt(0);
        temp_pose_msg.pose.position.y = CoM_gt(1);
        temp_pose_msg.pose.position.z = CoM_gt(2);

	cout<<"COM "<<endl;        
	cout<<CoM_gt<<endl;
	cout<<"odom "<<endl;        
	cout<< ground_truth_odom_msg.pose.pose.position.x<<endl;
	cout<< ground_truth_odom_msg.pose.pose.position.y<<endl;
	cout<< ground_truth_odom_msg.pose.pose.position.z<<endl;
        ground_truth_com_path_msg.poses.push_back(temp_pose_msg);
        ground_truth_com_path_pub.publish(ground_truth_com_path_msg);


        ground_truth_com_odom_msg.header.stamp = ros::Time::now();
        ground_truth_com_odom_msg.header.frame_id = "ground_truth_com";
        ground_truth_com_odom_msg.pose.pose = temp_pose_msg.pose;
        ground_truth_com_pub.publish(ground_truth_com_odom_msg);

        ground_truth_odom_msg.header.frame_id = "ground_truth_odom";
        ground_truth_odom_pub.publish(ground_truth_odom_msg);
}

void gt_sim::advertise() {

      ground_truth_odom_path_msg.poses.resize(1000);
      ground_truth_com_path_msg.poses.resize(1000);
      ground_truth_odom_path_pub = n.advertise<nav_msgs::Path>("/valkyrie/ground_truth/odom/path",1000);
      ground_truth_com_path_pub = n.advertise<nav_msgs::Path>("/valkyrie/ground_truth/CoM/path",1000);
      ground_truth_com_pub = n.advertise<nav_msgs::Odometry>("/valkyrie/ground_truth/CoM",1000);
      ground_truth_odom_pub = n.advertise<nav_msgs::Odometry>("/valkyrie/ground_truth/odom",1000);

}



void gt_sim::init() {
  /** Initialize Variables **/
  kin = new Kinematics(base_link_frame,rfoot_frame,lfoot_frame);
}



void gt_sim::gt_compute(){
  static ros::Rate rate(freq); //ROS Node Loop Rate
  while (ros::ok()){
    ground_truth_compute_CoM(); // is estimateWithCoMEKF

    publishBodyEstimates();

    ros::spinOnce();

    rate.sleep();

  }
  ROS_INFO_STREAM("Val Ground Truth Node Exiting....");
}







