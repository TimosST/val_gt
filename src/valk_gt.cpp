#include <gt_val_sim/gt_sim.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "gt_val_sim");
  ros::NodeHandle n;
  if(!ros::master::check())
  {
      cerr<<"Could not contact master!\nQuitting... "<<endl;
      return -1;
  }
 gt_sim *gtrun = new gt_sim();
//  ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);
 // ros::spin();
//De-allocation of Heap
 ROS_INFO( "connecting... " );
 gtrun->connect(n);
 if(!gtrun->connected())
 {
     ROS_ERROR("Could not connect to Humanoid robot!");
     return -1;
 }
 ROS_INFO( "computing... " );
 gtrun->gt_compute();
// delete gtrun;
//Done here
ROS_INFO( "Quitting... " );
 return 0;
}
