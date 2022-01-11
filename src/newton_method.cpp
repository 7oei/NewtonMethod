#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// ros::Publisher path_pub;

void
PoseCallback (const geometry_msgs::PoseStampedConstPtr & pose_msg)
{
  std::cout << "callback start" << std::endl;



  std::cout << "callback end" << std::endl;
}

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "NewtonMethod");
    ros::NodeHandle nh;
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber pose_sub = nh.subscribe ("move_base_simple/goal", 1, PoseCallback);
    // Spin
    ros::spin ();
}