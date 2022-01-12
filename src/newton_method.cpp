#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>

// ros::Publisher path_pub;
float
ObjectiveFn(Eigen::Vector2f state){
    return 4*pow(state.x(),4) + 3*pow(state.x(),3) + 5*pow(state.x(),2)*pow(state.y(),2) + 7*state.x()*pow(state.y(),3) + pow(state.y(),5);
}

Eigen::Vector2f
GradientFn(Eigen::Vector2f state){
    Eigen::Vector2f ret;
    ret.x() = 16 * pow(state.x(),3) + 9 * pow(state.x(),2) + 10 * state.x() * pow(state.y(),2) + 7 * pow(state.y(),3);
    ret.y() = 10 * pow(state.x(),2) * state.y() + 21 * state.x() * pow(state.y(),2) + 5 * pow(state.y(),4);
    return ret;
}

Eigen::Matrix2f
HessianFn(Eigen::Vector2f state){
    Eigen::Matrix2f ret;
    ret(0,0) = 48 * pow(state.x(),2) + 18 * state.x() + 10 * pow(state.y(),2);
    ret(0,1) = 20 * state.x() * state.y() + 21 * pow(state.y(),2);
    ret(1,0) = ret(0,1);
    ret(1,1) = 10 * pow(state.x(),2) + 42 * state.x() * state.y() + 20 * pow(state.y(),3);
    return ret;
}

void
PoseCallback (const geometry_msgs::PoseStampedConstPtr & pose_msg)
{
    std::cout << "callback start" << std::endl;
    Eigen::Vector2f state;
    // state << -0.5,1.6;//ok
    // state << -0.4,1.5;//ng
    state << -1.3,1.5;//ok
    // state << 0,2;//ng
    // state << -1.2,-0.5;//ng
    Eigen::Vector2f update;
    float error = 1;
    while(error>0.0001){
        update = state - HessianFn(state).inverse() * GradientFn(state);
        error = abs(ObjectiveFn(update) - ObjectiveFn(state));
        std::cout << "state score = " << ObjectiveFn(state) << std::endl;
        std::cout << "state = (" << state.x() <<","<< state.y() << ")";
        std::cout << " , update = (" << update.x() <<","<< update.y() << ")" << std::endl;
        state = update;
    }
    std::cout << "final_state = (" << state.x() <<","<< state.y() << ")" << std::endl;
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