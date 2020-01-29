#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>   //ros sensor data 
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "custom_joint_state_machine");
    ros::NodeHandle nh;

    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Rate loop_rate(30);

    while(ros::ok()){
        sensor_msgs::JointState joint_state;

        joint_state.header.stamp = ros::Time::now();
        joint_state.name.push_back("joint1");
        joint_state.position.push_back(-1);
        joint_state.name.push_back("joint2");
        joint_state.position.push_back(-1);
        joint_state.name.push_back("joint3");
        joint_state.position.push_back(-1);
        joint_state.name.push_back("joint4");
        joint_state.position.push_back(-1);

        joint_pub.publish(joint_state);

        loop_rate.sleep();
    }
    return 0;
}