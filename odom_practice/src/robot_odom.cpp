#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <cmath>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <iostream>

using namespace std;

class odom{
private:
    ros::NodeHandle nh, private_nh;
    ros::Subscriber sub;
    ros::Publisher odomPub;
    ros::Time last;
    string base_link_id, odom_link_id, wheel_1_id, wheel_3_id;

    double separation_length;
    double x_dot, y_dot, theta_dot;   //velocity
    double x, y, theta;               //position
    int seq; 

    tf::TransformBroadcaster br;
    double left_vel, right_vel, base_vel;
public:
    odom():seq(0), x(0.0), y(0.0), theta(0.0),
                        last(ros::Time::now()), private_nh("~"){
        sub = nh.subscribe("/gazebo/link_states", 100, &odom::calcWheelVelocityGazeboCB, this);   //get info of linked tf.
        odomPub = nh.advertise<nav_msgs::Odometry>(
            "/custom_odom",    //I wil publish msg with odomPub, and this is topic.
            100
        );   

        if(!private_nh.getParam("/robot_odom/base_link_id", base_link_id)) throw std::runtime_error("set custom_base");
        if(!private_nh.getParam("/robot_odom/odom_link_id", odom_link_id)) throw std::runtime_error("set custom_odom");
        if(!private_nh.getParam("/robot_odom/leftwheel_linkname", wheel_1_id)) throw std::runtime_error("set wheel_1");
        if(!private_nh.getParam("/robot_odom/rightwheel_linkname", wheel_3_id)) throw std::runtime_error("set wheel_3");
        if(!private_nh.getParam("/robot_odom/separation_length", separation_length)) throw std::runtime_error("set seapration_length");
    }
    void calcWheelVelocityGazeboCB(const gazebo_msgs::LinkStates::ConstPtr& ptr){
        int left_idx, right_idx;
        for(int i=0;i < ptr->name.size();i++){
            if(!strcmp(ptr->name[i].c_str(), wheel_1_id.c_str())) left_idx = i;
            else if(!strcmp(ptr->name[i].c_str(), wheel_3_id.c_str())) right_idx = i;
        }

        left_vel = sqrt(pow(ptr->twist[left_idx].linear.x, 2) + pow(ptr->twist[left_idx].linear.y, 2) +pow(ptr->twist[left_idx].linear.z, 2));
        right_vel = sqrt(pow(ptr->twist[right_idx].linear.x, 2) + pow(ptr->twist[right_idx].linear.y, 2) +pow(ptr->twist[right_idx].linear.z, 2));
        base_vel = (left_vel + right_vel) / 2;
    }
    void pubTF(){
        ros::Time cur = ros::Time::now();
        double dt = (cur-last).toSec();

        //left_vel, right_vel == left_ang_vel, right_ang_vel (because wheel's radius == 1)

        theta_dot = (right_vel - left_vel)/separation_length; //ang_vel. this is radian. differential below.
        x_dot = base_vel * cos(theta_dot);
        y_dot = base_vel * sin(theta_dot);

        theta += theta_dot * dt;   //integral below
        x += x_dot * dt;
        y += y_dot * dt;

        last = cur;

        //odom is ready. now make msg and publish it using odomPub instance.
        nav_msgs::Odometry msg;

        msg.header.seq = seq++;
        msg.header.stamp = cur;
        msg.header.frame_id = odom_link_id;
        msg.child_frame_id = base_link_id;

        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.position.z = 0;
        msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
        
        msg.twist.twist.linear.x = x_dot;
        msg.twist.twist.linear.y = y_dot;
        msg.twist.twist.angular.z = theta_dot;

        odomPub.publish(msg);

        broadcastTransform();
    }
    void broadcastTransform(){
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, theta);
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(
            transform,
            ros::Time::now(), 
            odom_link_id.c_str(), 
            base_link_id.c_str()  
        ));   //this links custom_base and custom_odom

        br.sendTransform(tf::StampedTransform(
            transform,
            ros::Time::now(),
            odom_link_id.c_str(),
            "base_link"  
        ));  //this actually links custom_odom and base_link
    }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "robot_odom");
    odom odometry;
    ros::Rate loop_rate(100);

    while(ros::ok()){
        ros::spinOnce();
        odometry.pubTF();
        loop_rate.sleep();
    }

    return 0;
}