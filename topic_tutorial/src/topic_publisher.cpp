#include <iostream>
#include <sstream>
#include "ros/ros.h"
#include "topic_tutorial/Msg_Tutorial.h"  //build Msg_Tutorial's msg filer header
using namespace std;
int main(int argc, char **argv)
{
	//below two lines are essential for every nodes.
	ros::init(argc, argv, "topic_publisher");  //file name == node name
	ros::NodeHandle nh;

	//declare publisher using Msg_Tutorial msgfile. (msg type == Msg_Tutorial)
	ros::Publisher ros_tutorial_pub =
		nh.advertise<topic_tutorial::Msg_Tutorial>("ros_tutorial_msg", 100);
	//msg's topic name == ros_tutorial_msg
	//publisher queue's size == 100

	ros::Rate loop_rate(10);  //set loop period. 10Hz == (msgs are sended every 0.1sec)
	
	topic_tutorial::Msg_Tutorial msg;  //declare msg. subscriber will receive this.
	int cnt = 0;
	
	while(ros::ok()){
		msg.stamp = ros::Time::now();
		msg.data = cnt++;
		
		ROS_INFO("send_msg = %d", msg.stamp.sec);
		ROS_INFO("send_msg = %d", msg.stamp.nsec);
		ROS_INFO("send_msg = %d", msg.data);
		ROS_INFO(" ");

		ros_tutorial_pub.publish(msg);
		loop_rate.sleep();
	}
	return 0;
}
