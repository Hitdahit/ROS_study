#include <iostream>
#include <sstream>
#include "ros/ros.h"
#include "topic_tutorial/Msg_Tutorial.h"  //if you use custom msg, include "packagename/msgname.h"
using namespace std;
void msgCallback(const topic_tutorial::Msg_Tutorial::ConstPtr& msg){  
//declare func this way when msg is param
	//when receives msg, this func will be called
	ROS_INFO("received msg = %d", msg->stamp.sec);
	ROS_INFO("received_msg = %d", msg->stamp.nsec);
	ROS_INFO("received msg = %d", msg->data);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "topic_subscriber");  //node name
	ros::NodeHandle nh;
	
//declare subscrber. requires topic name, subscriber queue size, 
//and callback func which will be exec if msg is received.
	ros::Subscriber ros_tutorial_sub = nh.subscribe("ros_tutorial_msg", 100, msgCallback);
	
	//subscriber's spin func waits for msg. and when msg arrivies, 
	//subscriber's callback func willbe exec. 
	ros::spin();
	return 0;
}
