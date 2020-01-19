#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include "service_tutorial/Srv_Tutorial.h"
using namespace std;

bool calculation(service_tutorial::Srv_Tutorial::Request &req,
			service_tutorial::Srv_Tutorial::Response &res){  
//srv file moves between node~node and we can declare srv type as Request or Response.
	res.result = req.a + req.b;

	ROS_INFO("request: x=%ld, y= %ld", (long int)req.a, (long int)req.b);
	ROS_INFO("sending back response: %ld", (long int)res.result);

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "srv_server");  //node name.
	ros::NodeHandle nh;
	
	//in service there's no publisher or subscriber.
	//in service we use ServiceServer and ServiceClient	
	//also, in service, we don't use nh.advertise.
	//we use nh.advertiseService()
	ros::ServiceServer ss_server = nh.advertiseService("call_server", calculation);  
	//when server receives request, calculation will be exec
	//call_server is like topic.
	ROS_INFO("server READY!");
	
	ros::spin();   //now server waits for request
	
	return 0;
}
