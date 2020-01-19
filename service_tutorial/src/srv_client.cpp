#include <iostream>
#include <cstdlib>
#include <sstream>
#include "ros/ros.h"
#include "service_tutorial/Srv_Tutorial.h"
using namespace std;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "srv_client");
	
	if(argc != 3){
		ROS_INFO("cmd:rosrun service_tutorial srv_client arg0 arg1 ");
		ROS_INFO("arg0: double number, arg1: double number");
		return 1;
	}
	
	ros::NodeHandle nh;
	
	//in server, nh doesn't use subscriber. use serviceClient instead.
	//of course use "call_server" (as topic)
	ros::ServiceClient sc_client = 
		nh.serviceClient<service_tutorial::Srv_Tutorial>("call_server");

	
	//in service we use srv instead msg.
	service_tutorial::Srv_Tutorial srv;
	
	//init srv
	srv.request.a = atoll(argv[1]);   //select srv type. 
	srv.request.b = atoll(argv[2]);   //i think, when we create srv file, it has options.(request,response)
	if(sc_client.call(srv)){ //.call(): client requests service and it returns true if request success
		ROS_INFO("send srv, srv.Request.a and b : %ld %ld", 
			(long int)srv.request.a, (long int)srv.request.b);
		//if server answers, srv will save server's answer(by srv_server.cpp code)
		ROS_INFO("receive srv, srv.Response.result: %ld", (long int)srv.response.result);
	}
	else{
		ROS_INFO("failed to call service");
		return 1;
	}
	return 0;
}
