#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace ros;
using namespace std;

int main(int argc, char **argv)
{
	init(argc, argv, "name_talker");
	NodeHandle n;

	Publisher name_pub = n.advertise<std_msgs::String>("names", 10);
	Rate loop_rate(10);

	std_msgs::String msg;
	msg.data = "Pranav_Hitwarth";

	while(ok())
	{
		name_pub.publish(msg);
		spinOnce();
		loop_rate.sleep();
	}

	return 0;
}