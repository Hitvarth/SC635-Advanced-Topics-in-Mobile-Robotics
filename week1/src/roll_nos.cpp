#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace ros;
using namespace std;

int main(int argc, char **argv)
{
	init(argc, argv, "roll_talker");
	NodeHandle n;

	Publisher roll_pub = n.advertise<std_msgs::String>("roll_nos", 10);
	Rate loop_rate(10);

	std_msgs::String msg;
	msg.data = "170040012_190100057";

	while(ok())
	{
		roll_pub.publish(msg);
		spinOnce();
		loop_rate.sleep();
	}

	return 0;
}