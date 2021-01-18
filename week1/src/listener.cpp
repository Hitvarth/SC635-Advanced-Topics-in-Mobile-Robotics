#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>

using namespace ros;
using namespace std;

bool has_name = false;
bool has_roll = false;

string name1, name2, roll1, roll2;


// HELPER FUNCTION
int give_split_idx(string msg){
	int current = 0;
	while(msg[current] != '_') current++;
	return current;
}

// PRINTING FUNCTION
void print_msg(){
	if(has_name and has_roll){
		cout << "Student "<< name1 <<" has roll : " << roll1 <<" \nStudent "<< name2 <<" has roll : " << roll2 << endl;
	}
}

// CHANGING GLOBAL VARIABLES 
void get_names(string name){
	int idx = give_split_idx(name);
	name1 = name.substr(0, idx); 
    name2 = name.substr(idx+1, name.length()-1); 
	has_name = true;
}

void get_rolls(string roll){
	int idx = give_split_idx(roll);
	roll1 = roll.substr(0, idx); 
    roll2 = roll.substr(idx+1, roll.length()-1);
	has_roll = true;
}

// SUBSCRIBER CALLBACKS
void name_callback(const std_msgs::String::ConstPtr& msg){
	get_names(msg->data); 
	print_msg();
}

void roll_callback(const std_msgs::String::ConstPtr& msg){
	get_rolls(msg->data); 
	print_msg();
}


// MAIN FUNCTION
int main(int argc, char **argv)
{
	init(argc, argv, "listener");
	NodeHandle n;

	Subscriber roll_sub = n.subscribe("roll_nos", 10, roll_callback);
	Subscriber name_sub = n.subscribe("names", 10, name_callback);
	Rate loop_rate(10);

	spin();

	return 0;
}