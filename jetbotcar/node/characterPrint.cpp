#include <iostream>
#include <ros/ros.h>
#include </opt/ros/melodic/include/std_msgs/String.h>
class intervalPrinter
{
public:
	
	ros::NodeHandle n = ros::NodeHandle("~");
	ros::Subscriber characterSub;
	ros::Publisher characterPrintPub;
	std::string print_topic, read_topic;
	double printRate;
	ros::Timer printTimer;
	std_msgs::String msg;
	std::string lastReceivedChararacter;
	int numCharactersPressed;
	void updateLastcharacter(const std_msgs::String& msg) {
		lastReceivedChararacter = msg.data;
		numCharactersPressed++;
		//ROS_INFO("unmber of chars: %i", numCharactersPressed);
	}
	void printCharacter(const ros::TimerEvent&) {
		if (numCharactersPressed == 0) {
			msg.data = std::to_string(numCharactersPressed) + "  characters recieved" ;
		}
		else {
			msg.data = std::to_string(numCharactersPressed) + "  characters recieved,  last character: " + lastReceivedChararacter;
		}
		characterPrintPub.publish(this->msg);
		numCharactersPressed = 0;
	}
	
	intervalPrinter() {

		//n.getParam("print_topic", print_topic);
		//n.getParam("keyboard_topic", read_topic);
		//n.getParam("character_print_rate", printRate);
		print_topic = "/print_topic";
		read_topic = "/key";
		printRate = 2.0;
		numCharactersPressed = 0;
		printTimer = n.createTimer(ros::Duration(printRate), &intervalPrinter::printCharacter, this);
		characterPrintPub = n.advertise<std_msgs::String>(print_topic, 10);
		characterSub = n.subscribe(read_topic, 1, &intervalPrinter::updateLastcharacter, this);
	};
	
private:
	
};


int main(int argc, char ** argv) {
	ros::init(argc, argv, "charPrint");
	intervalPrinter printOneSec;
	ros::spin();
	return 0;

}


