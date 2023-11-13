/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */
#include <iostream>
#include <ros/ros.h>
#include <string.h>
#include <vector>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

using namespace std;

serial::Serial ser;
std_msgs::String ToSub;
std_msgs::String push_data;
float buffer=0;
string message_safety="1";

void alpha_callback(const std_msgs::Float32::ConstPtr& msg){
    
    buffer=msg->data;
    //ROS_INFO("%f",buffer);

    ToSub.data = to_string(buffer);
    //ser.write(ToSub.data);
    // ser.wirte는 무조건 std_msgs의 string형이여야함
    //ROS_INFO_STREAM("write data" << ToSub.data);
}

void push_data_callback(const std_msgs::String& msg){
    push_data.data = msg.data;
}


int main (int argc, char** argv){
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;
    
    ros::Subscriber push_data_sub = nh.subscribe("ToSubData",1,push_data_callback);
    ros::Publisher read_from_sub = nh.advertise<std_msgs::String>("read_from_sub", 1);

    

    try
    {
        ser.setPort("/dev/ttySERIAL");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
	//ser.setFlowcontrol(ser.getFlowcontrol());
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    
    }else{
        return -1;
    }
    
    ros::Rate loop_rate(200);

    int cnt=0;
    bool reopen_flag=false; 
    while(ros::ok()){
         
        ros::spinOnce();


	//write data-------------------------------//

        //ROS_INFO_STREAM("subscribe : " <<push_data.data);
        /*
	if(ser.available()){
		message_safety=ser.read();
		ROS_INFO_STREAM("receive   : " << message_safety);}
 	if(message_safety=="0"){
		cnt++;}
	if(cnt>1000){
	message_safety="1";
		cnt=0;}
		
	ROS_INFO_STREAM(message_safety);
	if(message_safety=="1"){ser.write(push_data.data);
	ROS_INFO("data_sending");}*/
	ROS_INFO_STREAM(push_data.data);

	ser.write(push_data.data);

        //----------------------------------------//
	



        loop_rate.sleep();

    }
}

