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
#include <std_msgs/UInt16.h>
#include <sys/ioctl.h>

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

int switch_data;
void switch_data_callback(const std_msgs::UInt16::ConstPtr& msg){
    switch_data = msg->data;
}

int cnt=0;
int time_cnt=0;
int cnt_mean=0;
bool switch_toggle=false;
void debouncing()
{
  time_cnt++;
  cnt_mean+=switch_data;
  if(time_cnt==50){
	  if(cnt_mean>=25){switch_toggle=false; cnt_mean=0;time_cnt=0; ROS_INFO("OFF");}
	  else {switch_toggle=true;cnt_mean=0; time_cnt=0;ROS_INFO("ON");}
  }

}


int main (int argc, char** argv){
    ros::init(argc, argv, "serial_node_main");
    ros::NodeHandle nh;
    
    ros::Subscriber push_data_sub = nh.subscribe("ToSubData",1,push_data_callback);
    ros::Subscriber switch_data_sub = nh.subscribe("switch_onoff",1,switch_data_callback,ros::TransportHints().tcpNoDelay());
    ros::Publisher read_from_sub = nh.advertise<std_msgs::String>("read_from_sub", 1);

    

    try
    {
        ser.setPort("/dev/ttySERIAL");
        ser.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
	serial::flowcontrol_t flow_state = serial::flowcontrol_t::flowcontrol_none;
        ser.setFlowcontrol(flow_state);
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

    bool reopen_flag=false; 
    while(ros::ok()){
         
        ros::spinOnce();
	debouncing(); //for debouncing count

	//write data-------------------------------//

        //ROS_INFO_STREAM("subscribe : " <<push_data.data);
	if(ser.available()){
		message_safety=ser.read();
		ROS_INFO_STREAM("receive   : " << message_safety);}
		
	//ROS_INFO_STREAM(push_data.data);
	
	if(!switch_data){if(!push_data.data.empty()){ser.write(push_data.data);}}
	if(switch_data){ser.flush();}
	if(time_cnt=200){time_cnt=0;cnt=0;}

        //----------------------------------------//
	



        loop_rate.sleep();

    }
}
