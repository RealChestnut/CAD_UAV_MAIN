#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <std_msgs/Float32MultiArray.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#define PI 3.141592


double X_main=0.0;
double Y_main=0.0;
double Z_main=0.0;
double Roll_main=0.0;
double Pitch_main=0.0;
double Yaw_main=0.0;


double yaw_now = 0.0;
double base_yaw = 0.0;
double yaw_prev = 0.0;
int yaw_rotate_count =0;

void MAIN_PoseCallback(const geometry_msgs::PoseStamped &msg)
{  

    double roll, pitch, yaw;
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world"; // rviz의 월드프레임을 기준으로 사실은 maindrone의 pose정보를 msg에 담아서 받아옴
    transformStamped.child_frame_id = "tf/MAIN_tf"; //그 world프레임을 기준으로 새롭게 월드 프레임을 생성함 이 월드 프레임의 데이터는 -180.0*PI/180.0, 0.0, 90.0*PI/180.0회전되야함
    
    //QUAT.setRPY(-180.0*PI/180.0, 0.0, 90.0*PI/180.0); :: 이 함수는 각도를 고정해버림
    tf2::Matrix3x3 new_mat;
    tf2::Quaternion quat_new;

    tf2::Quaternion quat_pre(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w);
    tf2::Matrix3x3 pre_mat = tf2::Matrix3x3(quat_pre); // quat_pre를 matrix 3x3에 mapping함
    pre_mat.getRPY(Roll_main,Pitch_main,Yaw_main);
    //ROS_INFO("roll ; %lf|pitch ; %lf|yaw : %lf",Roll_main,Pitch_main,Yaw_main);


    tf::Quaternion quat;
    Eigen::AngleAxisd yawAngle_body(Yaw_main, Eigen::Vector3d::UnitZ()); //optitrack 기준 :: yaw --> Y_axis
    Eigen::AngleAxisd pitchAngle_body(Pitch_main, Eigen::Vector3d::UnitY()); //optitrack 기준 :: pitch --> X axis 
    Eigen::AngleAxisd rollAngle_body(Roll_main, Eigen::Vector3d::UnitX()); // optitrack 기준 :: roll --> Z axis

    // optitrack 원 데이터는 world를 기준으로 frame 데이터가 들어옴
    Eigen::Quaternion<double> q_body = pitchAngle_body * yawAngle_body * rollAngle_body; //world to body :: XYZ, body to world  :: ZYX
    
    Eigen::Matrix3d rotationMatrix_body = q_body.matrix();
    

    ROS_INFO(" X ; %lf| Y ; %lf| Z : %lf", msg.pose.position.y, msg.pose.position.z, msg.pose.position.x);
    tf2::Quaternion quat_body(q_body.x(),q_body.y(),q_body.z(),q_body.w());
    tf2::Matrix3x3(quat_body).getRPY(yaw,roll,pitch);

    
    
    
    transformStamped.transform.translation.x = msg.pose.position.y; //아래가 Z+인 좌표계 기준 global X
    transformStamped.transform.translation.y = msg.pose.position.z; //아래가 Z+인 좌표계 기준 global Y
    transformStamped.transform.translation.z = msg.pose.position.x; //아래가 Z+인 좌표계 기준 global Z
    

    

    transformStamped.transform.rotation.x = q_body.x();
    transformStamped.transform.rotation.y = q_body.y();
    transformStamped.transform.rotation.z = q_body.z();
    transformStamped.transform.rotation.w = q_body.w();
   
    
    X_main = msg.pose.position.y;
    Y_main = msg.pose.position.z;
    Z_main = msg.pose.position.x;
/*    
    base_yaw = yaw;
    if((yaw>0) && (yaw<=PI)) 
    {
        if((fabs(base_yaw)-fabs(yaw_prev))>0){yaw_rotate_count++;}
        else {yaw_rotate_count--;}
    }

    if((yaw<0) && (yaw>-PI)) 
    {
        if((fabs(base_yaw)-fabs(yaw_prev))<0){yaw_rotate_count++;}
        else {yaw_rotate_count--;}
    }
	yaw_now = 0.005*(yaw_rotate_count);//base_yaw+2*PI*yaw_rotate_count;
	
    //imu_rpy.z = yaw_now;
    yaw_prev = base_yaw;
*/
    Roll_main = roll;
    Pitch_main = pitch;
    Yaw_main = yaw;//yaw_now;

    br.sendTransform(transformStamped); //TF publish 개념
    
}
{
    double roll, pitch, yaw;
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "drone_world";
    transformStamped.child_frame_id = "tf/SUB_tf";
    transformStamped.transform.translation.x = msg.pose.position.x;
    transformStamped.transform.translation.y = msg.pose.position.y;
    transformStamped.transform.translation.z = msg.pose.position.z;

    
    transformStamped.transform.rotation.x = msg.pose.orientation.x;
    transformStamped.transform.rotation.y = msg.pose.orientation.y;
    transformStamped.transform.rotation.z = msg.pose.orientation.z;
    transformStamped.transform.rotation.w = msg.pose.orientation.w;

    X_sub = msg.pose.position.x;
    Y_sub = msg.pose.position.y;
    Z_sub = msg.pose.position.z;
    
    tf::Quaternion quat;
    
    tf::quaternionMsgToTF(msg.pose.orientation, quat);

    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    Roll_sub = roll;
    Pitch_sub = pitch;
    Yaw_sub = yaw;

    br.sendTransform(transformStamped);

    // ROS_WARN("GIMBAL TF = %lf, %lf, %lf", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
}



int main(int argc, char **argv){
    ros::init(argc,argv,"setting_drone_tf");

    ros::NodeHandle nh;
    
    geometry_msgs::Vector3 Trans;
    geometry_msgs::Vector3 Rot;


    ros::Subscriber main_pose_sub_ = nh.subscribe("/MAINagent/world", 1, MAIN_PoseCallback, ros::TransportHints().tcpNoDelay());

    
    ros::Publisher pos=nh.advertise<geometry_msgs::Vector3>("/opti_pos",100);
    ros::Publisher rot=nh.advertise<geometry_msgs::Vector3>("/opti_rot",100);

    ros::Rate loop(250);

     while(ros::ok())
    {
    
        Trans.x=X_main;
        Trans.y=Y_main;
        Trans.z=Z_main;

        Rot.x=Roll_main;
        Rot.y=Pitch_main;
        Rot.z=Yaw_main; 

        pos.publish(Trans);
        rot.publish(Rot);
        
        ros::spinOnce();
        loop.sleep();
    }


    ros::spin();
    return 0;
}

