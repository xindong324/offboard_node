// attitude ctrl sample


#include<ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/String.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include<iostream>

using namespace std;


mavros_msgs::State current_state;
int land_mode = 0;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


geometry_msgs::PoseStamped local_position;
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "landing_pose_node");
    ros::NodeHandle nh;
    ROS_INFO("start landing node");


    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, position_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher local_att_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("mavros/setpoint_raw/attitude", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

 
    ros::Rate rate(40.0);
    int land_cnt = 0;
 
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
 
    geometry_msgs::PoseStamped pose;//位置控制
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    
    //geometry_msgs::TwistStamped vel;//速度控制
    double roll = 0;
    double pitch = 20.0*3.14159/180.0;
    double yaw = 0;
    double trust = 0.7;
    mavros_msgs::AttitudeTarget att_cmd; //姿态控制
    geometry_msgs::Quaternion quaternion;//定义四元数
	quaternion=tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw); //欧拉角
    att_cmd.orientation = quaternion;
    att_cmd.type_mask = 0b00000111;
    att_cmd.thrust = 0.8;

 
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
 
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
 
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
 
    ros::Time last_request = ros::Time::now();
 	
 	//起飞
    while(ros::ok())
    {
		if(land_mode == 1)
		{
			ROS_INFO("keyboard interrupt");
			break;
		}
		
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
           	last_request = ros::Time::now();
       	}
        else if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
        else if(fabs( local_position.pose.position.z - 2 )<0.1)
        {
            ROS_INFO("att ctrl start");
            break;
        }
		ROS_INFO("local_pos: x %f, z %f",local_position.pose.position.x,local_position.pose.position.z);
        local_pos_pub.publish(pose);
 
        ros::spinOnce();
        rate.sleep();
    }
    
	//控制降落部分
	while(ros::ok())
	{
		if(land_mode == 1)
		{
			ROS_INFO("keyboard interrupt");
			break;
		}
		
		local_att_pub.publish(att_cmd);	
    	ros::spinOnce();
        rate.sleep();
	}
    
    offb_set_mode.request.custom_mode = "AUTO.LAND";
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    {
        ROS_INFO("land enabled");
        last_request = ros::Time::now();
    }
 
    return 0;
}