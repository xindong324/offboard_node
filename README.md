# MAVROS OFFBOARD  姿态控制例程

## 四元数与欧拉角互转
```
#include<ros/ros.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h> //amcl_pose
#include<tf/transform_datatypes.h> //转换函数头文件
#include <tf/tf.h>

class RobotMove
{
public:
    RobotMove();
    void poseCallBack(geometry_msgs::PoseWithCovarianceStampedConstPtr &pose);

private:
    ros::NodeHandle n;
    ros::Subscriber pose_sub; //订阅amcl位置
    float pose_x,pose_y,pose_z;
};
//构造函数
RobotMove::RobotMove()
{  
	pose_sub=n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose",10,&RobotMove::poseCallBack,this);
}

//位置回调函数
void RobotMove::poseCallBack(geometry_msgs::PoseWithCovarianceStampedConstPtr &pose)
{
    //四元数转欧拉角
    tf::Quaternion quat; //定义一个四元数
    tf::quaternionMsgToTF(pose->pose.pose.orientation,quat); //取出方向存储于四元数
 
    double roll,pitch,yaw; //定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw); //进行转换
    pose_x=roll;
    pose_y=pitch;
    pose_z=yaw;

	//欧拉角转四元数,类型可以不同
	geometry_msgs::Quaternion quaternion;//定义四元数
	quaternion=tf::createQuaternionMsgFromRollPitchYaw(0,0,0); //欧拉角
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robotMoveAction");
    RobotMove robotmove;
    ros::spin();
    return 0;
}

```

## mavros 姿态控制
话题发布到 `/mavros/setpoint_raw/attitude`,发布时注意需要将AttitudeTarget中type_mask设置为`att_cmd.type_mask = 0b00000111;` 以屏蔽角速度；
```
~setpoint_raw/attitude (mavros_msgs/AttitudeTarget)

    Attitude, angular rate and thrust setpoint. 

# Raw Message Definition of mavros_msgs/AttitudeTarget
# Message for SET_ATTITUDE_TARGET
#
# Some complex system requires all feautures that mavlink
# message provide. See issue #402, #418.

std_msgs/Header header

uint8 type_mask
uint8 IGNORE_ROLL_RATE = 1 # body_rate.x
uint8 IGNORE_PITCH_RATE = 2 # body_rate.y
uint8 IGNORE_YAW_RATE = 4 # body_rate.z
uint8 IGNORE_THRUST = 64
uint8 IGNORE_ATTITUDE = 128 # orientation field

geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 body_rate
float32 thrust

```

## reference link
1. [px4 姿态控制](https://blog.csdn.net/qq_15390133/article/details/106205463)
2. [ros 四元数欧拉角互转](https://blog.csdn.net/da_ge_chen/article/details/101675556)
