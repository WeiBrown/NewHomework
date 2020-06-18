//这个采用监听坐标变换的方法得到当前的位姿
#include "ros/ros.h"
#include<iostream>
#include<geometry_msgs/Pose.h>
//这两个头文件便于输出当前的位姿信息，而且可以给出任意两个坐标系下面的位姿转换,这个必须存起来，这个很关键
#include<tf2_msgs/TFMessage.h>
#include<tf2_ros/transform_listener.h>
using namespace std;

void get_present_pos(geometry_msgs::Pose & pre_pos)
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    int flag=0;
    while(ros::ok()&&flag==0)//真好拿到就跑..
    {
    geometry_msgs::TransformStamped transformStamped;//这个就是当前点的坐标..
     try{
        transformStamped = tfBuffer.lookupTransform("map", "base_link",
                               ros::Time(0));
        pre_pos.position.x = transformStamped.transform.translation.x;
        pre_pos.position.y = transformStamped.transform.translation.y;
        pre_pos.position.z = transformStamped.transform.translation.z;
        pre_pos.orientation.x = transformStamped.transform.rotation.x;
        pre_pos.orientation.y = transformStamped.transform.rotation.y;
        pre_pos.orientation.z = transformStamped.transform.rotation.z;
        pre_pos.orientation.w = transformStamped.transform.rotation.w;
        flag=1;
        }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }//必须try..
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pub_present");//最好记录下来
    ros::NodeHandle nh;
    geometry_msgs::Pose cur_pose;
    ros::Publisher pub = nh.advertise<geometry_msgs::Pose>("/navi/present_pose", 1);//后期给出状态流动图
    ros::Rate loop_rate(10);//10hz的频率发送当前的位姿信息,相对于地图坐标系下的
    while(ros::ok())
    {
        get_present_pos(cur_pose);//发布当前的位姿信息,我可以把这一切都自动化，也就是到了门自动储存当前的位姿信息，便于后续的进一步操作。
        pub.publish(cur_pose);
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;//语音控制前后左右移动，控制建图等等，优化图形建立过程，建立语音控制板块,这个可以后期再说
}
