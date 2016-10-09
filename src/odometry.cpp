#include <ros/ros.h>
#include <iostream>
#include <tf/tf.h>
#include <string>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

float x,y,theta;
float linear_x,angular_z;


void odomCallBack(const geometry_msgs::Pose2D &pose)
{
    x = pose.x;
    y = pose.y;
    theta = pose.theta;
}

void cmdCallback(const geometry_msgs::Twist &cmd)
{
   linear_x = cmd.linear.x;
   angular_z = cmd.angular.z;
}

int main(int argc,char **argv)
{

    ros::init(argc,argv,"odom");
    ros::NodeHandle n;

    ros::Publisher odom_pub;
    ros::Subscriber sub_pose;
    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion odom_quat;
    geometry_msgs::TransformStamped odom_trans;
    tf::TransformBroadcaster broadcaster;
    geometry_msgs::Twist odom_twist;
    ros::Subscriber sub_cmd;
    ros::Rate r(40);


    sub_pose = n.subscribe("/pose",1000,odomCallBack);
    sub_cmd = n.subscribe("/cmd_vel",1000,cmdCallback);
    odom_pub = n.advertise<nav_msgs::Odometry>("/odom",1000);

    ROS_INFO("odom publish 40 Hz...");

    while (ros::ok())
    {

    //odom_pose
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = odom_quat;
    odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,theta);
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";


    //odom_trans
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta);
    \
    //odom_twist
    odom.twist.twist.linear.x = linear_x;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = angular_z;

    //publish odom and odom_trans
    broadcaster.sendTransform(odom_trans);
    odom_pub.publish(odom);
    ros::spinOnce();
    r.sleep();

    }

    return 0;

}
