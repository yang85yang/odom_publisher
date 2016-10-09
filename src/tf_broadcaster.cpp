#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Pose2D.h>





//void poseCallback(const turtlesim::PoseConstPtr& msg)

void poseCallback(const geometry_msgs::Pose2DConstPtr& msg)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  tf::Quaternion q;

  q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");

  ROS_INFO("my_tf_broadcaster .......");  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/pose", 100, &poseCallback);

  ros::spin();
  return 0;
};

