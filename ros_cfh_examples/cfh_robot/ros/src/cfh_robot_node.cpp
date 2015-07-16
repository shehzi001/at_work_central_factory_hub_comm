#include <ros/ros.h>
#include <cfh_robot/cfh_robot.h>

int main(int argc, char **argv){

  ros::init(argc, argv, "cfh_node");
  ROS_INFO("CFH is running!");
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(1); // one Hz

  CFHRobot cfh_robot(nh);

  while (ros::ok()) {
    cfh_robot.sendBeacon();

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}