#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gas_brake_pid_node");
  nodelet::Loader nodelet;
  nodelet::M_string remappings(ros::names::getRemappings());
  nodelet::V_string nargv;
  nodelet.load(ros::this_node::getName(), "speed_controller/gas_brake_pid", remappings, nargv);
  ros::spin();
  return 0;
}
