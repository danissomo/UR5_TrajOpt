#include <ros/ros.h>
#include "settings_custom_lib/SettingsCustomLib.hpp"
#include "settings_custom_lib/AddSettingsCustomLib.hpp"

SettingsCustomLibClass settingsConfig;

int main(int argc, char *argv[]) {

	ROS_INFO("Start update settings....:");
    ros::init(argc, argv, "settings_custom_lib");

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(10);
    spinner.start();

    settingsConfig.update();
    ros::Duration(1).sleep();
    ros::waitForShutdown();

    return 0;
}
