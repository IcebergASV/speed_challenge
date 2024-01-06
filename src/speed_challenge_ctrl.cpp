#include <ros/ros.h>
#include <task_master/TaskStatus.h>
#include <task_master/TaskGoalPosition.h>
#include <task_master/Task.h>
#include <string>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <prop_mapper/Prop.h>
#include <prop_mapper/PropArray.h>
#include <cmath>
#include <iostream>
#include <geometry_msgs/Point.h>

class SpeedChallenge{

public:

    SpeedChallenge(): nh_(""), private_nh_("~")
    {
        prop_sub = nh_.subscribe("/prop_array", 1, &SpeedChallenge::propCallback, this);
    }

    void spin() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();        
        }
    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber prop_sub;

    prop_mapper::PropArray prop_array_test;
    prop_mapper::PropArray prop_array;

    void propCallback(const prop_mapper::PropArray msg)
    {
        prop_array_test = msg;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "speed_challenge_node");

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        ros::console::notifyLoggerLevelsChanged();

    SpeedChallenge speed_challenge;
    speed_challenge.spin();

    return 0;

}