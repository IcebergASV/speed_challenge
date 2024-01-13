#include <ros/ros.h>
#include <ros/console.h>
#include <task_master/TaskStatus.h>
#include <task_master/TaskGoalPosition.h>
#include <task_master/Task.h>
#include <string>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <prop_mapper/Prop.h>
#include <prop_mapper/PropArray.h>
#include <task_master/TaskStatus.h>
#include <task_master/TaskGoalPosition.h>
#include <task_master/Task.h>
#include <cmath>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <vector>

class SpeedChallenge{

public:

    SpeedChallenge(): nh_(""), private_nh_("~")
    {
        // Get params 
        private_nh_.param<float>("avoidance_angle", avoidance_angle, 30.0);
        private_nh_.param<float>("avoidance_distance", avoidance_distance, 5.0);
        private_nh_.param<float>("acceptable_error", acceptable_error, 2.0);

        // Subcribe to prop array
        prop_sub = nh_.subscribe("/prop_array", 1, &SpeedChallenge::propCallback, this);

        // Subcribe to task_to_execute 
        task_to_exec = nh_.subscribe("/task_to_execute", 1, &SpeedChallenge::taskCallback, this);

        // Publish goal position 
        task_pos = nh_.advertise<task_master::TaskGoalPosition>("/task_goal_position", 10);

        // Get current pos 
        private_nh_.param<std::string>("local_pose_topic", local_pose_topic_, "/mavros/local_position/pose");
        global_pos_ = nh_.subscribe(local_pose_topic_, 1, &SpeedChallenge::globalPositionCallback, this);

    }

    void spin() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();        
        }
    }

    float error = 2;
    bool isReached() {

        bool atDestination = false;

        // TODO: use conversion node 
        // Converting to gazebo coordinates 
        float current_pos_x = current_pos_.pose.position.y;
        float current_pos_y = -1 * current_pos_.pose.position.x;

        ROS_INFO_STREAM(current_pos_);

        if (current_pos_x < task_goal_pos.point.x + error & current_pos_x > task_goal_pos.point.x - error) {
            if (current_pos_y < task_goal_pos.point.y + error & current_pos_y > task_goal_pos.point.y - error) {
                atDestination = true;
            }
        }

        return atDestination;
    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber prop_sub;
    ros::Subscriber task_to_exec;
    ros::Subscriber global_pos_;
    ros::Publisher task_pos;
   
    //Params 
    float avoidance_angle;
    float avoidance_distance;
    float acceptable_error;

    std::string local_pose_topic_;

    geometry_msgs::PoseStamped current_pos_;

    // Direction vectors
    std::vector<float> starting_pos_2_prop;
    std::vector<float> point1_direction_vec;
    std::vector<float> point2_direction_vec;
    std::vector<float> point3_direction_vec;
    
    geometry_msgs::PoseStamped starting_pos;

    enum states {not_started, point1, point2, point3, return_to_start};

    states status = states::not_started;

    prop_mapper::PropArray prop_array_;

    task_master::TaskGoalPosition task_goal_pos;

    // Get prop array 
    void propCallback(const prop_mapper::PropArray msg)
    {
        prop_array_ = msg;
    }

    void taskCallback(const task_master::Task msg){

        // Get x and y of prop
        float prop_x = prop_array_.props[0].vector.x;
        float prop_y = prop_array_.props[0].vector.y; 

        ROS_INFO_STREAM(prop_x);
        ROS_INFO_STREAM(prop_y);

        // Direction angle of prop 
        float prop_angle = atan(prop_y/prop_x) * (180.0/M_PI);

        // Distance from start to prop
        float prop_distance;

        // TODO implement config file
        float point2_distance;

       switch(status){

        case states::not_started: 

            // TODO: Use conversion node
            // Direction vector from start to prop 

            starting_pos_2_prop.push_back(prop_x - starting_pos.pose.position.y);
            starting_pos_2_prop.push_back(prop_y - (-1 * starting_pos.pose.position.x));

            // Distance from start to prop 

            prop_distance = pow((starting_pos_2_prop[0] * starting_pos_2_prop[0]) + (starting_pos_2_prop[1] * starting_pos_2_prop[1]), 0.5);


            // Direction vector from start to point 1 (Rotate 30 degrees clockwise)

            point1_direction_vec.push_back(starting_pos_2_prop[0] * cos(-avoidance_angle * M_PI/180) - starting_pos_2_prop[0] * sin(-avoidance_angle * M_PI/180));
            point1_direction_vec.push_back(starting_pos_2_prop[1] * sin(-avoidance_angle * M_PI/180) + starting_pos_2_prop[1] * cos(-avoidance_angle * M_PI/180));

            // Direction vector from start to point 2 

            point2_distance = prop_distance + avoidance_distance;

            point2_direction_vec.push_back(starting_pos_2_prop[0] * point2_distance/prop_distance);
            point2_direction_vec.push_back(starting_pos_2_prop[1] * point2_distance/prop_distance);

            //Direction vector from start to point 3 (Rotate 30 degrees counter clockwise)

            point3_direction_vec.push_back(starting_pos_2_prop[0] * cos(avoidance_angle * M_PI/180) - starting_pos_2_prop[0] * sin(avoidance_angle * M_PI/180));
            point3_direction_vec.push_back(starting_pos_2_prop[1] * sin(avoidance_angle * M_PI/180) + starting_pos_2_prop[1] * cos(avoidance_angle * M_PI/180));

            status = states::point1;
            break;


        case states::point1:

            // Get point 1 

            task_goal_pos.point.x = point1_direction_vec[0] + starting_pos.pose.position.y;
            task_goal_pos.point.y = point1_direction_vec[1] + starting_pos.pose.position.x * -1;
            task_goal_pos.task.current_task = task_master::Task::SPEED_RUN;

            ROS_INFO_STREAM(task_goal_pos);
            ROS_INFO("Point 1");
            task_pos.publish(task_goal_pos);

            if(isReached()){

                status = states::point2;

                ROS_INFO("Point 1 reached");
            }
            break;

        case states::point2:

            // Get point 2
            
            task_goal_pos.point.x = point2_direction_vec[0] + starting_pos.pose.position.y;
            task_goal_pos.point.y = point2_direction_vec[1] + starting_pos.pose.position.x * -1;
            task_goal_pos.task.current_task = task_master::Task::SPEED_RUN;

            ROS_INFO_STREAM(task_goal_pos);
            task_pos.publish(task_goal_pos);
            ROS_INFO("Point 2");

            if(isReached()){

                status = states::point3;

                ROS_INFO("Point 2 reached");
            }
            break;

        case states::point3:

            // Get point 3 

            task_goal_pos.point.x = point3_direction_vec[0] + starting_pos.pose.position.y;
            task_goal_pos.point.y = point3_direction_vec[1] + starting_pos.pose.position.x * -1;
            task_goal_pos.task.current_task = task_master::Task::SPEED_RUN;

            ROS_INFO_STREAM(task_goal_pos);
            task_pos.publish(task_goal_pos);
            ROS_INFO("Point 3");

            if(isReached()){

                status = states::return_to_start;

                ROS_INFO("Point 3 reached");
            }
            break;

        case states::return_to_start:

            task_goal_pos.point.x = starting_pos.pose.position.y;
            task_goal_pos.point.y = starting_pos.pose.position.x * -1;
            task_goal_pos.task.current_task = task_master::Task::SPEED_RUN;


            ROS_INFO_STREAM(task_goal_pos);
            task_pos.publish(task_goal_pos);

            break;

       }
    
    }

    void globalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
       
       // Store starting pos
       if(status == states::not_started){
            starting_pos = *msg;
       }

        // Get current position
        current_pos_ = *msg;
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