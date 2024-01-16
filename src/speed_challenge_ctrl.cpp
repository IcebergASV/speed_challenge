#include <ros/ros.h>
#include <ros/console.h>
#include <task_master/TaskStatus.h>
#include <task_master/TaskGoalPosition.h>
#include <task_master/Task.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <prop_mapper/Prop.h>
#include <prop_mapper/PropArray.h>
#include <cmath>
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
        prop_sub_ = nh_.subscribe("/prop_array", 1, &SpeedChallenge::propCallback, this);

        // Subcribe to task_to_execute 
        task_to_exec_ = nh_.subscribe("/task_to_execute", 1, &SpeedChallenge::taskCallback, this);

        // Publish goal position 
        task_pos_ = nh_.advertise<task_master::TaskGoalPosition>("/task_goal_position", 10);

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

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber prop_sub_;
    ros::Subscriber task_to_exec_;
    ros::Subscriber global_pos_;
    ros::Publisher task_pos_;
   
    //Params 
    float avoidance_angle;
    float avoidance_distance;
    float acceptable_error;

    std::string local_pose_topic_;

    geometry_msgs::PoseStamped current_pos_;

    // Direction vectors
    geometry_msgs::Point starting_pos_2_prop;
    geometry_msgs::Point point1_direction_vec;
    geometry_msgs::Point point2_direction_vec;
    geometry_msgs::Point point3_direction_vec;
    
    geometry_msgs::PoseStamped starting_pos;

    enum states {NOT_STARTED, MOVE_TO_POINT1, MOVE_TO_POINT2, MOVE_TO_POINT3, RETURN_TO_START};

    states status = states::NOT_STARTED;

    prop_mapper::PropArray prop_array_;

    prop_mapper::Prop yellow_prop;

    task_master::TaskGoalPosition task_goal_pos;

    bool isReached() {

        bool atDestination = false;

        // TODO: use conversion node 
        // Converting to gazebo coordinates 
        float current_pos_x = current_pos_.pose.position.y;
        float current_pos_y = -1 * current_pos_.pose.position.x;

        if (current_pos_x < task_goal_pos.point.x + acceptable_error & current_pos_x > task_goal_pos.point.x - acceptable_error) {
            if (current_pos_y < task_goal_pos.point.y + acceptable_error & current_pos_y > task_goal_pos.point.y - acceptable_error) {
                atDestination = true;
            }
        }

        return atDestination;
    }

    // Get prop array 
    void propCallback(const prop_mapper::PropArray msg)
    {
        prop_array_ = msg;
    }

    void taskCallback(const task_master::Task msg){

        find_yellow_prop();

        // Get x and y of prop
        float prop_x = yellow_prop.vector.x;
        float prop_y = yellow_prop.vector.y; 

        // Direction angle of prop 
        float prop_angle = atan(prop_y/prop_x) * (180.0/M_PI);

        // Distance from start to prop
        float prop_distance;

        // TODO implement config file
        float point2_distance;

        switch(status){

        case states::NOT_STARTED: 

           

            // TODO: Use conversion node
            // Direction vector from start to prop 

            starting_pos_2_prop.x = prop_x - starting_pos.pose.position.y;
            starting_pos_2_prop.y = prop_y - (-1 * starting_pos.pose.position.x);

            // Distance from start to prop 

            prop_distance = pow((starting_pos_2_prop.x * starting_pos_2_prop.x) + (starting_pos_2_prop.y * starting_pos_2_prop.y), 0.5);


            // Direction vector from start to point 1 (Rotate 30 degrees clockwise)

            point1_direction_vec.x = starting_pos_2_prop.x * cos(-avoidance_angle * M_PI/180) - starting_pos_2_prop.x * sin(-avoidance_angle * M_PI/180);
            point1_direction_vec.y = starting_pos_2_prop.y * sin(-avoidance_angle * M_PI/180) + starting_pos_2_prop.y * cos(-avoidance_angle * M_PI/180);

            // Direction vector from start to point 2 

            point2_distance = prop_distance + avoidance_distance;

            point2_direction_vec.x = starting_pos_2_prop.x * point2_distance/prop_distance;
            point2_direction_vec.y = starting_pos_2_prop.y * point2_distance/prop_distance;

            //Direction vector from start to point 3 (Rotate 30 degrees counter clockwise)

            point3_direction_vec.x = starting_pos_2_prop.x * cos(avoidance_angle * M_PI/180) - starting_pos_2_prop.x * sin(avoidance_angle * M_PI/180);
            point3_direction_vec.y = starting_pos_2_prop.y * sin(avoidance_angle * M_PI/180) + starting_pos_2_prop.y * cos(avoidance_angle * M_PI/180);

            status = states::MOVE_TO_POINT1;
            ROS_INFO("Moving to Point 1");
            break;


        case states::MOVE_TO_POINT1:

            // Get point 1 

            task_goal_pos.point.x = point1_direction_vec.x + starting_pos.pose.position.y;
            task_goal_pos.point.y = point1_direction_vec.y + starting_pos.pose.position.x * -1;
            task_goal_pos.task.current_task = task_master::Task::SPEED_RUN;
            
            ROS_INFO("Moving to Point 1");
            task_pos_.publish(task_goal_pos);

            if(isReached()){

                status = states::MOVE_TO_POINT2;

                ROS_INFO("Point 1 reached");
                ROS_INFO("Moving to Point 2");
            }
            break;

        case states::MOVE_TO_POINT2:

            // Get point 2
            
            task_goal_pos.point.x = point2_direction_vec.x + starting_pos.pose.position.y;
            task_goal_pos.point.y = point2_direction_vec.y + starting_pos.pose.position.x * -1;
            task_goal_pos.task.current_task = task_master::Task::SPEED_RUN;

            task_pos_.publish(task_goal_pos);

            if(isReached()){

                status = states::MOVE_TO_POINT3;

                ROS_INFO("Point 2 reached");
                ROS_INFO("Moving to Point 3");
            }
            break;

        case states::MOVE_TO_POINT3:

            // Get point 3 

            task_goal_pos.point.x = point3_direction_vec.x + starting_pos.pose.position.y;
            task_goal_pos.point.y = point3_direction_vec.y + starting_pos.pose.position.x * -1;
            task_goal_pos.task.current_task = task_master::Task::SPEED_RUN;

            task_pos_.publish(task_goal_pos);

            if(isReached()){

                status = states::RETURN_TO_START;

                ROS_INFO("Point 3 reached");
                ROS_INFO("Moving back to start");
            }
            break;

        case states::RETURN_TO_START:

            task_goal_pos.point.x = starting_pos.pose.position.y;
            task_goal_pos.point.y = starting_pos.pose.position.x * -1;
            task_goal_pos.task.current_task = task_master::Task::SPEED_RUN;

            task_pos_.publish(task_goal_pos);

            break;
       }
    }

    void globalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
       
       // Store starting pos
       if(status == states::NOT_STARTED){
            starting_pos = *msg;
       }

        // Get current position
        current_pos_ = *msg;
    }

    void find_yellow_prop(){

        for(int i = 0; i < prop_array_.props.size(); i++){
            
            // TODO: Add logic for if yellow prop isn't found 
            if(prop_array_.props[i].prop_label == "yellow_marker"){
                ROS_INFO_STREAM("Found yellow prop");
                yellow_prop = prop_array_.props[i];
                return;
            }
        }
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