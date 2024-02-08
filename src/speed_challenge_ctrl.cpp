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
        private_nh_.param<float>("avoidance_angle", avoidance_angle_p, 15.0);
        private_nh_.param<float>("avoidance_distance", avoidance_distance_p, 5.0);
        private_nh_.param<float>("acceptable_error", acceptable_error_p, 2.0);

        // Subcribe to prop array
        prop_sub_ = nh_.subscribe("/prop_array", 1, &SpeedChallenge::propCallback, this);

        // Subcribe to task_to_execute 
        task_to_exec_ = nh_.subscribe("task_to_execute", 1, &SpeedChallenge::taskCallback, this);

        // Publish task_status 
        task_status_ = nh_.advertise<task_master::TaskStatus>("task_status", 10);

        // Publish goal position 
        task_pos_ = nh_.advertise<task_master::TaskGoalPosition>("/task_goal_position", 10);

        // Get current pos 
        private_nh_.param<std::string>("local_pose_topic", local_pose_topic_, "/mavros/local_position/pose");
        global_pos_ = nh_.subscribe(local_pose_topic_, 1, &SpeedChallenge::localPositionCallback, this);

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
    ros::Publisher task_status_;
   
    //Params 
    float avoidance_angle_p;
    float avoidance_distance_p;
    float acceptable_error_p;

    std::string local_pose_topic_;

    geometry_msgs::PoseStamped current_pos_;

    std::string TAG = "SPEED CHALLENGE CTRL: ";

    // Direction vectors
    geometry_msgs::Point starting_pos_2_prop_;
    geometry_msgs::Point point1_direction_vec_;
    geometry_msgs::Point point2_direction_vec_;
    geometry_msgs::Point point3_direction_vec_;
    
    geometry_msgs::PoseStamped starting_pos_;

    geometry_msgs::Point gate_mid_point_;

    enum states {NOT_STARTED, FIND_GATE, MOVE_TO_GATE, FIND_YELLOW_BUOY, MOVE_TO_POINT1, MOVE_TO_POINT2, MOVE_TO_POINT3, MOVE_BACK_TO_GATE, RETURN_TO_START, COMPLETE};

    states status = states::NOT_STARTED;

    prop_mapper::PropArray prop_array_;

    task_master::TaskGoalPosition task_goal_pos_;

    bool isReached() {

        bool atDestination = false;

        // TODO: use conversion node 
        // Converting to gazebo coordinates 
        float current_pos_x = current_pos_.pose.position.x;
        float current_pos_y = current_pos_.pose.position.y;

        if (current_pos_x < task_goal_pos_.point.x + acceptable_error_p & current_pos_x > task_goal_pos_.point.x - acceptable_error_p) {
            if (current_pos_y < task_goal_pos_.point.y + acceptable_error_p & current_pos_y > task_goal_pos_.point.y - acceptable_error_p) {
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

        task_master::TaskStatus taskStatus;
        taskStatus.status = task_master::TaskStatus::NOT_STARTED;
        taskStatus.task.current_task = task_master::Task::SPEED_RUN;

        if(msg.current_task == task_master::Task::SPEED_RUN)
        {
            switch (status)
            {
            case states::NOT_STARTED: 
            {
                ROS_DEBUG_STREAM(TAG << "waiting for at least 2 markers");

                taskStatus.status = task_master::TaskStatus::IN_PROGRESS;

                if (prop_array_.props.size() >= 2)
                {
                    ROS_INFO_STREAM(TAG << "At least 2 markers detected, moving on to find gate");
                    status = states::FIND_GATE;
                }
                break;
            }
            case states::FIND_GATE: {
                // if have two good props, ie. red on left, green on right, within acceptable distance of each other, then go

                ROS_INFO_STREAM(TAG << "Looking for the gate");
                
                if (findGate()) 
                {
                    ROS_INFO_STREAM(TAG << "Moving to Gate at x: " << gate_mid_point_.x << ", y: " << gate_mid_point_.y );
                    status = states::MOVE_TO_GATE;
                }
                break;
            }
            case states::MOVE_TO_GATE: {
   
                task_goal_pos_.point.x = gate_mid_point_.x;
                task_goal_pos_.point.y = gate_mid_point_.y;
   
                task_goal_pos_.task.current_task = task_master::Task::SPEED_RUN;
               
                task_pos_.publish(task_goal_pos_);
   
                if(isReached()){
   
                    status = states::FIND_YELLOW_BUOY;
   
                    ROS_INFO_STREAM(TAG << "Gate reached");
                    ROS_INFO_STREAM(TAG << "Moving to Point 1 at " << point1_direction_vec_.x + starting_pos_.pose.position.x << ", y: " << point1_direction_vec_.y + starting_pos_.pose.position.y);
                }
                break;

            }
            case states::FIND_YELLOW_BUOY: {

                ROS_INFO_STREAM(TAG << "Looking for the yellow buoy");
                
                prop_mapper::Prop yellow_buoy; 
                
                if(find_yellow_buoy(yellow_buoy)){

                    ROS_INFO_STREAM(TAG << "Yellow Prop Found");
                    float prop_x = yellow_buoy.point.x;
                    float prop_y = yellow_buoy.point.y; 

                    // Direction vector from start to prop 

                    starting_pos_2_prop_.x = prop_x - starting_pos_.pose.position.x;
                    starting_pos_2_prop_.y = prop_y - starting_pos_.pose.position.y;

                    // Distance from start to prop 

                    float prop_distance = pow((starting_pos_2_prop_.x * starting_pos_2_prop_.x) + (starting_pos_2_prop_.y * starting_pos_2_prop_.y), 0.5);

                    // Direction vector from start to point 1 (Rotate 30 degrees clockwise)

                    point1_direction_vec_.x = starting_pos_2_prop_.x * cos(-avoidance_angle_p * M_PI/180) - starting_pos_2_prop_.y * sin(-avoidance_angle_p * M_PI/180);
                    point1_direction_vec_.y = starting_pos_2_prop_.x * sin(-avoidance_angle_p * M_PI/180) + starting_pos_2_prop_.y * cos(-avoidance_angle_p * M_PI/180);

                    // Direction vector from start to point 2 

                    float point2_distance = prop_distance + avoidance_distance_p;

                    point2_direction_vec_.x = starting_pos_2_prop_.x * point2_distance/prop_distance;
                    point2_direction_vec_.y = starting_pos_2_prop_.y * point2_distance/prop_distance;

                    //Direction vector from start to point 3 (Rotate 30 degrees counter clockwise)

                    point3_direction_vec_.x = starting_pos_2_prop_.x * cos(avoidance_angle_p * M_PI/180) - starting_pos_2_prop_.y * sin(avoidance_angle_p * M_PI/180);
                    point3_direction_vec_.y = starting_pos_2_prop_.x * sin(avoidance_angle_p * M_PI/180) + starting_pos_2_prop_.y * cos(avoidance_angle_p * M_PI/180);

                    status = states::MOVE_TO_POINT1; 
                    ROS_INFO_STREAM(TAG << "Moving to Point 1 at " << point1_direction_vec_.x + starting_pos_.pose.position.x << ", y: " << point1_direction_vec_.y + starting_pos_.pose.position.y);

                }
                break;
            }
            case states::MOVE_TO_POINT1: {
   
               // Get point 1 
   
                task_goal_pos_.point.x = point1_direction_vec_.x + starting_pos_.pose.position.x;
                task_goal_pos_.point.y = point1_direction_vec_.y + starting_pos_.pose.position.y;
   
                task_goal_pos_.task.current_task = task_master::Task::SPEED_RUN;
   
                task_pos_.publish(task_goal_pos_);
   
                if(isReached()){
                    status = states::MOVE_TO_POINT2;
                    ROS_INFO_STREAM(TAG << "Point 1 reached");
                    ROS_INFO_STREAM(TAG << "Moving to Point 2 at " << point2_direction_vec_.x + starting_pos_.pose.position.x << ", y: " << point2_direction_vec_.y + starting_pos_.pose.position.y);
                }
                break;
            }
            case states::MOVE_TO_POINT2: {
   
               // Get point 2
               
               task_goal_pos_.point.x = point2_direction_vec_.x + starting_pos_.pose.position.x;
               task_goal_pos_.point.y = point2_direction_vec_.y + starting_pos_.pose.position.y;
               task_goal_pos_.task.current_task = task_master::Task::SPEED_RUN;
   
               task_pos_.publish(task_goal_pos_);
   
                if(isReached()){

                    status = states::MOVE_TO_POINT3;

                    ROS_INFO_STREAM(TAG << "Point 2 reached");
                    ROS_INFO_STREAM(TAG << "Moving to Point 3 at " << point3_direction_vec_.x + starting_pos_.pose.position.x << ", y: " << point3_direction_vec_.y + starting_pos_.pose.position.y);
                }
                break;
            }

            case states::MOVE_TO_POINT3:{
    
                // Get point 3 
                task_goal_pos_.point.x = point3_direction_vec_.x + starting_pos_.pose.position.x;
                task_goal_pos_.point.y = point3_direction_vec_.y + starting_pos_.pose.position.y;
                task_goal_pos_.task.current_task = task_master::Task::SPEED_RUN;
    
                task_pos_.publish(task_goal_pos_);
    
                if(isReached()){
    
                    status = states::MOVE_BACK_TO_GATE;
    
                    ROS_INFO_STREAM(TAG << "Point 3 reached");
                    ROS_INFO_STREAM(TAG << "Moving back to gate at x: " << gate_mid_point_.x << ", y: " << gate_mid_point_.y );
                }
                break;
            }
            case states::MOVE_BACK_TO_GATE: {
    
                task_goal_pos_.point.x = gate_mid_point_.x;
                task_goal_pos_.point.y = gate_mid_point_.y;
    
                task_goal_pos_.task.current_task = task_master::Task::SPEED_RUN;
                
                task_pos_.publish(task_goal_pos_);
    
                if(isReached()){
    
                    status = states::RETURN_TO_START;
    
                    ROS_INFO_STREAM(TAG << "Gate reached");
                    ROS_INFO_STREAM(TAG << "Returning to start at x: " << starting_pos_.pose.position.x << ", y: " << starting_pos_.pose.position.y );
                 }
                 break;
            }
            case states::RETURN_TO_START: {

                task_goal_pos_.point.x = starting_pos_.pose.position.x;
                task_goal_pos_.point.y = starting_pos_.pose.position.y;
                task_goal_pos_.task.current_task = task_master::Task::SPEED_RUN;

                task_pos_.publish(task_goal_pos_);

                if(isReached()){

                    status = states::COMPLETE;

                    ROS_INFO_STREAM(TAG << "Task Complete");

                 }
                break;
            }

            case states::COMPLETE:{

                taskStatus.status = task_master::TaskStatus::COMPLETE;
                taskStatus.task.current_task = task_master::Task::SPEED_RUN;
                break;
            }
            }
        }
        task_status_.publish(taskStatus);
    }

    void localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
       
       // Store starting pos
       if(status == states::NOT_STARTED){
            starting_pos_ = *msg;
       }

        // Get current position
        current_pos_ = *msg;
    }

    bool find_yellow_buoy(prop_mapper::Prop& yellow_buoy){

        for(int i = 0; i < prop_array_.props.size(); i++){
           
           // TODO: Add logic for if yellow prop isn't found 
           if(prop_array_.props[i].prop_label == "yellow_buoy"){
               ROS_INFO_STREAM("Found yellow prop");
               yellow_buoy = prop_array_.props[i];
               return true;
           }
        }
        return false;
    }

    bool findGate(){

        bool green_found = false;
        bool red_found = false;

        prop_mapper::Prop green_prop;
        prop_mapper::Prop red_prop;

        ROS_INFO_STREAM(TAG << "Finding gate");

       for(int i = 0; i < prop_array_.props.size(); i++){
            
           // TODO: Add logic for if props aren't found
            if(prop_array_.props[i].prop_label == "red_buoy"){
    
               red_prop = prop_array_.props[i];
               red_found = true;
            }
            if(prop_array_.props[i].prop_label == "green_buoy"){

               green_prop = prop_array_.props[i];
               green_found = true;
            }
            if(green_found && red_found){

                gate_mid_point_ = calc_midpoint(green_prop, red_prop);
                return true; 
            }
        }
        return false;
    }
    

    geometry_msgs::Point calc_midpoint(prop_mapper::Prop& green_prop, prop_mapper::Prop& red_prop){

        geometry_msgs::Point mid_point;

        mid_point.x =  (green_prop.point.x + red_prop.point.x)/2;
        mid_point.y =  (green_prop.point.y + red_prop.point.y)/2;

        return mid_point;

    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "speed_challenge_node");

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();

    SpeedChallenge speed_challenge;
    speed_challenge.spin();

    return 0;

}