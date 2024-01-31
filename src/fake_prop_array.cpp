#include <ros/ros.h>
#include <prop_mapper/PropArray.h>
#include <prop_mapper/Prop.h>
#include <task_master/TaskStatus.h>

// ALSO INCLUDES FAKE TASK MANAGER

int main(int argc, char** argv) {
	ros::init(argc, argv, "fake_prop_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<prop_mapper::PropArray>("/prop_array", 1000);

    ros::Rate loop_rate(2);

    // TEST PROP SET 

    prop_mapper::Prop red_prop;
    red_prop.prop_label = "red_marker";
    red_prop.id = 0;
    red_prop.point.x = -6;       
    red_prop.point.y = 10;       
    red_prop.point.z = 0;       

    prop_mapper::Prop green_prop;
    green_prop.prop_label = "green_marker";
    green_prop.id = 1;
    green_prop.point.x = 6;     
    green_prop.point.y = 10;   
    green_prop.point.z = 0;     

    prop_mapper::Prop yellow_prop;
    yellow_prop.id = 2;
    yellow_prop.prop_label = "yellow_marker";
    yellow_prop.point.x = 0;       
    yellow_prop.point.y = 19;     
    yellow_prop.point.z = 0;       

  

    prop_mapper::PropArray prop_array;
    prop_array.props.push_back(red_prop);
    prop_array.props.push_back(green_prop);
    prop_array.props.push_back(yellow_prop);


    // Sets task to nav_channel without use of task_master
    //nav_channel::TaskStatus task;
    //task.task.current_task = 1;

    while (ros::ok())
    {
        pub.publish(prop_array);
        //task_pub.publish(task);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}