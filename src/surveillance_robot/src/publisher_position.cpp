#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Vector3.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"


//receives: a point from decision node where we want to move_base_simple
//receives: a current possition from amcl_pose
//publishes:the translataion and the rotation that needs to be performed
class publisher {
private:

    ros::NodeHandle n;
    bool aaa = true;
    ros::Publisher sub_next_local_goal;
    ros::Publisher sub_current_position;

    geometry_msgs::Point local_goal_to_reach;
    geometry_msgs::PoseWithCovarianceStamped current_position;
public:
publisher() {

    sub_next_local_goal = n.advertise<geometry_msgs::Point>("local_goal", 100);
    sub_current_position = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 100);

    local_goal_to_reach.x = 10;
    local_goal_to_reach.y = 21;

    current_position.pose.pose.position.x = 0;
    current_position.pose.pose.position.y = 0;


    current_position.pose.pose.orientation.x = 0.1;
    current_position.pose.pose.orientation.y = 0;
    current_position.pose.pose.orientation.z = 0.3;
    current_position.pose.pose.orientation.w = 1;
    //INFINTE LOOP TO COLLECT POSITION DATA AND PROCESS THEM
    ros::Rate rate(1);// this node will work at 10hz
    while (ros::ok()) {
        // ros::spinOnce();//each callback is called once
        update();
        rate.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }

}

//UPDATE: main processing
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {
    ROS_INFO("publishing");
    sub_next_local_goal.publish(local_goal_to_reach);
    sub_current_position.publish(current_position);
    ROS_INFO("done");

}
};

int main(int argc, char **argv){

    ROS_INFO("(publisher) ");
    ros::init(argc, argv, "publisher");

    publisher bsObject;

    ros::spin();

    return 0;
}
