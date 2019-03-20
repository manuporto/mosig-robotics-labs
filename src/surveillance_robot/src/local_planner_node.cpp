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

class local_planner {
private:

    ros::NodeHandle n;

    ros::Subscriber sub_goal;
    ros::Subscriber sub_pose;
    ros::Publisher pub_local_motion;




    bool new_pose; // check if a new position arrived
    bool new_goal_to_reach; //to check if a new /goal_to_reach is available or not

    geometry_msgs::Point goal_to_reach;
    geometry_msgs::Point current_position;
    geometry_msgs::Point vector_to_go;

    int state;
    bool display_state;

public:
local_planner() {

    state = 1;
    display_state = false;

    sub_pose = n.subscribe("amcl_pose", 1000, &local_planner::getPosition, this);
    sub_goal = n.subscribe("move_base_simple/goal", 1, &local_planner::getPointGoal, this);
    // ('move_base_simple/goal', PoseStamped, self.update_goal);

    pub_local_motion = n.advertise<geometry_msgs::Point>("goal_to_reach", 1);
    //INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will work at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once
        update();
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }

}

//UPDATE: main processing
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {
    if ( !display_state ) {
        display_state = true;
        ROS_INFO("state: %i", state);

    }

    // we receive a new /goal_to_reach and robair is not doing a translation or a rotation
    if ( new_pose ) {
        // calculation

        // publish

        pub_local_motion.publish(vectorPoints(current_position,goal_to_reach));
        new_pose = false;
    }

}// update

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void getPosition(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  ROS_INFO_STREAM("Estimated position: " << msg);
  current_position.x = msg->pose.pose.position.x;
  current_position.y = msg->pose.pose.position.y;
  ROS_INFO_STREAM("Saved as current_position :" << current_position);
}

void getPointGoal(const geometry_msgs::PoseStamped::ConstPtr& msg){
    ROS_INFO_STREAM("Received pose: " << msg);
    goal_to_reach.x = msg->pose.position.x;
    goal_to_reach.y = msg->pose.position.y;
    ROS_INFO_STREAM("Saved as goal_to_reach :" << goal_to_reach);
}

// Distance between two points
geometry_msgs::Point vectorPoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {
    geometry_msgs::Point vector;
    vector.x = (pa.x-pb.x);
    vector.y = (pa.y-pb.y);
    return vector;
}
// // Distance between two points
// float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {
//
//     return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));
//
// }

};

int main(int argc, char **argv){

    ROS_INFO("(planner_node) waiting for a /goal_to_reach and /current_position");
    ros::init(argc, argv, "planner");

    local_planner bsObject;

    ros::spin();

    return 0;
}
