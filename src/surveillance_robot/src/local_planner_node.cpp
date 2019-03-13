#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
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


    Odometry::odom odom;
    bool new_pose; // check if a new position arrived
    bool new_goal_to_reach; //to check if a new /goal_to_reach is available or not
    geometry_msgs::Point goal_to_reach;

    int state;
    bool display_state;

public:

decision() {

    state = 1;
    display_state = false;

    sub_pose = n.subscribe("COMPLETE", 1, &decision::odomCallback, this);
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
        new_pose = false;
    }

}// update

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void odomCallback(const nav_msgs::Odometry::ConstPtr& o) {
    new_odom = true;
    odom.x = o->pose.pose.position.x;
    odom.y = o->pose.pose.position.y;
    odom orientation = tf::getYaw(o->pose.pose.orientation);
    new_pose = true;
}

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

};

int main(int argc, char **argv){

    ROS_INFO("(decision_node) waiting for a /goal_to_reach");
    ros::init(argc, argv, "decision");

    decision bsObject;

    ros::spin();

    return 0;
}
