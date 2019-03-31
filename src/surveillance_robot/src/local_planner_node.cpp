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
    ros::Publisher pub_current_position;
    ros::Publisher pub_goal_to_reach;

    float pose_distance_threshold = 1;


    bool new_pose; // check if a new position arrived
    bool new_goal_to_reach; //to check if a new /goal_to_reach is available or not
    bool new_pose_estim;

    geometry_msgs::Point goal_to_reach;
    geometry_msgs::Pose current_position_estim;
    geometry_msgs::Pose old_pose_estim;

    int state;
    bool display_state;

public:
local_planner() {

    state = 1;
    display_state = false;
    new_pose_estim = false;
    new_goal_to_reach = false;

    sub_pose = n.subscribe("amcl_pose", 1000, &local_planner::getPosition, this);
    sub_goal = n.subscribe("move_base_simple/goal", 1, &local_planner::getPointGoal, this);
    // ('move_base_simple/goal', PoseStamped, self.update_goal);

    pub_goal_to_reach = n.advertise<geometry_msgs::Point>("goal_to_reach", 1);
    pub_current_position = n.advertise<geometry_msgs::Pose>("current_postition", 1);
    //INFINTE LOOP TO COLLECT POSITION DATA AND PROCESS THEM
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
    if ( new_goal_to_reach ) {
        // calculation

        // publish
        ROS_INFO("publishing new goal to reach");
        pub_goal_to_reach.publish(goal_to_reach);
        new_goal_to_reach = false;
    }

    if(new_pose_estim){
      if(distancePoints(current_position_estim.position, old_pose_estim.position) > pose_distance_threshold){

        old_pose_estim = current_position_estim;
        // publish
        ROS_INFO("publishing new goal to reach");
        pub_current_position.publish(current_position_estim);
        new_pose = false;
      }
    }
}// update

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void getPosition(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  ROS_INFO_STREAM("Estimated position: " << msg);
  current_position_estim.position = msg->pose.pose.position;
  current_position_estim.orientation = msg->pose.pose.orientation;
  ROS_INFO_STREAM("Saved as current_position_estim :" << current_position_estim << "\n");
  new_pose = true;
}

void getPointGoal(const geometry_msgs::PoseStamped::ConstPtr& msg){
    ROS_INFO_STREAM("Received final pose: " << msg);
    goal_to_reach.x = msg->pose.position.x;
    goal_to_reach.y = msg->pose.position.y;
    ROS_INFO_STREAM("Saved as goal_to_reach :" << goal_to_reach);
    new_goal_to_reach = true;
}

// Vector
geometry_msgs::Point vectorPoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {
    geometry_msgs::Point vector;
    vector.x = (pa.x-pb.x);
    vector.y = (pa.y-pb.y);
    return vector;
}
// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

};

int main(int argc, char **argv){

    ROS_INFO("(planner_node) waiting for a /goal_to_reach and /current_position_estim");
    ros::init(argc, argv, "planner");

    local_planner bsObject;

    ros::spin();

    return 0;
}
