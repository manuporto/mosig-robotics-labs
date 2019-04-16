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
class local_planner {
private:

    ros::NodeHandle n;

    ros::Subscriber sub_next_local_goal;
    ros::Subscriber sub_current_position;
    ros::Publisher pub_tran_rot_to_do;

    float pose_distance_threshold = 1;


    bool new_position; // check if a new position arrived
    bool new_local_goal_to_reach; //to check if a new /local_goal_to_reach is available or not

    geometry_msgs::Point local_goal_to_reach;
    geometry_msgs::Pose current_position;
    geometry_msgs::Point translation_rotation;//x = translation, y = rotation, z = 0;

    int state;
    bool display_state;

public:
local_planner() {

    state = 1;
    display_state = false;
    new_position = false;
    new_local_goal_to_reach = false;

    pub_tran_rot_to_do = n.advertise<geometry_msgs::Point>("local_planner/translation_rotation", 1);


    sub_current_position = n.subscribe("amcl_pose", 1, &local_planner::getPosition, this);
    sub_next_local_goal = n.subscribe("local_planner/local_goal", 1, &local_planner::getPointGoal, this);

    //debugging
    // sub_next_local_goal = n.subscribe("move_base_simple/goal", 1, &local_planner::getPointGoal, this);

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

    // we receive both position estimation and new local_goal_to_reach
    // we can compute the translation and the rotation
    // and publish it to the decision node
      // ROS_INFO("nothing new");

    if ( new_local_goal_to_reach && new_position) {
        // calculation
        ROS_INFO("New local goal to reach and new position estimation");
        ROS_INFO("calculating the translation and the rotation to do");
        float tranlation = get_translation_to_do(current_position, local_goal_to_reach);
        float rotation = get_angle_to_do(current_position, local_goal_to_reach);

        translation_rotation.x = tranlation;
        translation_rotation.y = rotation;

        // publish
        ROS_INFO("publishing new tranlation and rotation to do to the decision node");
        ROS_INFO_STREAM(translation_rotation);
        pub_tran_rot_to_do.publish(translation_rotation); //TODO
        new_local_goal_to_reach = false;
        new_position = false;
    }
}// update

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void getPosition(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){//receive new position estimation from AMCL
  // ROS_INFO_STREAM("Estimated position: " << msg);
  current_position.position = msg->pose.pose.position;
  current_position.orientation = msg->pose.pose.orientation;
  ROS_INFO("Received new position estimation");

  // ROS_INFO_STREAM("Received new position estimation:" << current_position << "\n");
  new_position = true;
}

void getPointGoal(const geometry_msgs::Point::ConstPtr& msg){//receive new local point to reach from decision node
    ROS_INFO_STREAM("Received local goal " << msg);
    local_goal_to_reach.x = msg->x;
    local_goal_to_reach.y = msg->y;
    new_local_goal_to_reach = true;
}

//debugging:
// void getPointGoal(const geometry_msgs::PoseStamped::ConstPtr& msg){//receive new local point to reach from decision node
//     ROS_INFO_STREAM("Received local goal " << msg);
//     local_goal_to_reach.x = msg->pose.position.x;
//     local_goal_to_reach.y = msg->pose.position.y;
//     new_local_goal_to_reach = true;
// }

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {
    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));
}

float deg_from_quaternion(geometry_msgs::Quaternion msg){
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);
    // the tf::Quaternion has a method to access roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    // ROS_INFO("Angles: roll=%f pitch=%f yaw=%f", rpy.x, rpy.y, rpy.z);


    float degree = rpy.z;
    ROS_INFO("Angle from quaternion: %f", degree*180/M_PI);
    return degree;
}

float get_angle_to_do(geometry_msgs::Pose position, geometry_msgs::Point point){
  geometry_msgs::Point current_point = position.position;
  geometry_msgs::Quaternion current_orientation = position.orientation;
  float angle_to_do = atan2( point.y-current_point.y, point.x-current_point.x);
  ROS_INFO("Rotation to do without current orientation: %f\n", angle_to_do*180/M_PI);
  deg_from_quaternion(current_orientation);
  angle_to_do-=deg_from_quaternion(current_orientation);
  ROS_INFO("Rotation to do: %f", angle_to_do*180/M_PI);
  return angle_to_do;
}

float get_translation_to_do(geometry_msgs::Pose position, geometry_msgs::Point point){
  float translation = distancePoints(position.position, point);
  ROS_INFO("Translation to do: %f", translation);
  return translation;
}

};

int main(int argc, char **argv){

    ROS_INFO("(local_planner_node) waiting for a /local_goal_to_reach and /current_position");
    ros::init(argc, argv, "local_planner");

    local_planner bsObject;

    ros::spin();

    return 0;
}
