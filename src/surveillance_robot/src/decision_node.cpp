#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_datatypes.h>

class decision {
private:

    ros::NodeHandle n;

    // communication with one_moving_person_detector or person_tracker
    ros::Publisher pub_goal_reached;


    //comunication with local_planner
    ros::Subscriber sub_current_position;
    ros::Subscriber sub_final_goal_to_reach;
    ros::Subscriber sub_tran_rot_to_do;

    //TODO communication with global_planer

    // communication with rotation
    ros::Publisher pub_rotation_to_do;
    ros::Subscriber sub_rotation_done;

    // communication with translation
    ros::Publisher pub_translation_to_do;
    ros::Subscriber sub_translation_done;

    float rotation_to_do;
    float rotation_done;
    float translation_to_do;
    float translation_done;


    bool new_translation;
    bool new_rotation;
    bool new_final_goal_to_reach;//to check if a new /final_goal_to_reach is available or not
    bool new_current_position;//to check if a new current position estimation is available or not
    bool new_rotation_done;//to check if a new /rotation_done is available or not
    bool new_translation_done;//to check if a new /translation_done is available or not

    geometry_msgs::Pose current_position;
    geometry_msgs::Point final_goal_to_reach;
    geometry_msgs::Point goal_to_reach;
    geometry_msgs::Point goal_reached;

    int state;
    bool display_state;
    float distance_destination_threshold = 0.5; //to check if we're close enough to our final goal

public:

decision() {

    // communication with moving_persons_detector or person_tracker
    pub_goal_reached = n.advertise<geometry_msgs::Point>("goal_reached", 1);
    sub_final_goal_to_reach = n.subscribe("final_goal_to_reach", 1, &decision::final_goal_to_reachCallback, this);
    // sub_current_position = n.subscribe("current_position", 1, &decision::current_positionCallback, this);
    sub_tran_rot_to_do = n.subscribe("translation_rotation", 1, &decision::translation_rotation, this);
    // communication with rotation_action
    pub_rotation_to_do = n.advertise<std_msgs::Float32>("rotation_to_do", 0);
    sub_rotation_done = n.subscribe("rotation_done", 1, &decision::rotation_doneCallback, this);

    // communication with translation_action
    pub_translation_to_do = n.advertise<std_msgs::Float32>("translation_to_do", 0);
    sub_translation_done = n.subscribe("translation_done", 1, &decision::translation_doneCallback, this);

    state = 1;
    display_state = false;
    new_final_goal_to_reach = false;
    new_rotation_done = false;
    new_translation_done = false;
    new_current_position = false;

    //INFINITE LOOP TO COLLECT LASER DATA AND PROCESS THEM
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

    // we receive a new translation and rotation and robair is not doing a translation or a rotation
    if (new_rotation && new_translation && ( state == 1 ) ) {

          // we have a rotation and a translation to perform

          if ( rotation_to_do && state == 1) {
              display_state = false;
              //we first perform the /rotation_to_do
              ROS_INFO("(decision_node) /rotation_to_do: %f", rotation_to_do*180/M_PI);
              std_msgs::Float32 msg_rotation_to_do;

              msg_rotation_to_do.data = rotation_to_do;
              pub_rotation_to_do.publish(msg_rotation_to_do);
              state = 2;
              new_rotation = false;
          }

          //if there's no ratation, do translation
          if ( translation_to_do && state == 1) {
              display_state = false;
              ROS_INFO("(decision_node) /tranlation: %f", translation_to_do);
              std_msgs::Float32 msg_translation_to_do;

              msg_translation_to_do.data = translation_to_do;
              pub_translation_to_do.publish(msg_translation_to_do);
              state = 3;
              new_translation = false;
              ROS_INFO("======================================================");
          }
    }

    //we receive an ack from rotation_action_node. So, we perform the /translation_to_do
    if ( ( new_rotation_done ) && ( state == 2 ) ) {
        ROS_INFO("(decision_node) /rotation_done : %f", rotation_done*180/M_PI);
        new_rotation_done = false;

        display_state = false;
        //the rotation_to_do is done so we perform the translation_to_do
        ROS_INFO("(decision_node) /translation_to_do: %f", translation_to_do);
        std_msgs::Float32 msg_translation_to_do;
        //to complete
          msg_translation_to_do.data = translation_to_do;
          pub_translation_to_do.publish(msg_translation_to_do);
          state = 3;

    }

    //we receive an ack from translation_action_nodnew_current_positione. So, we send an ack to the moving_persons_detector_node
    if ( ( new_translation_done ) && ( state == 3 ) ) {
        ROS_INFO("(decision_node) /translation_done : %f\n", translation_done);
        new_translation_done = false;

        display_state = false;
        //the translation_to_do is done so we inform the planner
        // geometry_msgs::Point msg_goal_reached;
        // ROS_INFO("(decision_node) /goal_reached (%f, %f)", msg_goal_reached.x, msg_goal_reached.y);
        state = 1;

        ROS_INFO("======================================================");
        ROS_INFO("(decision_node) waiting for new info from local planner");
    }
}// update



//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void translation_rotation(const geometry_msgs::Point::ConstPtr& msg) {
// process the goal received from local planner

    // ROS_INFO_STREAM(msg->x << " " << msg->y);
    new_translation = true;
    new_rotation = true;

    translation_to_do = msg->x;
    rotation_to_do = msg->y;

    ROS_INFO("Translation to do: %f Rotation to do: %f", translation_to_do, rotation_to_do);

}

void final_goal_to_reachCallback(const geometry_msgs::Point::ConstPtr& msg) {
// process the goal received from local planner
    new_final_goal_to_reach = true;

    final_goal_to_reach.x = msg->x;
    final_goal_to_reach.y = msg->y;

    ROS_INFO("GOAL TO  REACH: x: %f y: %f", final_goal_to_reach.x, final_goal_to_reach.y);

}

// void current_positionCallback(const geometry_msgs::Pose::ConstPtr& msg) {
// // process current_position received from local planner
//
//     new_current_position = true;
//
//     current_position.position = msg->position;
//     current_position.orientation = msg->orientation;
//
//     ROS_INFO_STREAM("=====POSITION ESTIMATION: " << current_position);
//
// }

void rotation_doneCallback(const std_msgs::Float32::ConstPtr& a) {
// process the angle received from the rotation node

    new_rotation_done = true;
    rotation_done = a->data;

}

void translation_doneCallback(const std_msgs::Float32::ConstPtr& r) {
// process the range received from the translation node
    new_translation_done = true;
    translation_done = r->data;

}

};

int main(int argc, char **argv){

    ROS_INFO("(decision_node)");
    ros::init(argc, argv, "decision");

    decision bsObject;

    ros::spin();

    return 0;
}
