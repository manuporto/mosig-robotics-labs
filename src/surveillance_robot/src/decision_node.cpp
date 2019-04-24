#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include <queue>
#include <iostream>

class decision {
private:

    ros::NodeHandle n;

    // communication with RVIZ
    ros::Subscriber sub_final_goal_to_reach;

    //comunication with local_planner
    ros::Subscriber sub_local_planner;
    ros::Publisher pub_local_planner;

    //communication with global_planer
    ros::Publisher pub_global_planner;
    ros::Subscriber sub_global_planner;

    // communication with rotation
    ros::Publisher pub_rotation_node;
    ros::Subscriber sub_rotation_done;

    // communication with translation
    ros::Publisher pub_translation_node;
    ros::Subscriber sub_translation_done;

    // communication with checker
    ros::Publisher pub_check_node;
    ros::Subscriber sub_check_node;

    //booleans for controling state machine
    bool new_global_goal;//to check if a new /global_goal is available or not
    bool new_path;
    bool new_rotation;
    bool new_translation;
    bool new_rotation_done;//to check if a new /rotation_done is available or not
    bool new_translation_done;//to check if a new /translation_done is available or not
    bool new_check;

    //values received from subscribers
    geometry_msgs::Point global_goal;
    std::queue<geometry_msgs::Point> path;
    float rotation_to_do;
    float translation_to_do;
    float rotation_done;
    float translation_done;
    bool check_reply;


    //other variables
    bool debug;
    int state;
    bool display_state;
    geometry_msgs::Point local_goal; //to store next goal between local planner and the check node

public:

decision() {
    //comunication with RVIZ
    sub_final_goal_to_reach = n.subscribe("move_base_simple/goal", 1, &decision::final_goal_to_reachCallback, this);

    //comunication with the global planner
    pub_global_planner = n.advertise<geometry_msgs::Point>("global_planner/global_goal", 1);
    sub_global_planner = n.subscribe("global_planner/planned_path", 1, &decision::path_points_Callback, this);

    //communication with the local planner
    pub_local_planner = n.advertise<geometry_msgs::Point>("local_planner/local_goal", 1);
    sub_local_planner = n.subscribe("local_planner/translation_rotation", 1, &decision::translation_rotation_Callback, this);

    // communication with rotation_action
    pub_rotation_node = n.advertise<std_msgs::Float32>("rotation_node/rotation_to_do", 1);
    sub_rotation_done = n.subscribe("rotation_node/rotation_done", 1, &decision::rotation_doneCallback, this);

    // communication with translation_action
    pub_translation_node = n.advertise<std_msgs::Float32>("translation_node/translation_to_do", 1);
    sub_translation_done = n.subscribe("translation_node/translation_done", 1, &decision::translation_doneCallback, this);

    // communication with checker
    pub_check_node = n.advertise<geometry_msgs::Point>("check_node/point", 1);
    sub_check_node = n.subscribe("check_node/recalculate_goal", 1, &decision::check_Callback, this);
    state = 0;
    display_state = false;
    new_rotation_done = false;
    new_translation_done = false;
    new_path = false;
    new_check = false;
    new_global_goal = false;

    debug = true;

    //INFINITE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(100);// this node will work at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once
        update();
        r.sleep();
    }

}

//UPDATE: main processing
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {

    if ( display_state ) {
        display_state = false;
        ROS_INFO("============state: %i============", state);
    }

    //waiting for a global goal point from rviz
    if(state == 0 && new_global_goal){
      ROS_INFO("(decision_node) /Publishing the global goal to the global planner");
      wait_user_input();

      ros::Rate rate(100);
      while(pub_global_planner.getNumSubscribers() == 0){
        rate.sleep();
      }
      pub_global_planner.publish(global_goal);
      state = 1;
      new_global_goal = false;
      display_state = true;
    }

    //waiting for the global_planner
    if(new_path && state == 1){
      ROS_INFO("(decision_node) /Publishing next local goal to the local planner");
      wait_user_input();
      local_goal = get_next_point();
      pub_local_planner.publish(local_goal);
      state = 2;
      new_path = false;
      display_state = true;
    }


    // we receive a new translation and rotation and robair is not doing a translation or a rotation
    if (new_rotation && new_translation && ( state == 2 ) ) {

          // we have a rotation and a translation to perform

          if ( rotation_to_do && state == 2) {

              //we first perform the /rotation_to_do
              ROS_INFO("(decision_node) /rotation_to_do: %f", rotation_to_do*180/M_PI);
              wait_user_input();
              std_msgs::Float32 msg_rotation_to_do;

              msg_rotation_to_do.data = rotation_to_do;
              pub_rotation_node.publish(msg_rotation_to_do);
              state = 3;
              new_rotation = false;
          }

          //if there's no rotation, do translation
          if ( translation_to_do && state == 2) {
              display_state = false;
              ROS_INFO("(decision_node) /tranlation: %f", translation_to_do);
              wait_user_input();
              std_msgs::Float32 msg_translation_to_do;

              msg_translation_to_do.data = translation_to_do;
              pub_translation_node.publish(msg_translation_to_do);
              state = 4;
              new_translation = false;
          }

          display_state = true;
    }

    //we receive an ack from rotation_action_node. So, we perform the /translation_to_do
    if ( ( new_rotation_done ) && ( state == 3 ) ) {
        ROS_INFO("(decision_node) /rotation_done : %f", rotation_done*180/M_PI);

        //the rotation_to_do is done so we perform the translation_to_do
        ROS_INFO("(decision_node) /translation_to_do: %f", translation_to_do);
        wait_user_input();
        std_msgs::Float32 msg_translation_to_do;

        msg_translation_to_do.data = translation_to_do;
        pub_translation_node.publish(msg_translation_to_do);

        new_rotation_done = false;
        display_state = true;
        state = 4;

    }

    //we receive an ack from translation_action_node. So, we send an ack to the moving_persons_detector_node
    if ( ( new_translation_done ) && ( state == 4 ) ) {
        ROS_INFO("(decision_node) /translation_done : %f\n", translation_done);
        wait_user_input();

        //the movement is done so we inform the check node
        geometry_msgs::Point assumed_position = local_goal;
        pub_check_node.publish(assumed_position);


        new_translation_done = false;
        display_state = true;
        state = 5;
        ROS_INFO("(decision_node) informing the check node");

    }

    //movement performed, check node contacted, waiting for it's reply
    if(new_check && state == 5){

      //our position is correct
      if(check_reply == false){
        ROS_INFO("(decision_node) /Our position is correct");
        wait_user_input();


        local_goal = get_next_point();
        pub_local_planner.publish(local_goal);

        state = 2;
      }

      //our position is not correct
      else{
        ROS_INFO("(decision_node) /Our position is not correct");
        wait_user_input();
        pub_global_planner.publish(global_goal);

        state = 1;
      }

    new_check = false;
    display_state = true;

    }


}// update


//return next local_goal from the path
geometry_msgs::Point get_next_point(){
  geometry_msgs::Point point = path.front();
  path.pop();
  ROS_INFO_STREAM("Next point " << point);
  return point;
}

void wait_user_input(){
  if(debug){
    std::string inputString;
    std::cout << '\n' << "Type 'next' to continue...";
    std::getline(std::cin, inputString);
    std_msgs::String msg;

    do{
    }
    while(inputString.compare("next") != 0);
    std::cout << '\n' << "ok...";
  }
}


//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

//
void check_Callback(const std_msgs::Bool::ConstPtr& msg){
  ROS_INFO_STREAM("Received new message from the check node"  << msg->data);
  new_check = true;
  check_reply = msg->data;
}

//receive new global goal to reach from rviz
void final_goal_to_reachCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    ROS_INFO_STREAM("Received global goal " << msg);
    global_goal.x = msg->pose.position.x;
    global_goal.y = msg->pose.position.y;
    new_global_goal = true;
}
//process the array of point received from the global planner
void path_points_Callback(const geometry_msgs::PoseArray::ConstPtr& msg){
  ROS_INFO("Received new path from the global planner");

  if( path.empty() == false){
    std::queue<geometry_msgs::Point> empty;
    std::swap( path, empty );
  }

  for (geometry_msgs::Pose pose : msg->poses) {
    path.push(pose.position);
  }
  new_path = true;
}
// process the translation and rotation received from local planner
void translation_rotation_Callback(const geometry_msgs::Point::ConstPtr& msg) {

    new_translation = true;
    new_rotation = true;

    translation_to_do = msg->x;
    rotation_to_do = msg->y;

    ROS_INFO("Translation to do: %f Rotation to do: %f", translation_to_do, rotation_to_do);

}

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

    ROS_INFO("Starting: (decision_node)/ waiting for a final goal");
    ros::init(argc, argv, "decision");

    decision bsObject;

    ros::spin();

    return 0;
}
