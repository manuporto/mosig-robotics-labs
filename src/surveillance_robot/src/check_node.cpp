#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Bool.h"

#define RECALCULATION_TRESHOLD 5

class check_node {
    private:
        ros::NodeHandle n;
        ros::Subscriber sub_next_point;
        ros::Subscriber sub_pose;
        ros::Publisher pub_recalculate_goal;

        geometry_msgs::Point current_position_estim;
        geometry_msgs::Point next_point;

        bool new_next_point;
        bool new_pose;

    public:
        check_node() {
            new_next_point = false;
            new_pose = false;

            sub_next_point = n.subscribe("decision_node/next_point", 1, &check_node::getNextPoint, this);
            sub_pose = n.subscribe("amcl_pose", 1, &check_node::getPosition, this);

            pub_recalculate_goal = n.advertise<std_msgs::Bool>("check_node/recalculate_goal", 1);

            ros::Rate r(10);// this node will work at 10hz
            while (ros::ok()) {
                ros::spinOnce();//each callback is called once
                update();
                r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
            }
        }

        void update() {
            if (new_next_point && new_pose) {
                ROS_INFO("Received both a next point and a new pose.");
                ROS_INFO("Calculating if a recalculation of the current goal is needed.");
                std_msgs::Bool recalculation;
                if (recalculationNeeded()) {
                    recalculation.data = true;
                    pub_recalculate_goal.publish(recalculation);
                    ROS_INFO("Recalculation needed.");
                } else {
                    recalculation.data = false;
                    pub_recalculate_goal.publish(recalculation);
                    ROS_INFO("Recalculation not needed.");
                }
                new_next_point = false;
                new_pose = false;
            }
        }

        bool recalculationNeeded() {
            return distancePoints(next_point, current_position_estim) > RECALCULATION_TRESHOLD;
        }

        void getNextPoint(const geometry_msgs::Point incoming_next_point) {
            ROS_INFO_STREAM("Incoming next point: " << incoming_next_point);
            ROS_INFO_STREAM("Incoming next point: " << incoming_next_point);

            next_point.x = incoming_next_point.x;
            next_point.y = incoming_next_point.y;
            ROS_INFO_STREAM("Saved as next_point.");
            new_next_point = true;
        }

        void getPosition(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
            ROS_INFO_STREAM("Estimated position: " << msg);
            current_position_estim.x = msg->pose.pose.position.x;
            current_position_estim.y = msg->pose.pose.position.y;
            ROS_INFO_STREAM("Saved as current_position_estim :" << current_position_estim << "\n");
            new_pose = true;
        }

        // Distance between two points
        float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {
            return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));
        }
};

int main(int argc, char **argv){

    ROS_INFO("(check_node) waiting for an /amc_pose and /next_point");
    ros::init(argc, argv, "checker");

    check_node bsObject;

    ros::spin();

    return 0;
}
