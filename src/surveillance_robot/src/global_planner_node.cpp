#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "map_file_parser.hpp"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <limits.h>
#include <stdio.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <visualization_msgs/Marker.h>
class global_planner {
private:
  ros::NodeHandle n;

  // Communication with decision node
  ros::Subscriber sub_initial_position;
  ros::Subscriber sub_goal;
  ros::Publisher pub_path_to_go;
  ros::Publisher pub_path_rviz;

  bool new_initial_position; // check if a new position arrived
  bool new_goal_to_reach;    // to check if a new /goal_to_reach is available or
                             // not

  geometry_msgs::Pose goal_to_reach;
  geometry_msgs::Point initial_position;

  std::vector<geometry_msgs::Pose> pointArray;
  std::vector<geometry_msgs::Pose> finalPathPoint;
  int number_of_vertices;
  std::vector<std::vector<int>> graph;
  std::vector<std::vector<float>> graph_dist;

  int state;
  bool display_state;

  int srcPoint;
  int goalPoint;

  visualization_msgs::Marker marker;

public:
  global_planner() {

    // Communication with decision node
    sub_goal = n.subscribe("global_planner/global_goal", 1,
                           &global_planner::final_goal_to_reachCallback, this);
    sub_initial_position =
        n.subscribe("amcl_pose", 1, &global_planner::getPosition, this);
    pub_path_to_go = n.advertise<geometry_msgs::PoseArray>(
        "global_planner/planned_path", 1, true);

    pub_path_rviz =
        n.advertise<visualization_msgs::Marker>("global_planner/rviz", 1, true);

    new_initial_position = false;
    new_goal_to_reach = false;

    MapFileParser mparser(
        "src/surveillance_robot/res/corridor/corridor_graph.json");
    number_of_vertices = mparser.getNumberOfVertices();
    std::vector<std::pair<float, float>> points = mparser.getPoints();

    // display all intermidiate poitns in RVIZ
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // marker.color.a = 0.5;
    // marker.color.r = 0.0;
    // marker.color.g = 0.0;
    // marker.color.b = 1.0;

    for (int i = 0; i < points.size(); i++) {
      geometry_msgs::Pose aPose;
      aPose.position.x = points[i].first;
      aPose.position.y = points[i].second;
      pointArray.push_back(aPose);

      // fill the marker message
      geometry_msgs::Point p;
      p.x = aPose.position.x;
      p.y = aPose.position.y;
      std_msgs::ColorRGBA c;
      c.a = 1.0;
      c.g = 0.0;
      c.r = 0.0;
      c.b = 1.0;
      p.x = aPose.position.x;
      p.y = aPose.position.y;
      marker.points.push_back(p);
      marker.colors.push_back(c);
    }

    pub_path_rviz.publish(marker);
    ROS_INFO("Published to rviz");

    graph = mparser.getAdjacencyMatrix();
    estimate_graph_position();

    // INFINTE LOOP TO COLLECT POSITION DATA AND PROCESS THEM
    ros::Rate r(10); // this node will work at 10hz
    while (ros::ok()) {
      ros::spinOnce(); // each callback is called once
      update();
      r.sleep(); // we wait if the processing (ie, callback+update) has taken
                 // less than 0.1s (ie, 10 hz)
    }
  }

  void estimate_graph_position() {
    for (int i = 0; i < number_of_vertices; i++) {
      std::vector<float> row;
      graph_dist.push_back(row);
      for (int j = 0; j < number_of_vertices; j++) {
        if (graph[i][j] == 1) {
          graph_dist[i].push_back(
              distancePoints(pointArray[i].position, pointArray[j].position));
        } else
          graph_dist[i].push_back(0);
      }
    }
  }

  // UPDATE: main processing
  /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
  void update() {

    if (new_initial_position && new_goal_to_reach) {

      geometry_msgs::Point goal_to_reach_point;
      goal_to_reach_point.x = goal_to_reach.position.x;
      goal_to_reach_point.y = goal_to_reach.position.y;

      srcPoint = get_nearest_point(pointArray, initial_position, goal_to_reach_point);
      goalPoint = get_nearest_point(pointArray, goal_to_reach_point, initial_position);
      ROS_INFO_STREAM("srcPoint: " << srcPoint << " goalPoint: " << goalPoint
                                   << std::endl);
      dijkstra(srcPoint);

      ROS_INFO("Publishing new path to go");
      geometry_msgs::PoseArray pubFinalPathPoint;

      pubFinalPathPoint.poses.assign(finalPathPoint.begin(),
                                     finalPathPoint.end());

      ROS_INFO("========= Path: ");
      for(geometry_msgs::Pose pose : finalPathPoint){
        //fill the marker message
        geometry_msgs::Point p;
        std_msgs::ColorRGBA c;
        c.a = 1.0;
        c.g = 1.0;
        c.r = 0.0;
        c.b = 0.0;
        p.x = pose.position.x;
        p.y = pose.position.y;
        marker.points.push_back(p);
        marker.colors.push_back(c);
        ROS_INFO_STREAM("====" << p);
      }


      pub_path_rviz.publish(marker);
      ROS_INFO("Publishing path to rviz");

      pub_path_to_go.publish(pubFinalPathPoint);
      new_goal_to_reach = false;
      new_initial_position = false;
    }

  } // update

  void getFinalPathPoint(std::vector<int> path, int n) {
    int j = 0;
    for (int i = 0; i <= n; i++) {
      j = path[i];
      finalPathPoint.push_back(pointArray[j]);
    }
    finalPathPoint.push_back(goal_to_reach);
  }

  void dijkstra(int startnode) {
    int cnt = 0, x = 0, y = 0;
    std::vector<int> path, mainPath;
    std::vector<std::vector<float>> cost;
    std::vector<float> distance;
    std::vector<int> visited, pred;
    int count, mindistance, nextnode, i, j;

    // pred[] stores the predecessor of each node
    // count gives the number of nodes seen so far
    // create the cost matrix
    for (int i = 0; i < number_of_vertices; i++) {
      std::vector<float> row_cost;
      cost.push_back(row_cost);
      for (int j = 0; j < number_of_vertices; j++) {
        if (graph_dist[i][j] == 0)
          cost[i].push_back(std::numeric_limits<int>::max());
        else
          cost[i].push_back(graph_dist[i][j]);
      }
    }

    // initialize pred[],distance[] and visited[]
    for (int i = 0; i < number_of_vertices; i++) {
      distance.push_back(cost[startnode][i]);
      pred.push_back(startnode);
      visited.push_back(0);
    }

    distance[startnode] = 0;
    visited[startnode] = 1;
    count = 1;

    while (count < number_of_vertices - 1) {
      mindistance = std::numeric_limits<int>::max();

      // nextnode gives the node at minimum distance
      for (int i = 0; i < number_of_vertices; i++)
        if (distance[i] < mindistance && !visited[i]) {
          mindistance = distance[i];
          nextnode = i;
        }

      // check if a better path exists through nextnode
      visited[nextnode] = 1;
      for (int i = 0; i < number_of_vertices; i++)
        if (!visited[i])
          if (mindistance + cost[nextnode][i] < distance[i]) {
            distance[i] = mindistance + cost[nextnode][i];
            pred[i] = nextnode;
          }
      count++;
    }

    if (goalPoint != startnode) {
      j = goalPoint;
      do {
        j = pred[j];
        path.push_back(j);
        cnt = cnt + 1;
      } while (j != startnode);
    }

    for (x = cnt - 1, y = 0; x >= 0, y < cnt; x--, y++) {
      mainPath.push_back(path[x]);
    }

    mainPath.push_back(goalPoint);
    getFinalPathPoint(mainPath, cnt);
  }

  std::pair<int, int>
  get_two_nearest_nodes(std::vector<geometry_msgs::Pose> nodes,
                        geometry_msgs::Point current_node) {
    std::vector<geometry_msgs::Pose> nodes_without_nearest(nodes);

    int first_nearest = get_nearest_node(nodes, current_node);
    nodes_without_nearest[first_nearest].position.x =
        std::numeric_limits<float>::max();
    nodes_without_nearest[first_nearest].position.y =
        std::numeric_limits<float>::max();
    int second_nearest = get_nearest_node(nodes_without_nearest, current_node);

    return std::pair<int, int>(first_nearest, second_nearest);
  }

  int get_nearest_node(std::vector<geometry_msgs::Pose> nodes,
                       geometry_msgs::Point current_node) {
    float min_distance = std::numeric_limits<float>::max();
    int nearest_node = 0;
    for (int i = 0; i < nodes.size(); i++) {
      //ROS_INFO_STREAM("Node: " << nodes[i].position.x << ", "
      //                         << nodes[i].position.y << std::endl);
      float distance = distancePoints(nodes[i].position, current_node);
      // ROS_INFO_STREAM("Distance: " << distance << std::endl);
      if (distance < min_distance) {
        // ROS_INFO_STREAM("New Min Distance: " << distance << std::endl);
        min_distance = distance;
        nearest_node = i;
      }
    }
    return nearest_node;
  }

  int get_nearest_point(std::vector<geometry_msgs::Pose> nodes, geometry_msgs::Point initial_node,
                        geometry_msgs::Point objective_node) {
    std::pair<int, int> nearest_points =
        get_two_nearest_nodes(nodes, initial_node);
    ROS_DEBUG_STREAM("Got the two nearest points: "
                     << nodes[nearest_points.first].position << " and "
                     << nodes[nearest_points.second].position);

    float distance1 = distancePoints(nodes[nearest_points.first].position,
                                     objective_node);
    float distance2 = distancePoints(nodes[nearest_points.second].position,
                                     objective_node);

    if (distance1 < distance2) {
      ROS_DEBUG_STREAM(
          "Returning first point: " << nodes[nearest_points.first].position);
      return nearest_points.first;
    } else {
      ROS_DEBUG_STREAM(
          "Returning first point: " << nodes[nearest_points.first].position);
      return nearest_points.second;
    }
  }

  // Distance between two points
  float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x - pb.x), 2.0) + pow((pa.y - pb.y), 2.0));
  }

  // CALLBACKS
  /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
  void
  getPosition(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    ROS_INFO_STREAM("Received new initial position");
    initial_position = msg->pose.pose.position;
    ROS_INFO_STREAM("Saved as initial_position :" << initial_position << "\n");
    new_initial_position = true;
  }

  void getPointGoal(const geometry_msgs::Point::ConstPtr &msg) {
    ROS_INFO_STREAM("Received new global goal: " << msg);
    goal_to_reach.position.x = msg->x;
    goal_to_reach.position.y = msg->y;
    goal_to_reach.position.z = 0;
    ROS_INFO_STREAM("Saved as goal_to_reach :" << goal_to_reach);
    new_goal_to_reach = true;
  }

  void final_goal_to_reachCallback(const geometry_msgs::Point::ConstPtr &msg) {
    ROS_INFO_STREAM("Received global goal");
    goal_to_reach.position.x = msg->x;
    goal_to_reach.position.y = msg->y;
    ROS_INFO_STREAM("Saved as goal_to_reach :" << goal_to_reach);
    new_goal_to_reach = true;
    finalPathPoint.erase(finalPathPoint.begin(), finalPathPoint.end());


  }
};

int main(int argc, char **argv) {
  ROS_INFO("(planner_node) waiting for a /goal_to_reach and an "
           "/amcl_pose");
  ros::init(argc, argv, "global_planner");

  global_planner bsObject;

  ros::spin();

  return 0;
}
