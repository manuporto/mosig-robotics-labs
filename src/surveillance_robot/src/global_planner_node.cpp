#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "json.hpp"
#include "map_file_parser.hpp"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <limits.h>
#include <stdio.h>
#include <tf/transform_datatypes.h>

// Number of vertices in the graph
#define V 4

class global_planner {
private:
  ros::NodeHandle n;

  // Communication with decision node
  ros::Subscriber sub_initial_position;
  ros::Subscriber sub_goal;
  ros::Publisher pub_path_to_go;

  bool new_initial_position; // check if a new position arrived
  bool new_goal_to_reach;    // to check if a new /goal_to_reach is available or
                             // not

  geometry_msgs::Point goal_to_reach;
  geometry_msgs::Point initial_position;

  geometry_msgs::Pose pointArray[V];
  geometry_msgs::Pose finalPathPoint[V];
  int graph[V][V];

  int state;
  bool display_state;

  int srcPoint;
  int goalPoint;

public:
  global_planner() {

    // Communication with decision node
    sub_goal = n.subscribe("move_base_simple/goal", 1,
                           &global_planner::final_goal_to_reachCallback, this);
    sub_initial_position =
        n.subscribe("/initialpose", 1, &global_planner::getPosition, this);
    pub_path_to_go =
        n.advertise<geometry_msgs::PoseArray>("global_planer/planned_path", 1);

    new_initial_position = false;
    new_goal_to_reach = false;

    MapFileParser mparser(
        "src/surveillance_robot/res/corridor/corridor_graph.json");
    ROS_INFO_STREAM("Number of vertices: " << mparser.getNumberOfVertices());
    std::vector<std::pair<float, float>> points = mparser.getPoints();
    for (int i = 0; i < points.size(); i++) {
      ROS_INFO_STREAM("x: " << points[i].first << " y: " << points[i].second);
      pointArray[i].position.x = points[i].first;
      pointArray[i].position.y = points[i].second;
    }
    std::vector<std::vector<int>> adj = mparser.getAdjacencyMatrix();
    for (int i = 0; i < adj.size(); i++) {
      for (int j = 0; j < adj[i].size(); j++) {
        ROS_INFO_STREAM("Graph: " << adj[i][j]);
        graph[i][j] = adj[i][j];
      }
    }
    // estimate_graph_position();

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
    for (int i = 0; i < 14; i++)
      for (int j = 0; j < 14; j++) {
        if (abs(i - j) == 1)
          graph[i][j] = 1;
        else
          graph[i][j] = 0;
      }
    graph[14][15] = 0;
    graph[15][14] = 0;

    for (int i = 15; i < 20; i++)
      for (int j = 15; j < 20; j++) {
        if (abs(i - j) == 1)
          graph[i][j] = 1;
        else
          graph[i][j] = 0;
      }
  }

  // UPDATE: main processing
  /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
  void update() {

    if (new_initial_position && new_goal_to_reach) {
      srcPoint = get_nearest_node(pointArray, V, initial_position);
      goalPoint = get_nearest_node(pointArray, V, goal_to_reach);
      ROS_INFO_STREAM("srcPoint: " << srcPoint << " goalPoint: " << goalPoint
                                   << std::endl);
      dijkstra(graph, srcPoint);

      ROS_INFO("Publishing new path to go");
      geometry_msgs::PoseArray pubFinalPathPoint;
      pubFinalPathPoint.poses.assign(finalPathPoint, finalPathPoint + V);
      pub_path_to_go.publish(pubFinalPathPoint);
      new_goal_to_reach = false;
      new_initial_position = false;
    }

  } // update

  // A utility function to find the vertex with minimum distance value, from
  // the set of vertices not yet included in shortest path tree
  int minDistance(int dist[], bool sptSet[]) {
    // Initialize min value
    int min = INT_MAX, min_index;

    for (int i = 0; i < V; i++)
      if (sptSet[i] == false && dist[i] <= min)
        min = dist[i], min_index = i;

    return min_index;
  }

  void getFinalPathPoint(int arr[]) {
    int j = 0;
    for (int i = 0; i < V; i++) {
      j = arr[i];
      finalPathPoint[i] = pointArray[j];
      ROS_INFO_STREAM("========= Path: " << finalPathPoint[i] << " ");
    }
    ROS_INFO_STREAM(std::endl);
  }

  // Function to print shortest
  // path from source to goal
  // using path array
  void printPath(int path[], int j) {
    // Base Case : If j is source
    if (path[j] == -1)
      return;
    ROS_INFO_STREAM("Path j: " << path[j] << " J: " << j << std::endl);
    printPath(path, path[j]);

    int finalpath[V];
    for (int i = 0; i < V; i++) {
      ROS_INFO_STREAM("I: " << i << std::endl);
      finalpath[i] = j;
    }
    getFinalPathPoint(finalpath);
  }

  // Function that implements Dijkstra's single source shortest path algorithm
  // for a graph represented using adjacency matrix representation
  void dijkstra(int graph[V][V], int src) {
    ROS_INFO("CHECK 1");
    // The output array.  dist[i] will hold the shortest distance from src to i
    int dist[V];

    // sptSet[i] will be true if vertex i is included in shortest
    // path tree or shortest distance from src to i is finalized
    bool sptSet[V];

    // path array to store
    // shortest path tree
    int path[V];

    // Initialize all distances as
    // INFINITE and stpSet[] as false
    for (int i = 0; i < V; i++) {
      path[0] = -1;
      dist[i] = std::numeric_limits<int>::max();
      sptSet[i] = false;
    }

    // Distance of source vertex from itself is always 0
    dist[src] = 0;
    ROS_INFO("CHECK 2");
    // Find shortest path for all vertices
    for (int count = 0; count < V - 1; count++) {
      // Pick the minimum distance vertex from the set of vertices not
      // yet processed. u is always equal to src in the first
      // iteration.
      int u = minDistance(dist, sptSet);

      // Mark the picked vertex as processed
      sptSet[u] = true;

      // Update dist value of the adjacent vertices of the picked vertex.
      for (int i = 0; i < V; i++)
        // Update dist[V] only if is
        // not in sptSet, there is
        // an edge from u to V, and
        // total weight of path from
        // src to V through u is smaller
        // than current value of
        // dist[V]
        if (!sptSet[i] && graph[u][i] && dist[u] + graph[u][i] < dist[i]) {
          ROS_INFO_STREAM("U: " << u << std::endl);
          path[i] = u;
          dist[i] = dist[u] + graph[u][i];
        }
    }
    ROS_INFO("CHECK 3");
    // print the constructed distance array
    printPath(path, goalPoint);
    ROS_INFO("CHECK 4");
  }

  size_t get_nearest_node(geometry_msgs::Pose nodes[], size_t nodes_len,
                          geometry_msgs::Point current_node) {
    float min_distance = std::numeric_limits<float>::max();
    size_t nearest_node = 0;
    for (size_t i = 0; i < nodes_len; i++) {
      ROS_INFO_STREAM("Node: " << nodes[i].position.x << ", "
                               << nodes[i].position.y << std::endl);
      float distance = distancePoints(nodes[i].position, current_node);
      ROS_INFO_STREAM("Distance: " << distance << std::endl);
      if (distance < min_distance) {
        ROS_INFO_STREAM("New Min Distance: " << distance << std::endl);
        min_distance = distance;
        nearest_node = i;
      }
    }
    return nearest_node;
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
    ROS_INFO_STREAM("Initial position: " << msg);
    initial_position = msg->pose.pose.position;
    ROS_INFO_STREAM("Saved as initial_position :" << initial_position << "\n");
    new_initial_position = true;
  }

  void getPointGoal(const geometry_msgs::Point::ConstPtr &msg) {
    ROS_INFO_STREAM("Received new global goal: " << msg);
    goal_to_reach.x = msg->x;
    goal_to_reach.y = msg->y;
    ROS_INFO_STREAM("Saved as goal_to_reach :" << goal_to_reach);
    new_goal_to_reach = true;
  }

  void
  final_goal_to_reachCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    ROS_INFO_STREAM("Received global goal " << msg);
    goal_to_reach.x = msg->pose.position.x;
    goal_to_reach.y = msg->pose.position.y;
    new_goal_to_reach = true;
  }
};

int main(int argc, char **argv) {
  ROS_INFO("(planner_node) waiting for a /goal_to_reach and an "
           "/initial_position");
  ros::init(argc, argv, "planner");

  global_planner bsObject;

  ros::spin();

  return 0;
}
