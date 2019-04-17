#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
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
#define V 21

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
  geometry_msgs::Pose initial_position;

  geometry_msgs::Pose p1;
  geometry_msgs::Pose p2;
  geometry_msgs::Pose p3;
  geometry_msgs::Pose p4;
  geometry_msgs::Pose p5;
  geometry_msgs::Pose p6;
  geometry_msgs::Pose p7;
  geometry_msgs::Pose p8;
  geometry_msgs::Pose p9;
  geometry_msgs::Pose p10;
  geometry_msgs::Pose p11;
  geometry_msgs::Pose p12;
  geometry_msgs::Pose p13;
  geometry_msgs::Pose p14;
  geometry_msgs::Pose p15;
  geometry_msgs::Pose p16;
  geometry_msgs::Pose p17;
  geometry_msgs::Pose p18;
  geometry_msgs::Pose p19;
  geometry_msgs::Pose p20;
  geometry_msgs::Pose p21;

  geometry_msgs::Pose pointArray[V] = {p1,  p2,  p3,  p4,  p5,  p6,  p7,
                                       p8,  p9,  p10, p11, p12, p13, p14,
                                       p15, p16, p17, p18, p19, p20, p21};
  geometry_msgs::Pose finalPathPoint[V];
  int graph[V][V];

  int state;
  bool display_state;

  int srcPoint;
  int goalPoint;

public:
  global_planner() {

    // Communication with decision node
    sub_goal = n.subscribe("global_planner/global_goal", 1,
                           &global_planner::getPointGoal, this);
    sub_goal = n.subscribe("global_planner/initial_position", 1,
                           &global_planner::getPosition, this);
    pub_path_to_go =
        n.advertise<geometry_msgs::PoseArray>("global_planer/planned_path", 1);

    new_initial_position = false;
    new_goal_to_reach = false;

    // Set up point coordinates
    p1.position.x = 17.878;
    p1.position.y = -22.837;

    p2.position.x = 17.878;
    p2.position.y = -20.837;

    p3.position.x = 17.878;
    p3.position.y = -18.837;

    p4.position.x = 17.878;
    p4.position.y = -16.837;

    p5.position.x = 17.878;
    p5.position.y = -14.837;

    p6.position.x = 17.878;
    p6.position.y = -12.837;

    p7.position.x = 17.878;
    p7.position.y = -10.837;

    p8.position.x = 13.603;
    p8.position.y = -8.464;

    p9.position.x = 13.603;
    p9.position.y = -6.464;

    p10.position.x = 13.603;
    p10.position.y = -4.464;

    p11.position.x = 13.603;
    p11.position.y = -2.464;

    p12.position.x = 13.603;
    p12.position.y = 0.464;

    p13.position.x = 13.603;
    p13.position.y = 2.464;

    p14.position.x = 13.603;
    p14.position.y = 4.464;

    p15.position.x = 13.603;
    p15.position.y = 6.990;

    p16.position.x = 15.603;
    p16.position.y = -4.985;

    p17.position.x = 17.603;
    p17.position.y = -4.985;

    p18.position.x = 19.603;
    p18.position.y = -4.985;

    p19.position.x = 21.603;
    p19.position.y = -4.985;

    p20.position.x = 23.603;
    p20.position.y = -4.985;

    p21.position.x = 26.651;
    p21.position.y = -4.985;

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
      srcPoint = get_nearest_node(pointArray, V, initial_position.position);
      goalPoint = get_nearest_node(pointArray, V, goal_to_reach);
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
    }
  }

  // Function to print shortest
  // path from source to goal
  // using path array
  void printPath(int path[], int j) {
    // Base Case : If j is source
    if (path[j] == -1)
      return;

    printPath(path, path[j]);

    int finalpath[V];
    for (int i = 0; i < V; i++) {
      finalpath[i] = j;
    }
    getFinalPathPoint(finalpath);
  }

  // Function that implements Dijkstra's single source shortest path algorithm
  // for a graph represented using adjacency matrix representation
  void dijkstra(int graph[V][V], int src) {
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
          path[i] = u;
          dist[i] = dist[u] + graph[u][i];
        }
    }

    // print the constructed distance array
    printPath(path, goalPoint);
  }

  size_t get_nearest_node(geometry_msgs::Pose nodes[], size_t nodes_len,
                          geometry_msgs::Point current_node) {
    float min_distance = std::numeric_limits<float>::max();
    size_t nearest_node = 0;
    for (size_t i = 0; i < nodes_len; i++) {
      if (distancePoints(nodes[i].position, current_node) < min_distance) {
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
  void getPosition(const geometry_msgs::Pose::ConstPtr &msg) {
    ROS_INFO_STREAM("Initial position: " << msg);
    initial_position.position = msg->position;
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
};

int main(int argc, char **argv) {
  ROS_INFO("(planner_node) waiting for a /goal_to_reach and an "
           "/initial_position");
  ros::init(argc, argv, "planner");

  global_planner bsObject;

  ros::spin();

  return 0;
}
