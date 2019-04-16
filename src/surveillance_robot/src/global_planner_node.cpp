#include "geometry_msgs/Point.h"
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
#define V 6

class global_planner {
private:
  ros::NodeHandle n;

  ros::Subscriber sub_goal;
  ros::Subscriber sub_pose;
  ros::Publisher pub_current_position;
  ros::Publisher pub_goal_to_reach;

  bool new_pose;          // check if a new position arrived
  bool new_goal_to_reach; // to check if a new /goal_to_reach is available or
                          // not

  geometry_msgs::Point goal_to_reach;
  geometry_msgs::Pose current_position_estim;

  geometry_msgs::Point p1;
  geometry_msgs::Point p2;
  geometry_msgs::Point p3;
  geometry_msgs::Point p4;

  int state;
  bool display_state;

public:
  global_planner() {

    state = 1;
    display_state = false;
    new_goal_to_reach = false;

    // Set up point coordinates
    p1.x = 17.878;
    p1.y = -22.837;
    p2.x = 13.603;
    p2.y = -8.464;
    p3.x = 9.795;
    p3.y = 6.990;
    p4.x = 26.651;
    p4.y = -4.985;

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

    float d12 = distancePoints(p1, p2);
    float d23 = distancePoints(p2, p3);
    float d24 = distancePoints(p2, p4);

    if (current_position_estim.position.y < p3.y &&
            current_position_estim.position.y > p2.y && goal_to_reach.x < p4.x &&
            goal_to_reach.x > p2.x ||
            goal_to_reach.y < p3.y && goal_to_reach.y > p2.y &&
            current_position_estim.x < p4.x &&
            current_position_estim.x > p2.x) { // FIXME
      float d3c = distancePoints(p3, current_position_estim.position);
      float d2c = distancePoints(p2, current_position_estim.position);
      float d2g = distancePoints(p2, goal_to_reach);
      float d4g = distancePoints(p4, goal_to_reach);

      float graph[6][6] = {{0, d12, 0, 0, 0, 0},   {d12, 0, 0, 0, d2c, d2g},
                           {0, 0, 0, 0, d3c, 0},   {0, 0, 0, 0, 0, d4g},
                           {0, d2c, d3c, 0, 0, 0}, {0, d2g, 0, d4g, 0, 0}};
      dijkstra(graph, current_position_estim);
    }

    if (current_position_estim.position.y < p2.y &&
            current_position_estim.position.y > p1.y && goal_to_reach.x < p4.x &&
            goal_to_reach.x > p2.x ||
            goal_to_reach.y < p2.y && goal_to_reach.y > p1.y &&
            current_position_estim.positionx < p4.x &&
            current_position_estim.position.x > p2.x) { // FIXME
      float d1c = distancePoints(p1, current_position_estim.position);
      float d2c = distancePoints(p2, current_position_estim.position);
      float d2g = distancePoints(p2, goal_to_reach);
      float d4g = distancePoints(p4, goal_to_reach);

      float graph[6][6] = {{0, 0, 0, 0, d1c, 0},   {0, 0, d23, 0, d2c, d2g},
                           {0, d23, 0, 0, 0, 0},   {0, 0, 0, 0, 0, d4g},
                           {d1c, d2c, 0, 0, 0, 0}, {0, d2g, 0, d4g, 0, 0}};
      dijkstra(graph, current_position_estim);
    }
  }

  // UPDATE: main processing
  /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
  void update() {
    if (!display_state) {
      display_state = true;
      ROS_INFO("state: %i", state);
    }

    // we receive a new /goal_to_reach and robair is not doing a translation or
    // a rotation
    if (new_goal_to_reach) {
      // calculation
      // publish
      ROS_INFO("publishing new goal to reach");
      pub_goal_to_reach.publish(goal_to_reach);
      new_goal_to_reach = false;
    }

    if (new_pose) {
      estimate_graph_position();
    }

  } // update

  // A utility function to find the vertex with minimum distance value, from
  // the set of vertices not yet included in shortest path tree
  int minDistance(int dist[], bool sptSet[]) {
    // Initialize min value
    int min = INT_MAX, min_index;

    for (int v = 0; v < V; v++)
      if (sptSet[v] == false && dist[v] <= min)
        min = dist[v], min_index = v;

    return min_index;
  }

  /* // A utility function to print the constructed distance array
  int printSolution(int dist[], int n)
  {
      printf("Vertex Distance from Source\n");
      for (int i = 0; i < V; i++)
          printf("%d tt %d\n", i, dist[i]);
  }  */

  // Function to print shortest
  // path from source to j
  // using path array
  void printPath(int path[], int j) {
    // Base Case : If j is source
    if (path[j] == -1)
      return;

    printPath(path, path[j]);
  }

  // A utility function to print
  // the constructed distance
  // array
  int printSolution(int dist[], int n, int path[]) {
    int src = 0;
    printf("Vertex\t Distance\tPath");
    for (int i = 1; i < V; i++) {
      printf("\n%d -> %d \t\t %d\t\t%d ", src, i, dist[i], src);
      printPath(path, i);
    }
  }

  // Function that implements Dijkstra's single source shortest path algorithm
  // for a graph represented using adjacency matrix representation
  void dijkstra(float graph[V][V], int currentposition) {
    int dist[V]; // The output array.  dist[i] will hold the shortest
    // distance from currentposition to i

    bool
        sptSet[V]; // sptSet[i] will be true if vertex i is included in shortest
    // path tree or shortest distance from currentposition to i is finalized

    // path array to store
    // shortest path tree
    int path[V];

    // Initialize all distances as
    // INFINITE and stpSet[] as false
    for (int i = 0; i < V; i++) {
      path[0] = -1;
      dist[i] = INT_MAX;
      sptSet[i] = false;
    }

    // Distance of source vertex from itself is always 0
    dist[currentposition] = 0;

    // Find shortest path for all vertices
    for (int count = 0; count < V - 1; count++) {
      // Pick the minimum distance vertex from the set of vertices not
      // yet processed. u is always equal to currentposition in the first
      // iteration.
      int u = minDistance(dist, sptSet);

      // Mark the picked vertex as processed
      sptSet[u] = true;

      // Update dist value of the adjacent vertices of the picked vertex.
      for (int v = 0; v < V; v++)
        // Update dist[v] only if is
        // not in sptSet, there is
        // an edge from u to v, and
        // total weight of path from
        // src to v through u is smaller
        // than current value of
        // dist[v]
        if (!sptSet[v] && graph[u][v] && dist[u] + graph[u][v] < dist[v]) {
          path[v] = u;
          dist[v] = dist[u] + graph[u][v];
        }
    }

    // print the constructed distance array
    // printSolution(dist, V);
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
    ROS_INFO_STREAM("Estimated position: " << msg);
    current_position_estim.position = msg->pose.pose.position;
    current_position_estim.orientation = msg->pose.pose.orientation;
    ROS_INFO_STREAM(
        "Saved as current_position_estim :" << current_position_estim << "\n");
    new_pose = true;
  }

  void getPointGoal(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    ROS_INFO_STREAM("Received final pose: " << msg);
    goal_to_reach.x = msg->pose.position.x;
    goal_to_reach.y = msg->pose.position.y;
    ROS_INFO_STREAM("Saved as goal_to_reach :" << goal_to_reach);
    new_goal_to_reach = true;
  }
};

int main(int argc, char **argv) {
  ROS_INFO("(planner_node) waiting for a /goal_to_reach and "
           "/current_position_estim");
  ros::init(argc, argv, "planner");

  global_planner bsObject;

  ros::spin();

  return 0;
}
