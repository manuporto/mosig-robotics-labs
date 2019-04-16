#include "geometry_msgs/PoseArray.h"
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
#define V 21

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
  geometry_msgs::Point p5;
  geometry_msgs::Point p6;
  geometry_msgs::Point p7;
  geometry_msgs::Point p8;
  geometry_msgs::Point p9;
  geometry_msgs::Point p10;
  geometry_msgs::Point p11;
  geometry_msgs::Point p12;
  geometry_msgs::Point p13;
  geometry_msgs::Point p14;
  geometry_msgs::Point p15;
  geometry_msgs::Point p16;
  geometry_msgs::Point p17;
  geometry_msgs::Point p18;
  geometry_msgs::Point p19;
  geometry_msgs::Point p20;
  geometry_msgs::Point p21;

  geometry_msgs::Point pointArray[v] = {p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p14,p15,p16,p17,p18,p19,p20,p21};
  geometry_msgs::Point finalPathPoint[v];

  int state;
  bool display_state;

  int src = ;
  int goal = ;

public:
  global_planner() {

    state = 1;
    display_state = false;
    new_goal_to_reach = false;

    // Set up point coordinates
    p1.x = 17.878;
    p1.y = -22.837;

    p2.x = 17.878;
    p2.y = -20.837;

    p3.x = 17.878;
    p3.y = -18.837;

    p4.x = 17.878;
    p4.y = -16.837;

    p5.x = 17.878;
    p5.y = -14.837;

    p6.x = 17.878;
    p6.y = -12.837;

    p7.x = 17.878;
    p7.y = -10.837;        

    p8.x = 13.603;
    p8.y = -8.464;

    p9.x = 13.603;
    p9.y = -6.464;

    p10.x = 13.603;
    p10.y = -4.464;

    p11.x = 13.603;
    p11.y = -2.464;

    p12.x = 13.603;
    p12.y = 0.464;

    p13.x = 13.603;
    p13.y = 2.464;

    p14.x = 13.603;
    p14.y = 4.464;    

    p15.x = 13.603;
    p15.y = 6.990;

    p16.x = 15.603;
    p16.y = -4.985;

    p17.x = 17.603;
    p17.y = -4.985;

    p18.x = 19.603;
    p18.y = -4.985;

    p19.x = 21.603;
    p19.y = -4.985; 

    p20.x = 23.603;
    p20.y = -4.985;                

    p21.x = 26.651;
    p21.y = -4.985;

    // INFINTE LOOP TO COLLECT POSITION DATA AND PROCESS THEM
    ros::Rate r(10); // this node will work at 10hz
    while (ros::ok()) {
      ros::spinOnce(); // each callback is called once
      update();
      r.sleep(); // we wait if the processing (ie, callback+update) has taken
                 // less than 0.1s (ie, 10 hz)
    }
  }

  int graph[v][v];
  void estimate_graph_position() {

      for(int i=0; i<v; i++)
        for(int j=0; j<v; j++)
        {
            if( abs(i-j) == 1)
                graph[i][j] = 1;
            else
                graph[i][j] = 0;      
        }

      dijkstra(graph, src);
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

  // Function to print shortest
  // path from source to goal
  // using path array
  void printPath(int path[], int j) {
    // Base Case : If j is source
    if (path[j] == -1)
      return;

    printPath(path, path[j]);

    int finalpath[v];
    for(int i=0; i<v; i++){
        finalpath[i] = j;
    }
    int lenght= size(finalpath);
    finalPathPoint(finalpath,lenght);
  }

  void finalPathPoint(int arr[], int l){

      for(int i=0; i<l; i++){
        finalPathPoint[i]= arr[i];
      }
  }


  // A utility function to print
  // the constructed distance
  // array
/*   int printSolution(int dist[], int n, int path[]) {
    int src = 0;
    printf("Vertex\t Distance\tPath");
    for (int i = 1; i < V; i++) {
      printf("\n%d -> %d \t\t %d\t\t%d ", src, i, dist[i], src);
      printPath(path, i);
    }
  } */

  // Function that implements Dijkstra's single source shortest path algorithm
  // for a graph represented using adjacency matrix representation
  void dijkstra(int graph[V][V], int src) {
    int dist[V]; // The output array.  dist[i] will hold the shortest
    // distance from src to i

    bool sptSet[V]; // sptSet[i] will be true if vertex i is included in shortest
    // path tree or shortest distance from src to i is finalized

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
    printPath(path, goal);  
  }

  geometry_msgs::Pose get_nearest_node(const geometry_msgs::PoseArray::ConstPtr &nodes, const geometry_msgs::Pose::ConstPtr &current_node) {
    float min_distance = std::numeric_limits<float>::max();
    geometry_msgs::Pose nearest_node;
    for (geometry_msgs::Pose node : nodes->poses) {
      if (distancePoints(node.position, current_node->position) < min_distance) {
        nearest_node.position.x = node.position.x;
        nearest_node.position.y = node.position.y;
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
