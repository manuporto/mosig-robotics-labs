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
  geometry_msgs::Point initial_position;

  geometry_msgs::Pose pointArray[V];
  geometry_msgs::Pose finalPathPoint[V];
  int graph[V][V];
  float graph_dist[V][V];

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
    for(int i=0; i<V; i++)
      for(int j=0; j<V; j++)
      {
        if (graph[i][j] == 1)
          {
            graph_dist[i][j] = distancePoints (pointArray[i].position , pointArray[j].position);
          }
        else
          graph_dist[i][j] = 0;
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
      dijkstra(graph_dist, srcPoint);

      ROS_INFO("Publishing new path to go");
      geometry_msgs::PoseArray pubFinalPathPoint;
      pubFinalPathPoint.poses.assign(finalPathPoint, finalPathPoint + V);
      pub_path_to_go.publish(pubFinalPathPoint);
      new_goal_to_reach = false;
      new_initial_position = false;
    }

  } // update

void getFinalPathPoint(int path[], int n){
    int j = 0;
    for (int i = 0; i <= n; i++) {
      j = path[i];
      finalPathPoint[i] = pointArray[j];
      ROS_INFO_STREAM("========= Path: " << finalPathPoint[i] << " ");
    }
    ROS_INFO_STREAM(std::endl);
}

void dijkstra(float graph[V][V],int startnode)
{
  int cnt=0, x=0, y=0;
  int path[V], mainPath[V];
	float cost[V][V],distance[V];
	int visited[V],pred[V],count,mindistance,nextnode,i,j;
	
	//pred[] stores the predecessor of each node
	//count gives the number of nodes seen so far
	//create the cost matrix
	for(int i=0;i<V;i++)
		for(int j=0;j<V;j++)
			if(graph[i][j]==0)
				cost[i][j]= std::numeric_limits<int>::max();
			else
				cost[i][j]=graph[i][j];
	
	//initialize pred[],distance[] and visited[]
	for(int i=0;i<V;i++)
	{
		distance[i]=cost[startnode][i];
		pred[i]=startnode;
		visited[i]=0;
	}
	
	distance[startnode]=0;
	visited[startnode]=1;
	count=1;
	
	while(count<V-1)
	{
		mindistance= std::numeric_limits<int>::max();
		
		//nextnode gives the node at minimum distance
		for(int i=0;i<V;i++)
			if(distance[i]<mindistance&&!visited[i])
			{
				mindistance=distance[i];
				nextnode=i;
			}
			
			//check if a better path exists through nextnode			
			visited[nextnode]=1;
			for(int i=0;i<V;i++)
				if(!visited[i])
					if(mindistance+cost[nextnode][i]<distance[i])
					{
						distance[i]=mindistance+cost[nextnode][i];
						pred[i]=nextnode;
					}
		count++;
	}

		if(goalPoint!=startnode)
		{			
			j=goalPoint;
			do
			{
				j=pred[j];
 				path[cnt] = j;
                cnt = cnt+1;
			}while(j!=startnode);
	  }

    for( x=cnt-1 , y=0; x>=0, y<cnt; x--, y++){
        mainPath[y] = path[x];
    }
    mainPath[y] = goalPoint;
    getFinalPathPoint(mainPath,cnt);
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
