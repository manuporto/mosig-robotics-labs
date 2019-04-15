#ifndef GRAPH_H
#define GRAPH_H

#include <list>

// This class represents a directed graph using adjacency list representation
class Graph {
  int V;               // No. of vertices
  std::list<int> *adj; // Pointer to an array containing adjacency lists
  int *points;

public:
  Graph(int V);               // Constructor
  void addEdge(int v, int w); // function to add an edge to graph
  std::list<int> getPath(int s,
                         int d); // returns true if there is a path from s to d
};

#endif