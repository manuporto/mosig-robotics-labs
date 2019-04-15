#include <iostream>
#include <list>
 
using namespace std;
 
// This class represents a directed graph using adjacency list representation
class Graph
{
        int V; // No. of vertices
        list<int> *adj; // Pointer to an array containing adjacency lists
        int *points;
    public:
        Graph(int V); // Constructor
        void addEdge(int v, int w); // function to add an edge to graph
        list<int> getPath(int s, int d); // returns true if there is a path from s to d
};
 
Graph::Graph(int V)
{
    this->V = V;
    bool *visited = new bool[V];
    adj = new list<int> [V];
}
 
void Graph::addEdge(int v, int w)
{
    adj[v].push_back(w); // Add w to v’s list.
}
 
// A BFS based function to check whether d is reachable from s.
list<int> Graph::getPath(int start, int goal)
{
    list<int> path;
    int s;

    path.push_back(start);

    // Base case
    if (start == goal)
        return path;
 
    // Mark all the vertices as not visited
    bool *visited = new bool[V];
    for (int i = 0; i < V; i++){
        visited[i] = false;
    }
 
    // Create a queue for BFS
    list<int> queue;
 
    // Mark the current node as visited and enqueue it
    visited[start] = true;
    queue.push_back(start);
 
    // it will be used to get all adjacent vertices of a vertex
    list<int>::iterator i;
 
    while (!queue.empty())
    {
        // Dequeue a vertex from queue and print it
        s = queue.front();
        queue.pop_front();
 
        // Get all adjacent vertices of the dequeued vertex s
        // If a adjacent has not been visited, then mark it visited
        // and enqueue it
        for (i = adj[s].begin(); i != adj[s].end(); ++i)
        {
            // If this adjacent node is the destination node, then return true
            if (*i == goal){
                if(s != start)
                    path.push_back(s);
                path.push_back(goal);
                return path; 
            }         
            
 
            // Else, continue to do BFS
            if (!visited[*i])
            {
                visited[*i] = true;
                queue.push_back(*i);
            }
        }
    }
 
    path.pop_front();
    return path;
}
 
// Driver program to test methods of graph class
int main()
{
    //       3
    //       |
    // 0-----1-----2
    //Points [0: (x0,y0), 1: (x1,y1), 2: (x2,y2), 3: (y3,y3)]
    
    // Create a graph given in the above diagram
    Graph g(4);
    g.addEdge(0, 1);
    g.addEdge(1, 0);
    g.addEdge(1, 2);
    g.addEdge(2, 1);
    g.addEdge(1, 3);
    g.addEdge(3, 1);
    
    int u, v;
    u = 2;
    v = 2;
    
    for (auto x : g.getPath(u, v))
        std::cout << x << "\n";
         
    return 0;
}