
#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <map>
#include <algorithm>
#include <limits>
#include <cstdlib> // For exit()
using namespace std;

class GraphTraversalVisualizer {
private:
    // Adjacency list representation of the graph
    map<int, map<int, int> > adjList; // node -> {neighbor -> weight}

public:
    // Add a node to the graph
    void addNode(int node) {
        if (adjList.find(node) == adjList.end()) {
            adjList[node] = map<int, int>();
            cout << "Node " << node << " added.\n";
        } else {
            cout << "Node " << node << " already exists.\n";
        }
    }

    // Delete a node from the graph
    void deleteNode(int node) {
        if (adjList.find(node) != adjList.end()) {
            // Remove the node from the adjacency list
            adjList.erase(node);

            // Remove all edges pointing to this node
            for (map<int, map<int, int> >::iterator it = adjList.begin(); it != adjList.end(); ++it) {
                it->second.erase(node);
            }
            cout << "Node " << node << " deleted.\n";
        } else {
            cout << "Node " << node << " does not exist.\n";
        }
    }

    // Add an edge between two nodes with a given weight
    void addEdge(int src, int dest, int weight) {
        if (adjList.find(src) == adjList.end() || adjList.find(dest) == adjList.end()) {
            cout << "One or both nodes do not exist.\n";
            return;
        }
        adjList[src][dest] = weight;
        cout << "Edge added between " << src << " and " << dest << " with weight " << weight << ".\n";
    }

    // Update the weight of an existing edge
    void updateWeight(int src, int dest, int newWeight) {
        if (adjList.find(src) != adjList.end() && adjList[src].find(dest) != adjList[src].end()) {
            adjList[src][dest] = newWeight;
            cout << "Weight updated for edge between " << src << " and " << dest << " to " << newWeight << ".\n";
        } else {
            cout << "Edge between " << src << " and " << dest << " does not exist.\n";
        }
    }

    // Breadth-First Search (BFS) Traversal
    void bfs(int startNode) {
        if (adjList.find(startNode) == adjList.end()) {
            cout << "Start node " << startNode << " does not exist.\n";
            return;
        }

        map<int, bool> visited;
        queue<int> q;

        q.push(startNode);
        visited[startNode] = true;

        cout << "BFS Traversal: ";
        while (!q.empty()) {
            int current = q.front();
            q.pop();
            cout << current << " ";

            // Iterate over neighbors
            for (map<int, int>::iterator it = adjList[current].begin(); it != adjList[current].end(); ++it) {
                int neighbor = it->first;
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    q.push(neighbor);
                }
            }
        }
        cout << endl;
    }

    // Depth-First Search (DFS) Traversal
    void dfs(int startNode) {
        if (adjList.find(startNode) == adjList.end()) {
            cout << "Start node " << startNode << " does not exist.\n";
            return;
        }

        map<int, bool> visited;
        stack<int> s;

        s.push(startNode);

        cout << "DFS Traversal: ";
        while (!s.empty()) {
            int current = s.top();
            s.pop();

            if (!visited[current]) {
                cout << current << " ";
                visited[current] = true;

                // Push neighbors onto the stack
                for (map<int, int>::reverse_iterator it = adjList[current].rbegin(); it != adjList[current].rend(); ++it) {
                    int neighbor = it->first;
                    if (!visited[neighbor]) {
                        s.push(neighbor);
                    }
                }
            }
        }
        cout << endl;
    }

    // Dijkstra's Algorithm to find the shortest path
    void dijkstraShortestPath(int startNode, int endNode) {
        if (adjList.find(startNode) == adjList.end() || adjList.find(endNode) == adjList.end()) {
            cout << "Start or end node does not exist.\n";
            return;
        }

        // Priority queue for Dijkstra's algorithm (min-heap)
        priority_queue<pair<int, int>, vector<pair<int, int> >, std::greater<pair<int, int> > > pq; // {distance, node}
        map<int, int> distances; // node -> shortest distance from startNode
        map<int, int> previous;  // node -> previous node in the shortest path
        map<int, bool> visited;

        // Initialize distances
        for (map<int, map<int, int> >::iterator it = adjList.begin(); it != adjList.end(); ++it) {
            distances[it->first] = numeric_limits<int>::max();
        }
        distances[startNode] = 0;
        pq.push(make_pair(0, startNode));

        while (!pq.empty()) {
            int currentDist = pq.top().first;
            int currentNode = pq.top().second;
            pq.pop();

            if (visited[currentNode]) continue;
            visited[currentNode] = true;

            // If we've reached the destination, reconstruct the path
            if (currentNode == endNode) break;

            // Explore neighbors
            for (map<int, int>::iterator it = adjList[currentNode].begin(); it != adjList[currentNode].end(); ++it) {
                int neighborNode = it->first;
                int edgeWeight = it->second;

                if (visited[neighborNode]) continue;

                int newDist = currentDist + edgeWeight;
                if (newDist < distances[neighborNode]) {
                    distances[neighborNode] = newDist;
                    previous[neighborNode] = currentNode;
                    pq.push(make_pair(newDist, neighborNode));
                }
            }
        }
        // Reconstruct the shortest path
        if (distances[endNode] == numeric_limits<int>::max()) {
            cout << "No path exists from " << startNode << " to " << endNode << ".\n";
            return;
        }

        vector<int> path;
        int currentNode = endNode;
        while (currentNode != startNode) {
            path.push_back(currentNode);
            currentNode = previous[currentNode];
        }
        path.push_back(startNode);
        reverse(path.begin(), path.end());

        // Print the shortest path
        cout << "Shortest path from " << startNode << " to " << endNode << ": ";
        for (size_t i = 0; i < path.size(); ++i) {
            cout << path[i];
            if (i < path.size() - 1) cout << " -> ";
        }
        cout << "\nTotal distance: " << distances[endNode] << endl;
    }

    // Print the graph
    void printGraph() {
        cout << "Graph:\n";
        for (map<int, map<int, int> >::iterator it = adjList.begin(); it != adjList.end(); ++it) {
            cout << it->first << " -> ";
            for (map<int, int>::iterator jt = it->second.begin(); jt != it->second.end(); ++jt) {
                cout << "(" << jt->first << ", " << jt->second << ") ";
            }
            cout << endl;
        }
    }
};

int main() {
    GraphTraversalVisualizer graph;
    int choice, node, src, dest, weight;

    while (true) {
        cout << "1. Add Node\n";
        cout << "2. Add Edge\n";
        cout << "3. Delete Node\n";
        cout << "4. Update Edge Weight\n";
        cout << "5. BFS Traversal\n";
        cout << "6. DFS Traversal\n";
        cout << "7. Find Shortest Path\n";
        cout << "8. Print Graph\n";
        cout << "9. Exit\n";
        cout << "Enter your choice: ";
        cin >> choice;

        switch (choice) {
            case 1:
                cout << "Enter node to add: ";
                cin >> node;
                graph.addNode(node);
                break;

            case 2:
                cout << "Enter source node: ";
                cin >> src;
                cout << "Enter destination node: ";
                cin >> dest;
                cout << "Enter edge weight: ";
                cin >> weight;
                graph.addEdge(src, dest, weight);
                break;

            case 3:
                cout << "Enter node to delete: ";
                cin >> node;
                graph.deleteNode(node);
                break;

            case 4:
                cout << "Enter source node: ";
                cin >> src;
                cout << "Enter destination node: ";
                cin >> dest;
                cout << "Enter new weight: ";
                cin >> weight;
                graph.updateWeight(src, dest, weight);
                break;

            case 5:
                cout << "Enter start node for BFS: ";
                cin >> node;
                graph.bfs(node);
                break;

            case 6:
                cout << "Enter start node for DFS: ";
                cin >> node;
                graph.dfs(node);
                break;

            case 7:
                cout << "Enter start node: ";
                cin >> src;
                cout << "Enter destination node: ";
                cin >> dest;
                graph.showShortestPath(src, dest);
                break;

            case 8:
                graph.printGraph();
                break;

            case 9:
                cout << "Exiting...\n";
                return 0;

            default:
                cout << "Invalid choice. Please try again.\n";
        }
    }

    return 0;
}
