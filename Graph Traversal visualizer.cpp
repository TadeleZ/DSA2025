#include <iostream>
#include <queue>
#include <stack>
#include <limits.h> // For INT_MAX
using namespace std;

const int MAX_NODES = 100; // Define a maximum number of nodes

class GraphTraversalVisualizer {
private:
    int adjList[MAX_NODES][MAX_NODES]; // Adjacency matrix
    int weights[MAX_NODES][MAX_NODES];  // Weights of edges
    bool exists[MAX_NODES];              // To track if a node exists

public:
    GraphTraversalVisualizer() {
        // Initialize the graph
        for (int i = 0; i < MAX_NODES; ++i) {
            exists[i] = false;
            for (int j = 0; j < MAX_NODES; ++j) {
                adjList[i][j] = -1; // -1 indicates no edge
                weights[i][j] = 0;  // Initialize weights to 0
            }
        }
    }

    void addNode(int node) {
        if (node >= 0 && node < MAX_NODES && !exists[node]) {
            exists[node] = true;
            cout << "Node " << node << " added.\n";
        } else {
            cout << "Node " << node << " already exists or is out of bounds.\n";
        }
    }

    void deleteNode(int node) {
        if (node >= 0 && node < MAX_NODES && exists[node]) {
            // Remove all edges pointing to this node
            for (int i = 0; i < MAX_NODES; ++i) {
                adjList[i][node] = -1; // Remove edge
            }
            exists[node] = false;
            cout << "Node " << node << " deleted.\n";
        } else {
            cout << "Node " << node << " does not exist.\n";
        }
    }

    void addEdge(int src, int dest, int weight) {
        if (src >= 0 && src < MAX_NODES && dest >= 0 && dest < MAX_NODES && exists[src] && exists[dest]) {
            adjList[src][dest] = dest; // Add edge
            weights[src][dest] = weight; // Set weight
            cout << "Edge added between " << src << " and " << dest << " with weight " << weight << ".\n";
        } else {
            cout << "One or both nodes do not exist.\n";
        }
    }

    void updateWeight(int src, int dest, int newWeight) {
        if (src >= 0 && src < MAX_NODES && dest >= 0 && dest < MAX_NODES && exists[src] && exists[dest]) {
            if (adjList[src][dest] != -1) {
                weights[src][dest] = newWeight;
                cout << "Weight updated for edge between " << src << " and " << dest << " to " << newWeight << ".\n";
            } else {
                cout << "Edge between " << src << " and " << dest << " does not exist.\n";
            }
        } else {
            cout << "One or both nodes do not exist.\n";
        }
    }

    void bfs(int startNode) {
        if (startNode < 0 || startNode >= MAX_NODES || !exists[startNode]) {
            cout << "Start node " << startNode << " does not exist.\n";
            return;
        }

        bool visited[MAX_NODES] = {false};
        queue<int> q;

        q.push(startNode);
        visited[startNode] = true;

        cout << "BFS Traversal: ";
        while (!q.empty()) {
            int current = q.front();
            q.pop();
            cout << current << " ";

            // Iterate over neighbors
            for (int i = 0; i < MAX_NODES; ++i) {
                if (adjList[current][i] != -1 && !visited[i]) {
                    visited[i] = true;
                    q.push(i);
                }
            }
        }
        cout << endl;
    }

    void dfs(int startNode) {
        if (startNode < 0 || startNode >= MAX_NODES || !exists[startNode]) {
            cout << "Start node " << startNode << " does not exist.\n";
            return;
        }

        bool visited[MAX_NODES] = {false};
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
                for (int i = MAX_NODES - 1; i >= 0; --i) {
                    if (adjList[current][i] != -1 && !visited[i]) {
                        s.push(i);
                    }
                }
            }
        }
        cout << endl;
    }

    void showShortestPath(int startNode, int endNode) {
        if (startNode < 0 || startNode >= MAX_NODES || endNode < 0 || endNode >= MAX_NODES || !exists[startNode] || !exists[endNode]) {
            cout << "Start or end node does not exist.\n";
            return;
        }

        int distances[MAX_NODES];
        int previous[MAX_NODES];
        bool visited[MAX_NODES] = {false};

        // Initialize distances
        for (int i = 0; i < MAX_NODES; ++i) {
            distances[i] = INT_MAX;
            previous[i] = -1;
        }
        distances[startNode] = 0;

        for (int count = 0; count < MAX_NODES - 1; ++count) {
            int minIndex = -1;
            int minValue = INT_MAX;

            // Find the unvisited node with the smallest distance
            for (int i = 0; i < MAX_NODES; ++i) {
                if (!visited[i] && distances[i] < minValue) {
                    minValue = distances[i];
                    minIndex = i;
                }
            }

            if (minIndex == -1) break; // No more reachable nodes

            visited[minIndex] = true;

            // Update distances for neighbors
            for (int i = 0; i < MAX_NODES; ++i) {
                if (adjList[minIndex][i] != -1 && !visited[i]) {
                    int newDist = distances[minIndex] + weights[minIndex][i];
                    if (newDist < distances[i]) {
                        distances[i] = newDist;
                        previous[i] = minIndex;
                    }
                }
            }
        }

        // Reconstruct the shortest path
        if (distances[endNode] == INT_MAX) {
            cout << "No path exists from " << startNode << " to " << endNode << ".\n";
            return;
        }

        cout << "Shortest path from " << startNode << " to " << endNode << ": ";
        int currentNode = endNode;
        while (currentNode != -1) {
            cout << currentNode << " ";
            currentNode = previous[currentNode];
        }
        cout << "\nTotal distance: " << distances[endNode] << endl;
    }

    void printGraph() {
        cout << "Graph:\n";
        for (int i = 0; i < MAX_NODES; ++i) {
            if (exists[i]) {
                cout << i << " -> ";
                for (int j = 0; j < MAX_NODES; ++j) {
                    if (adjList[i][j] != -1) {
                        cout << "(" << j << ", " << weights[i][j] << ") ";
                    }
                }
                cout << endl;
            }
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
