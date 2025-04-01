   Documentation for Graph travrsal visualizer Implementation

This C++ program implements a **GraphTraversalVisualizer** class to represent and manipulate a weighted directed graph. The graph is stored using an adjacency list, where each node maps to its neighbors and the corresponding edge weights. The program provides a menu-driven interface for interacting with the graph, allowing users to add nodes, edges, delete nodes, update edge weights, perform traversals (Breadth-First Search and Depth-First Search), find the shortest path using Dijkstra's algorithm, and print the graph.

The implementation is designed to be modular, extensible, and user-friendly, making it suitable for educational purposes or as a foundation for more complex graph algorithms. Below is a detailed breakdown of the program's structure, functionality, and usage.

---

### Key Features of the Code:

1. **Graph Representation**:
   - The graph is represented as a `map<int, map<int, int>>` where:
     - The outer map represents nodes in the graph.
     - The inner map represents the neighbors of each node and their corresponding edge weights.
   - This representation allows efficient insertion, deletion, and lookup operations for nodes and edges.

2. **Graph Operations**:
   - **Add Node**: Adds a new node to the graph if it doesn't already exist. This operation ensures that duplicate nodes are not added.
   - **Delete Node**: Removes a node from the graph and deletes all edges connected to it, ensuring the graph remains consistent.
   - **Add Edge**: Adds a directed edge between two nodes with a specified weight. The operation checks if both nodes exist before adding the edge.
   - **Update Edge Weight**: Updates the weight of an existing edge between two nodes. If the edge does not exist, an error message is displayed.
   - **Print Graph**: Displays the graph in a readable format showing nodes and their neighbors with edge weights.

3. **Graph Traversals**:
   - **Breadth-First Search (BFS)**: Traverses the graph level by level starting from a given node. BFS uses a queue to explore nodes in order of their distance from the start node.
   - **Depth-First Search (DFS)**: Traverses the graph depth-wise using a stack-based implementation. DFS explores as far as possible along each branch before backtracking.

4. **Shortest Path Calculation**:
   - **Dijkstra's Algorithm**: Finds the shortest path between two nodes using a priority queue (min-heap) to efficiently process nodes based on their current shortest distance. The algorithm assumes non-negative edge weights.

5. **Menu-Driven Interface**:
   - Provides a user-friendly menu to interact with the graph, allowing users to perform various operations dynamically. The menu is implemented using a `while` loop and a `switch-case` structure for clear and concise handling of user input.

---

### Class Methods:

#### 1. **`addNode(int node)`**
   - **Purpose**: Adds a new node to the graph.
   - **Behavior**:
     - Checks if the node already exists in the graph.
     - If the node does not exist, it is added to the adjacency list.
     - If the node already exists, an appropriate message is displayed.
   - **Output**:
     ```
     Node X added.
     ```
     or
     ```
     Node X already exists.
     ```

#### 2. **`deleteNode(int node)`**
   - **Purpose**: Deletes a node and all edges connected to it from the graph.
   - **Behavior**:
     - Checks if the node exists in the graph.
     - If the node exists, it is removed from the adjacency list, and all edges pointing to this node are deleted.
     - If the node does not exist, an appropriate message is displayed.
   - **Output**:
     ```
     Node X deleted.
     ```
     or
     ```
     Node X does not exist.
     ```

#### 3. **`addEdge(int src, int dest, int weight)`**
   - **Purpose**: Adds a directed edge between two nodes with a specified weight.
   - **Behavior**:
     - Checks if both the source and destination nodes exist in the graph.
     - If both nodes exist, the edge is added to the adjacency list with the specified weight.
     - If one or both nodes do not exist, an appropriate message is displayed.
   - **Output**:
     ```
     Edge added between X and Y with weight Z.
     ```
     or
     ```
     One or both nodes do not exist.
     ```

#### 4. **`updateWeight(int src, int dest, int newWeight)`**
   - **Purpose**: Updates the weight of an existing edge between two nodes.
   - **Behavior**:
     - Checks if the edge between the source and destination nodes exists.
     - If the edge exists, its weight is updated to the new value.
     - If the edge does not exist, an appropriate message is displayed.
   - **Output**:
     ```
     Weight updated for edge between X and Y to Z.
     ```
     or
     ```
     Edge between X and Y does not exist.
     ```

#### 5. **`bfs(int startNode)`**
   - **Purpose**: Performs a Breadth-First Search traversal starting from the given node.
   - **Behavior**:
     - Uses a queue to traverse the graph level by level.
     - Marks nodes as visited to avoid revisiting them.
     - Displays the traversal order.
   - **Output**:
     ```
     BFS Traversal: X Y Z ...
     ```

#### 6. **`dfs(int startNode)`**
   - **Purpose**: Performs a Depth-First Search traversal starting from the given node.
   - **Behavior**:
     - Uses a stack to traverse the graph depth-wise.
     - Marks nodes as visited to avoid revisiting them.
     - Displays the traversal order.
   - **Output**:
     ```
     DFS Traversal: X Y Z ...
     ```

#### 7. **`dijkstraShortestPath(int startNode, int endNode)`**
   - **Purpose**: Computes the shortest path between two nodes using Dijkstra's algorithm.
   - **Behavior**:
     - Uses a priority queue (min-heap) to process nodes based on their current shortest distance.
     - Tracks the shortest distances and previous nodes to reconstruct the path.
     - If no path exists, an appropriate message is displayed.
   - **Output**:
     ```
     Shortest path from X to Y: X -> Z -> W -> Y
     Total distance: D
     ```
     or
     ```
     No path exists from X to Y.
     ```

#### 8. **`printGraph()`**
   - **Purpose**: Prints the graph in a human-readable format.
   - **Behavior**:
     - Iterates through the adjacency list and displays each node along with its neighbors and edge weights.
   - **Output**:
     ```
     Graph:
     X -> (Y, Z) (W, D)
     Y -> (Z, E)
     ...
     ```

---

### Example Workflow:

1. **Add Nodes**:
   - Use option `1` to add nodes to the graph. For example:
     ```
     Enter your choice: 1
     Enter node to add: 1
     Node 1 added.
     ```

2. **Add Edges**:
   - Use option `2` to connect nodes with edges. For example:
     ```
     Enter your choice: 2
     Enter source node: 1
     Enter destination node: 2
     Enter edge weight: 5
     Edge added between 1 and 2 with weight 5.
     ```

3. **Perform BFS or DFS Traversals**:
   - Use options `5` or `6` to perform traversals. For example:
     ```
     Enter your choice: 5
     Enter start node for BFS: 1
     BFS Traversal: 1 2 
     ```

4. **Find Shortest Path**:
   - Use option `7` to compute the shortest path between two nodes. For example:
     ```
     Enter your choice: 7
     Enter start node: 1
     Enter destination node: 3
     Shortest path from 1 to 3: 1 -> 2 -> 3
     Total distance: 8
     ```

5. **Print the Graph**:
   - Use option `8` to display the graph structure. For example:
     ```
     Enter your choice: 8
     Graph:
     1 -> (2, 5) (3, 8)
     2 -> (3, 3)
     ```

6. **Exit the Program**:
   - Use option `9` to terminate the program.

---

### Notes and Assumptions:

1. **Input Validation**:
   - The program assumes that all inputs are valid integers. Additional validation can be added to handle invalid inputs gracefully.

2. **Edge Weights**:
   - The program assumes that all edge weights are non-negative, as required by Dijkstra's algorithm. If negative weights are introduced, the algorithm may produce incorrect results.

3. **Directed Graph**:
   - The graph is directed, meaning edges have a specific direction from the source to the destination. If an undirected graph is desired, edges should be added in both directions.

4. **Extensibility**:
   - The program can be extended to include additional graph algorithms such as Bellman-Ford, Floyd-Warshall, or Kruskal's algorithm for minimum spanning trees.

---

This implementation provides a robust and flexible framework for working with weighted graphs. It supports essential graph operations, including node and edge management, traversals, and shortest path calculations. The menu-driven interface makes it easy to interact with the graph dynamically, while the modular design ensures that the code is maintainable and extensible. Whether used for educational purposes or as a foundation for more advanced graph applications, this program serves as a valuable tool for exploring graph theory concepts.
