# Metro-Route-Planner
## Project Description

The Metro Route Planner is a C++ application designed to assist users in finding optimal routes between metro stations. It leverages multiple algorithms to provide the shortest, cheapest, and best paths based on different criteria like distance and cost. This tool can be particularly useful for commuters and tourists navigating a metro system.

## Features

- **Shortest Path**: Uses Dijkstra's algorithm to find the shortest path between two stations.
- **Cheapest Path**: Utilizes a BFS algorithm to find the path with the least cost.
- **Best Path**: Implements a DFS algorithm to find the best path based on a combination of criteria.

## Algorithms Used

1. **Dijkstra's Algorithm**: For finding the shortest path.
2. **Breadth-First Search (BFS)**: For finding the cheapest path.
3. **Depth-First Search (DFS)**: For finding the best path.

## Usage

### Example Graph
The project includes an example graph representing a hypothetical metro network with the following stations:

- Station A
- Station B
- Station C
- Station D
- Station E
- Station F
- Station G

### Running the Program
To run the program, compile the C++ code using a compatible compiler and execute the resulting binary. 

bash
g++ -o metro_route_planner metro_route_planner.cpp
./metro_route_planner

The program will output the shortest, cheapest, and best paths between the specified source and destination stations.

### Example Output

Shortest Path (Distance: 20, Cost: 80): Station E -> Station C -> Station B
Cheapest Path (Distance: 20, Cost: 60): Station E -> Station C -> Station B
Best Path (Distance: 20, Cost: 60): Station E -> Station C -> Station B


## Code Structure

- **Station**: Represents a metro station with attributes like id, name, and line number.
- **Edge**: Represents an edge between two stations with attributes like source, destination, distance, and cost.
- **dijkstra()**: Function to find the shortest path using Dijkstra's algorithm.
- **bfs()**: Function to find the cheapest path using BFS.
- **dfs()**: Function to find the best path using DFS.

## Future Enhancements

- Add support for real-time data integration.
- Expand the attributes of stations and edges to include factors like travel time and transportation mode.
- Implement a graphical user interface for better user interaction.

## Contributions

Contributions are welcome! Please fork the repository and submit a pull request with your 
