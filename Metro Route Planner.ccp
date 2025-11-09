#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <limits>
#include <algorithm>
using namespace std;

struct Station {
    int id;
    string name;
    int line;
};

struct Edge {
    int source;
    int destination;
    int distance;
    int cost;
};

//  Dijkstra for shortest path (focus on distance)
vector<int> dijkstra(const vector<vector<Edge>>& graph, int source, int destination, vector<Station>& stations, int& shortestDistance, int& shortestCost) {
    int numStations = graph.size();
    vector<int> distance(numStations, numeric_limits<int>::max());
    vector<int> cost(numStations, numeric_limits<int>::max());
    vector<int> parent(numStations, -1);
    vector<bool> visited(numStations, false);

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    distance[source] = 0;
    cost[source] = 0;
    pq.push({0, source});

    while (!pq.empty()) {
        int current = pq.top().second;
        pq.pop();

        if (visited[current]) continue;
        visited[current] = true;

        for (const Edge& edge : graph[current]) {
            int neighbor = edge.destination;
            int newDistance = distance[current] + edge.distance;
            int newCost = cost[current] + edge.cost;

            if (newDistance < distance[neighbor]) {
                distance[neighbor] = newDistance;
                cost[neighbor] = newCost;
                parent[neighbor] = current;
                pq.push({distance[neighbor], neighbor});
            }
        }
    }

    vector<int> path;
    int current = destination;
    shortestDistance = distance[destination];
    shortestCost = cost[destination];
    while (current != -1) {
        path.push_back(current);
        current = parent[current];
    }
    reverse(path.begin(), path.end());
    return path;
}

//  Dijkstra for cheapest path (focus on cost)
vector<int> dijkstraCheapest(const vector<vector<Edge>>& graph, int source, int destination, vector<Station>& stations, int& cheapestDistance, int& cheapestCost) {
    int numStations = graph.size();
    vector<int> cost(numStations, numeric_limits<int>::max());
    vector<int> distance(numStations, numeric_limits<int>::max());
    vector<int> parent(numStations, -1);
    vector<bool> visited(numStations, false);

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    cost[source] = 0;
    distance[source] = 0;
    pq.push({0, source});

    while (!pq.empty()) {
        int current = pq.top().second;
        pq.pop();

        if (visited[current]) continue;
        visited[current] = true;

        for (const Edge& edge : graph[current]) {
            int neighbor = edge.destination;
            int newCost = cost[current] + edge.cost;
            int newDistance = distance[current] + edge.distance;

            if (newCost < cost[neighbor]) {
                cost[neighbor] = newCost;
                distance[neighbor] = newDistance;
                parent[neighbor] = current;
                pq.push({cost[neighbor], neighbor});
            }
        }
    }

    vector<int> path;
    int current = destination;
    cheapestDistance = distance[destination];
    cheapestCost = cost[destination];
    while (current != -1) {
        path.push_back(current);
        current = parent[current];
    }
    reverse(path.begin(), path.end());
    return path;
}

// DFS Utility for minimum hop path
void dfsUtil(const vector<vector<Edge>>& graph, int current, int destination, vector<bool>& visited, vector<int>& path, vector<int>& bestPath, int& bestDistance, int& bestCost) {
    visited[current] = true;
    path.push_back(current);

    if (current == destination && (bestPath.empty() || path.size() < bestPath.size())) {
        bestPath = path;
        bestDistance = 0;
        bestCost = 0;
        for (int i = 1; i < path.size(); ++i) {
            int src = path[i - 1];
            int dst = path[i];
            for (const Edge& edge : graph[src]) {
                if (edge.destination == dst) {
                    bestDistance += edge.distance;
                    bestCost += edge.cost;
                    break;
                }
            }
        }
    }

    for (const Edge& edge : graph[current]) {
        int neighbor = edge.destination;
        if (!visited[neighbor]) {
            dfsUtil(graph, neighbor, destination, visited, path, bestPath, bestDistance, bestCost);
        }
    }

    visited[current] = false;
    path.pop_back();
}

vector<int> dfs(const vector<vector<Edge>>& graph, int source, int destination, vector<Station>& stations, int& bestDistance, int& bestCost) {
    int numStations = graph.size();
    vector<bool> visited(numStations, false);
    vector<int> path, bestPath;
    dfsUtil(graph, source, destination, visited, path, bestPath, bestDistance, bestCost);
    return bestPath;
}

int main() {
    int numStations = 7;
    vector<vector<Edge>> graph(numStations);
    vector<Station> stations = {
        {0, "Station A", 1},
        {1, "Station B", 1},
        {2, "Station C", 2},
        {3, "Station D", 2},
        {4, "Station E", 3},
        {5, "Station F", 3},
        {6, "Station G", 1}
    };

    graph[0].push_back({0, 1, 10, 50});
    graph[0].push_back({0, 2, 20, 350});
    graph[1].push_back({1, 2, 5, 110});
    graph[1].push_back({1, 3, 150, 400});
    graph[2].push_back({2, 1, 5, 20});
    graph[2].push_back({2, 3, 10, 205});
    graph[2].push_back({2, 4, 20, 50});
    graph[3].push_back({3, 2, 10, 110});
    graph[3].push_back({3, 5, 20, 30});
    graph[4].push_back({4, 2, 20, 30});
    graph[4].push_back({4, 5, 10, 20});
    graph[4].push_back({4, 6, 5, 10});
    graph[5].push_back({5, 3, 20, 10});
    graph[5].push_back({5, 4, 10, 10});
    graph[5].push_back({5, 6, 15, 20});

    int source = 1, destination = 5; //source Station Id and destination Station Id

    int shortestDistance, shortestCost;
    vector<int> shortestPath = dijkstra(graph, source, destination, stations, shortestDistance, shortestCost);
    cout << "Shortest Path (Distance: " << shortestDistance << ", Cost: " << shortestCost << "): ";
    for (int i = 0; i < shortestPath.size(); ++i) {
        cout << stations[shortestPath[i]].name;
        if (i != shortestPath.size() - 1) cout << " -> ";
    }
    cout << endl;

    int cheapestDistance, cheapestCost;
    vector<int> cheapestPath = dijkstraCheapest(graph, source, destination, stations, cheapestDistance, cheapestCost);
    cout << "Cheapest Path (Cost: " << cheapestCost << ", Distance: " << cheapestDistance << "): ";
    for (int i = 0; i < cheapestPath.size(); ++i) {
        cout << stations[cheapestPath[i]].name;
        if (i != cheapestPath.size() - 1) cout << " -> ";
    }
    cout << endl;

    int bestDistance, bestCost;
    vector<int> bestPath = dfs(graph, source, destination, stations, bestDistance, bestCost);
    cout << "Best Path (Distance: " << bestDistance << ", Cost: " << bestCost << "): ";
    for (int i = 0; i < bestPath.size(); ++i) {
        cout << stations[bestPath[i]].name;
        if (i != bestPath.size() - 1) cout << " -> ";
    }
    cout << endl;

    return 0;
}
