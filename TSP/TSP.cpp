#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>

using namespace std;

// A structure to represent a state in the search
struct State {
    vector<int> path;    // The path taken so far
    double cost;         // The cost of the current path
    double maxDist;      // The maximum distance between two consecutive cities in the path
    int lastCity;        // The last city visited

    // Comparator for priority queue based on cost for Uniform Cost Search
    bool operator<(const State& other) const {
        return cost > other.cost;
    }
};

// Function to find the maximum distance of a path
double findMaxDistance(const vector<int>& path, const vector<vector<double>>& distanceMatrix) {
    double maxDist = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        maxDist = max(maxDist, distanceMatrix[path[i - 1]][path[i]]);
    }
    return maxDist;
}

// Breadth-First Search strategy
vector<int> BFS(const vector<vector<double>>& distanceMatrix) {
    queue<State> frontier;
    State initialState = { {0}, 0.0, 0.0, 0 };
    frontier.push(initialState);

    vector<int> bestPath;
    double bestMaxDist = numeric_limits<double>::infinity();

    while (!frontier.empty()) {
        State current = frontier.front();
        frontier.pop();

        // If the path includes all cities and returns to the origin, check if it's a better solution
        if (current.path.size() == distanceMatrix.size()) {
            current.path.push_back(0);
            double dist = distanceMatrix[current.lastCity][0];
            current.cost += dist;
            current.maxDist = max(current.maxDist, dist);

            if (current.maxDist < bestMaxDist) {
                bestMaxDist = current.maxDist;
                bestPath = current.path;
            }
            continue;
        }

        // Expand the current state by visiting all unvisited cities
        for (size_t i = 0; i < distanceMatrix.size(); ++i) {
            if (find(current.path.begin(), current.path.end(), i) == current.path.end()) {
                State nextState = current;
                nextState.path.push_back(i);
                double dist = distanceMatrix[current.lastCity][i];
                nextState.cost += dist;
                nextState.maxDist = max(nextState.maxDist, dist);
                nextState.lastCity = i;
                frontier.push(nextState);
            }
        }
    }
    return bestPath;
}

// Least-Cost (Uniform Cost) Search strategy
vector<int> uniformCostSearch(const vector<vector<double>>& distanceMatrix) {
    priority_queue<State> frontier;
    State initialState = { {0}, 0.0, 0.0, 0 };
    frontier.push(initialState);

    vector<int> bestPath;
    double bestMaxDist = numeric_limits<double>::infinity();

    while (!frontier.empty()) {
        State current = frontier.top();
        frontier.pop();

        // If the path includes all cities and returns to the origin, check if it's a better solution
        if (current.path.size() == distanceMatrix.size()) {
            current.path.push_back(0);
            double dist = distanceMatrix[current.lastCity][0];
            current.cost += dist;
            current.maxDist = max(current.maxDist, dist);

            if (current.maxDist < bestMaxDist) {
                bestMaxDist = current.maxDist;
                bestPath = current.path;
            }
            continue;
        }

        // Expand the current state by visiting all unvisited cities
        for (size_t i = 0; i < distanceMatrix.size(); ++i) {
            if (find(current.path.begin(), current.path.end(), i) == current.path.end()) {
                State nextState = current;
                nextState.path.push_back(i);
                double dist = distanceMatrix[current.lastCity][i];
                nextState.cost += dist;
                nextState.maxDist = max(nextState.maxDist, dist);
                nextState.lastCity = i;
                frontier.push(nextState);
            }
        }
    }
    return bestPath;
}

// A* Search strategy
vector<int> AStarSearch(const vector<vector<double>>& distanceMatrix) {
    auto heuristic = [](int currentCity, int startCity, const vector<vector<double>>& distanceMatrix) {
        return distanceMatrix[currentCity][startCity];
        };

    priority_queue<State> frontier;
    State initialState = { {0}, 0.0, 0.0, 0 };
    frontier.push(initialState);

    vector<int> bestPath;
    double bestMaxDist = numeric_limits<double>::infinity();

    while (!frontier.empty()) {
        State current = frontier.top();
        frontier.pop();

        // If the path includes all cities and returns to the origin, check if it's a better solution
        if (current.path.size() == distanceMatrix.size()) {
            current.path.push_back(0);
            double dist = distanceMatrix[current.lastCity][0];
            current.cost += dist;
            current.maxDist = max(current.maxDist, dist);

            if (current.maxDist < bestMaxDist) {
                bestMaxDist = current.maxDist;
                bestPath = current.path;
            }
            continue;
        }

        // Expand the current state by visiting all unvisited cities
        for (size_t i = 0; i < distanceMatrix.size(); ++i) {
            if (find(current.path.begin(), current.path.end(), i) == current.path.end()) {
                State nextState = current;
                nextState.path.push_back(i);
                double dist = distanceMatrix[current.lastCity][i];
                nextState.cost += dist;
                nextState.maxDist = max(nextState.maxDist, dist);
                nextState.lastCity = i;
                nextState.cost += heuristic(i, 0, distanceMatrix);
                frontier.push(nextState);
            }
        }
    }
    return bestPath;
}

int main() {
    // Example distance matrix
    vector<vector<double>> distanceMatrix = {
        {0.0, 19.0, 10.0, 33.0},
        {15.0, 0.0, 34.0, 25.0},
        {16.0, 43.0, 0.0, 37.0},
        {21.0, 28.0, 30.0, 0.0}
    };

    vector<int> bfsPath = BFS(distanceMatrix);
    vector<int> ucsPath = uniformCostSearch(distanceMatrix);
    vector<int> astarPath = AStarSearch(distanceMatrix);

    auto printPath = [](const vector<int>& path) {
        for (int city : path) {
            cout << city << " ";
        }
        cout << endl;
        };

    auto calculateTotalCost = [](const vector<int>& path, const vector<vector<double>>& distanceMatrix) {
        double totalCost = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            totalCost += distanceMatrix[path[i - 1]][path[i]];
        }
        return totalCost;
        };

    cout << "BFS Path: ";
    printPath(bfsPath);
    cout << "Total Cost: " << calculateTotalCost(bfsPath, distanceMatrix) << endl;

    cout << "Uniform Cost Search Path: ";
    printPath(ucsPath);
    cout << "Total Cost: " << calculateTotalCost(ucsPath, distanceMatrix) << endl;

    cout << "A* Search Path: ";
    printPath(astarPath);
    cout << "Total Cost: " << calculateTotalCost(astarPath, distanceMatrix) << endl;

    return 0;
}