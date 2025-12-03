#include <iostream>
#include <fstream>
#include "alg_graphs.h"
#include <vector>
#include <algorithm>
#include <climits>
#include <cmath>
#include <queue>

using namespace std;

// Possible movement directions: up, up right, right, down right, down, down left, left and up left
//                  up
//          -1,-1  -1,0   -1,1
//               \   |   /
//                \  |  /
//     left 0,-1----NODE----0,1 right
//                /  |  \
//               /   |   \
//           1,-1   1,0   1,1
//                  down
const std::vector<std::pair<int, int>> directions = {{-1, 0}, {-1, 1}, {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}};

// Define a structure to represent a node in the graph
struct Node
{
  // Coordinates of the node in the graph
  int x = 0;
  int y = 0;

  // Cost values used by the A* algorithm
  int totalCostF = 0;
  int actualCostG = 0;
  int heuristicCostH = 0;

  Node() = default;

  Node(int xCoord, int yCoord)
      : x(xCoord), y(yCoord)
  {
  }

  bool operator>(const Node &other) const
  {
    return totalCostF > other.totalCostF;
  }

  bool operator==(const Node &other) const
  {
    return x == other.x && y == other.y;
  }
};

// -------------------------
// A* Pathfinding Algorithm
// -------------------------
std::vector<Node> FindPath(const std::vector<std::vector<int>> &graph, const Node &start, const Node &goal)
{
  int rows_cols = graph.size();

  // Priority queue (min-heap) sorted by lowest f cost
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList;

  // Closed list: marks visited nodes
  std::vector<std::vector<bool>> closedList(rows_cols, std::vector<bool>(rows_cols, false));

  // gScore: best-known cost to reach each cell
  std::vector<std::vector<int>> gScore(rows_cols, std::vector<int>(rows_cols, INT_MAX));

  // Parent matrix: stores the node from which we arrived at each position
  std::vector<std::vector<Node>> parent(rows_cols, std::vector<Node>(rows_cols));

  // Initialize start node cost
  gScore[start.x][start.y] = 0;
  openList.push(start);

  // -------------------------
  // Main A* search loop
  // -------------------------
  while (!openList.empty())
  {
    // Get node with lowest f score
    Node current = openList.top();
    openList.pop();

    // Check if we've reached the goal
    if (current == goal)
    {
      // Reconstruct the path by backtracking through parents
      std::vector<Node> path;
      while (!(current == start))
      {
        path.push_back(current);
        current = parent[current.x][current.y];
      }
      path.push_back(start);

      // Reverse to get path from start → goal
      std::reverse(path.begin(), path.end());
      return path;
    }

    // Mark current node as closed (visited)
    closedList[current.x][current.y] = true;

    // Explore all 8 neighboring cells
    for (int i = 0; i < directions.size(); ++i)
    {
      int newX = current.x + directions[i].first;
      int newY = current.y + directions[i].second;

      // Check grid boundaries and walkability
      if (newX >= 0 && newX < rows_cols && newY >= 0 && newY < rows_cols && graph[newX][newY] == 0)
      {
        // Skip already processed (closed) cells
        if (closedList[newX][newY])
          continue;

        // Tentative g cost (current cost + 1 for movement)
        int newG = gScore[current.x][current.y] + 1;

        // If we found a better path to this neighbor
        if (newG < gScore[newX][newY])
        {
          gScore[newX][newY] = newG;

          // Compute new neighbor costs
          Node neighbor(newX, newY);
          neighbor.actualCostG = newG;
          neighbor.heuristicCostH = std::abs(newX - goal.x) + std::abs(newY - goal.y); // Manhattan distance
          neighbor.totalCostF = neighbor.actualCostG + neighbor.heuristicCostH;

          // Record parent (for path reconstruction)
          parent[newX][newY] = current;

          // Add neighbor to open list for further exploration
          openList.push(neighbor);
        }
      }
    }
  }

  // No path found
  return {};
}

//Prints the path on the grid with 'X' for path and '-' for empty space
void PrintPath(const std::vector<std::vector<int>> &grid, const std::vector<Node> &path)
{

  for (int i = 0; i < grid.size(); i++)
  {
    for (int j = 0; j < grid[0].size(); j++)
    {
      bool isPath = false;
      for (const Node &node : path)
      {
        if (node.x == i && node.y == j)
        {
          isPath = true;
          break;
        }
      }
      if (isPath)
        std::cout << "X";
      else
        std::cout << "-";
    }
    std::cout << std::endl;
  }

  std::cout << std::endl;
}

int main()
{

  // 1--> Node is blocked
  // 0--> Node isn't blocked
  // Grid must be square 
  std::vector<std::vector<int>> grid = {{0, 0, 0, 0, 1, 0, 0, 0},
                                        {0, 0, 0, 0, 1, 0, 1, 0},
                                        {0, 0, 0, 0, 1, 0, 1, 0},
                                        {0, 0, 0, 0, 0, 0, 1, 0},
                                        {0, 0, 0, 0, 1, 0, 1, 0},
                                        {0, 0, 0, 0, 1, 0, 1, 0},
                                        {0, 0, 0, 0, 1, 0, 1, 0},
                                        {0, 0, 0, 0, 1, 0, 1, 0}};

  Node start(0, 0);
  Node goal(7, 7);

  auto path = FindPath(grid, start, goal);
  if (!path.empty())
  {
    std::cout << "Path found:\n";
    PrintPath(grid, path);
  }
  else
  {
    std::cout << "No path found.\n";
  }

  return 0;
}
