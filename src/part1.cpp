#include <iostream>
#include <fstream>
#include "alg_graphs.h"
#include <vector>
#include <algorithm>
#include <climits>
#include <cmath>
#include <queue>
#include <sstream>

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

// Function that takes a filepath and creates a grid
std::vector<std::vector<int>> createGridFromFile(std::string filePath, int gridSize)
{
  // Try and read in the file name of the activities
  std::ifstream fileStream;
  fileStream.open(filePath);
  if (!fileStream.is_open())
  {
    std::cout << "Failed to open: " << filePath << std::endl;
    return {};
  }

  std::string line;

  std::vector<std::vector<int>> grid;
  while (std::getline(fileStream, line))
  {
    std::istringstream ss(line);
    std::vector<int> gridLine;
    gridLine.resize(gridSize);
    for (int i = 0; i < gridSize; i++)
    {
      ss >> gridLine[i];
    }
    grid.push_back(gridLine);
  }
  fileStream.close();

  return grid;
}

// Define a structure to represent a node in the graph
struct Node
{
public:
  int getXIndex() const { return x; }
  int getYIndex() const { return y; }

  double getXCoord() const { return x; }
  double getYCoord() const { return y; }

  // Cost values used by the A* algorithm
  int totalCostF = 0;
  int actualCostG = 0;
  int heuristicCostH = 0;

  Node() = default;

  Node(double xCoord, double yCoord)
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

private:
  // Coordinates of the node in the graph
  double x = 0.;
  double y = 0.;
};

Node operator+(Node a, Node b)
{
  return Node(a.getXCoord() + b.getXCoord(), a.getYCoord() + b.getYCoord());
}

Node operator-(Node a, Node b)
{
  return Node(a.getXCoord() - b.getXCoord(), a.getYCoord() - b.getYCoord());
}

Node operator*(float s, Node a)
{
  return Node(s * a.getXCoord(), s * a.getYCoord());
}

// -------------------------
// A* Pathfinding Algorithm
// -------------------------
std::vector<Node> aStarPathing(const std::vector<std::vector<int>> &graph, const Node &start, const Node &goal)
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
  gScore[start.getXIndex()][start.getYIndex()] = 0;
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
        current = parent[current.getXIndex()][current.getYIndex()];
      }
      path.push_back(start);

      // Reverse to get path from start → goal
      std::reverse(path.begin(), path.end());
      return path;
    }

    // Mark current node as closed (visited)
    closedList[current.getXIndex()][current.getYIndex()] = true;

    // Explore all 8 neighboring cells
    for (int i = 0; i < directions.size(); ++i)
    {
      int newX = current.getXIndex() + directions[i].first;
      int newY = current.getYIndex() + directions[i].second;

      // Check grid boundaries and walkability
      if (newX >= 0 && newX < rows_cols && newY >= 0 && newY < rows_cols && graph[newX][newY] == 0)
      {
        // Skip already processed (closed) cells
        if (closedList[newX][newY])
          continue;

        // Tentative g cost (current cost + 1 for movement)
        int newG = gScore[current.getXIndex()][current.getYIndex()] + 1;

        // If we found a better path to this neighbor
        if (newG < gScore[newX][newY])
        {
          gScore[newX][newY] = newG;

          // Compute new neighbor costs
          Node neighbor(newX, newY);
          neighbor.actualCostG = newG;
          neighbor.heuristicCostH = std::abs(newX - goal.getXIndex()) + std::abs(newY - goal.getYIndex()); // Manhattan distance
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

// Function to get Bezier smoothed points from a path
std::vector<Node> getBezierPoint(const std::vector<Node> &path, float scaler)
{
  std::vector<Node> smoothPath = path;

  //apply de Casteljau's algorithm
  int i = smoothPath.size() - 1;
  while (i > 0)
  {
    for (int k = 0; k < i; k++)
      smoothPath[k] = smoothPath[k] + scaler * (smoothPath[k + 1] - smoothPath[k]);
    i--;
  }
  
  //if the first point is not the same as the start point, add it
  if(smoothPath[0].getXCoord() != path[0].getXCoord() && smoothPath[0].getYCoord() != path[0].getYCoord())
  {
    smoothPath.insert(smoothPath.begin(), path[0]);
  }

  return smoothPath;
}

// Function to check line of sight between two nodes
// uses Bresenham's Line Algorithm 
bool lineOfSight(Node node1, Node node2, const std::vector<std::vector<int>> &graph)
{
  int x0 = node1.getXIndex();
  int y0 = node1.getYIndex();
  int x1 = node2.getXIndex();
  int y1 = node2.getYIndex();
  int dx = abs(x1 - x0);
  int dy = -abs(y1 - y0);

  int sX = -1;
  int sY = -1;
  if (x0 < x1)
  {
    sX = 1;
  }
  if (y0 < y1)
  {
    sY = 1;
  }

  int e = dx + dy;
  while (true)
  {
    if (graph[x0][y0] == 1)
    {
      return false;
    }
    if (x0 == x1 && y0 == y1)
    {
      return true;
    }

    int e2 = 2 * e;
    if (e2 >= dy)
    {
      if (x0 == x1)
      {
        return true;
      }
      e += dy;
      x0 += sX;
    }
    if (e2 <= dx)
    {
      if (y0 == y1)
      {
        return true;
      }
      e += dx;
      y0 += sY;
    }
  }

  return false;
}

// Theta* Pathfinding Algorithm
std::vector<Node> thetaStarPathing(const std::vector<std::vector<int>> &graph, const Node &start, const Node &goal)
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
  gScore[start.getXIndex()][start.getYIndex()] = 0;
  openList.push(start);

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
        current = parent[current.getXIndex()][current.getYIndex()];
      }
      path.push_back(start);

      // Reverse to get path from start → goal
      std::reverse(path.begin(), path.end());
      return path;
    }
    // Mark current node as closed (visited)
    closedList[current.getXIndex()][current.getYIndex()] = true;

    // Explore all 8 neighboring cells
    for (int i = 0; i < directions.size(); ++i)
    {
      int newX = current.getXIndex() + directions[i].first;
      int newY = current.getYIndex() + directions[i].second;

      // Check grid boundaries and walkability
      if (newX >= 0 && newX < rows_cols && newY >= 0 && newY < rows_cols && graph[newX][newY] == 0)
      {
        // Skip already processed (closed) cells
        if (closedList[newX][newY])
          continue;

        // If we found a better path to this neighbor

        int newG = gScore[current.getXIndex()][current.getYIndex()] + 1;

        // Compute new neighbor costs
        Node neighbor(newX, newY);
        Node &parentNode = parent[current.getXIndex()][current.getYIndex()];

        // This part of the algorithm is the main difference between A* and Theta*
        if (lineOfSight(parentNode, neighbor, graph))
        {

          double heuristicCost = std::abs(newX - parentNode.getXIndex()) + std::abs(newY - parentNode.getYIndex()); // Manhattan distance

          // If there is line-of-sight between parent(s) and neighbor
          // then ignore s and use the path from parent(s) to neighbor
          if (parentNode.actualCostG + heuristicCost < gScore[newX][newY])
          {

            gScore[newX][newY] = parentNode.actualCostG + heuristicCost;
            parent[newX][newY] = parentNode;

            neighbor.actualCostG = parentNode.actualCostG + heuristicCost;
            neighbor.heuristicCostH = heuristicCost;
            neighbor.totalCostF = neighbor.actualCostG + neighbor.heuristicCostH;

            openList.push(neighbor);
          }
        }
        else
        {

          //If there is no line-of-sight, proceed as normal A*
          // Tentative g cost (current cost + 1 for movement)
          int newG = gScore[current.getXIndex()][current.getYIndex()] + 1;
          if (newG < gScore[newX][newY])
          {
            gScore[newX][newY] = newG;

            // Compute new neighbor costs
            Node neighbor(newX, newY);
            neighbor.actualCostG = newG;
            neighbor.heuristicCostH = std::abs(newX - goal.getXIndex()) + std::abs(newY - goal.getYIndex()); // Manhattan distance
            neighbor.totalCostF = neighbor.actualCostG + neighbor.heuristicCostH;

            // Record parent (for path reconstruction)
            parent[newX][newY] = current;

            // Add neighbor to open list for further exploration
            openList.push(neighbor);
          }
        }
      }
    }
  }

  return {};
}

// Prints the path on the grid with 'X' for path and '-' for empty space and '|' for blocked nodes
void PrintPath(const std::vector<std::vector<int>> &grid, const std::vector<Node> &path)
{

  for (int i = 0; i < grid.size(); i++)
  {
    for (int j = 0; j < grid[0].size(); j++)
    {
      bool isPath = false;
      for (const Node &node : path)
      {
        if (node.getXIndex() == i && node.getYIndex() == j)
        {
          isPath = true;
          break;
        }
      }
      if (isPath)
      {

        std::cout << "X";
      }
      else if (grid[i][j] == 1)
      {
        std::cout << "|";
      }
      else
      {
        std::cout << "-";
      }
    }
    std::cout << std::endl;
  }

  std::cout << std::endl;
}

// Function to calculate total distance of a path
double totalDistance(const std::vector<Node> &path)
{
  double totalDistance = 0.;
  for(int i=0; i<path.size()-1; i++)
  {
    totalDistance += sqrt(pow(path[i].getXCoord() - path[i + 1].getXCoord(), 2) + pow(path[i].getYCoord() - path[i+ 1].getYCoord(), 2));
  }
  return totalDistance; 
}

// Main function
// Takes in command line arguments for file name and grid size
int main(int argc, char *argv[])
{

  // 1--> Node is blocked
  // 0--> Node isn't blocked
  // Grid must be square
  // std::vector<std::vector<int>> grid = {{0, 0, 0, 0, 1, 0, 0, 0},
  //                                       {0, 0, 0, 0, 1, 0, 1, 0},
  //                                       {0, 0, 0, 0, 1, 0, 1, 0},
  //                                       {0, 0, 0, 0, 0, 0, 1, 0},
  //                                       {0, 0, 0, 0, 1, 0, 1, 0},
  //                                       {0, 0, 0, 0, 1, 0, 1, 0},
  //                                       {0, 0, 0, 0, 1, 0, 1, 0},
  //                                       {0, 0, 0, 0, 1, 0, 1, 0}};
  // Node goal(7, 7);


  // Create the matrix from the file
  std::string fileName = std::string(argv[1]);
  int n = std::stoi(argv[2]);
  std::vector<std::vector<int>> grid = createGridFromFile(fileName, n);

  Node start(0, 0);
  Node goal(63, 63);

  auto path = aStarPathing(grid, start, goal);
  if (!path.empty())
  {
    std::cout << "Path found:\n";
    std::cout << "Total Distance: " << totalDistance(path) << std::endl;
    PrintPath(grid, path);
  }
  else
  {
    std::cout << "No path found.\n";
  }
  
  std::vector<Node> smoothPath;

  bool pathValid = false;
  float scale = 0.05f;
  smoothPath = getBezierPoint(path, scale);

    

  if (!smoothPath.empty())
  {
    std::cout << "Path found:\n";
    std::cout << "Total Distance: " << totalDistance(smoothPath) << std::endl;
    PrintPath(grid, smoothPath);
  }
  else
  {
    std::cout << "No path found.\n";
  }

  auto thetaPath = thetaStarPathing(grid, start, goal);

  if (!thetaPath.empty())
  {
    std::cout << "Path found:\n";
    std::cout << "Total Distance: " << totalDistance(thetaPath) << std::endl;
    PrintPath(grid, thetaPath);
}
  else
  {
    std::cout << "No path found.\n";
  }

  return 0;
}
