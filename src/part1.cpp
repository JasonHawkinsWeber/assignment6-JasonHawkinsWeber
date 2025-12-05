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
  
Node operator+(Node a, Node b) {
    return Node(a.getXCoord() + b.getXCoord(), a.getYCoord() + b.getYCoord());
}

Node operator-(Node a, Node b) {
    return Node(a.getXCoord() - b.getXCoord(), a.getYCoord() - b.getYCoord());
}

Node operator * (float s, Node a) {
    return Node(s * a.getXCoord(), s * a.getYCoord());
}

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

std::vector<Node> getBezierPoint( const std::vector<Node> & path, float scaler ) {
    std::vector<Node> tmp = path;

    int i = tmp.size() - 1;
    while (i > 0) {
        for (int k = 0; k < i; k++)
            tmp[k] = tmp[k] + scaler * ( tmp[k+1] - tmp[k] );
        i--;
    }

    return tmp;
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
      else if(grid[i][j] == 1)
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


int main(int argc, char *argv[])
{

  // // 1--> Node is blocked
  // // 0--> Node isn't blocked
  // // Grid must be square 
  // std::vector<std::vector<int>> grid = {{0, 0, 0, 0, 1, 0, 0, 0},
  //                                       {0, 0, 0, 0, 1, 0, 1, 0},
  //                                       {0, 0, 0, 0, 1, 0, 1, 0},
  //                                       {0, 0, 0, 0, 0, 0, 1, 0},
  //                                       {0, 0, 0, 0, 1, 0, 1, 0},
  //                                       {0, 0, 0, 0, 1, 0, 1, 0},
  //                                       {0, 0, 0, 0, 1, 0, 1, 0},
  //                                       {0, 0, 0, 0, 1, 0, 1, 0}};

  // Create the matrix from the file
  std::string fileName = std::string(argv[1]);
  int n = std::stoi(argv[2]);
  auto grid = createGridFromFile(fileName, n);

  Node start(0, 0);
  Node goal(63, 63);

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
  auto smothPath = getBezierPoint(path, 0.05f);

  if (!smothPath.empty())
  {
    std::cout << "Path found:\n";
    PrintPath(grid, smothPath);
  }
  else
  {
    std::cout << "No path found.\n";
  }

  for(auto node : smothPath) {
    std::cout << "(" << node.getXCoord() << ", " << node.getYCoord() << ") ";
  } 


  return 0;
}
