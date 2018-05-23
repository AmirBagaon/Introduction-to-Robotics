#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <vector>
#include <fstream>
#include <stack>          //

using namespace std;

// Grid map definition
string location = "0,0"; //default location
double robot_size;
int rows;
int cols;
double mapResolution;
vector<vector<bool> > grid, gridD, coarseGrid4D;
int rowsD, colsD, rows4D ,cols4D, coarseStartX, coarseStartY;
double startX, startY;
bool requestMap(ros::NodeHandle &nh);
void readMap(const nav_msgs::OccupancyGrid& msg);
void printGridToFile();
void setLocationToParams();
void printCoverageStatToFile();
void DFS(int x, int y, vector<vector<bool> > &visited);
void notOccupied(int &x, int &y, vector<vector<bool> > &visited);
bool isSafe(int x, int y, vector<vector<bool> > &visited);
int availableCells = 0;

struct Node {
    int row;
    int col;
    bool isOccupied;
    Node* parent;
};
vector<vector<Node*> > spanTree;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coverage_node");
    ros::NodeHandle nh;
    if (nh.hasParam("starting_location")) {
        nh.getParam("starting_location", location);
    }
    else {
        cout << "\n\n no location!\n\n";
    }
    setLocationToParams();

    if (nh.hasParam("robot_size"))
    {
        nh.getParam("robot_size", robot_size);
    } else {
        cout << "\n\n nope!\n\n";
    }

    if (!requestMap(nh))
        exit(-1);
    printGridToFile();
    printCoverageStatToFile();
    return 0;
}
	
bool requestMap(ros::NodeHandle &nh) {
    nav_msgs::GetMap::Request req;
    nav_msgs::GetMap::Response res;

    while (!ros::service::waitForService("static_map", ros::Duration(3.0))) {
        ROS_INFO("Waiting for service static_map to become available");
    }

    ROS_INFO("Requesting the map...");
    ros::ServiceClient mapClient = nh.serviceClient<nav_msgs::GetMap>(
            "static_map");

    if (mapClient.call(req, res)) {
        readMap(res.map);
        return true;
    } else {
        ROS_ERROR("Failed to call map service");
        return false;
    }
}

void readMap(const nav_msgs::OccupancyGrid& map) {
    ROS_INFO("Received a %d X %d map @ %.3f m/px\n", map.info.width,
            map.info.height, map.info.resolution);

    rows = map.info.height;
    cols = map.info.width;
    mapResolution = map.info.resolution;
    int currCell = 0;

    double sizeD = (double)(robot_size / mapResolution);
    rowsD = rows / sizeD;
    rows4D = rowsD / 2;
    colsD = cols / sizeD;
    cols4D = colsD / 2;

    // Dynamically resize the gridD
    gridD.resize(rowsD);
    for (int i = 0; i < rowsD; i++) {
        gridD[i].resize(colsD);
    }

    //Make it all false (reachable cells), and after that we sign 1 when needed
    for (int i = 0; i < rowsD; i++) {
        for (int j = 0; j < colsD; j++) {
            gridD[i][j] = false;
        }
    }
    // Dynamically resize the coarse-grid (4D)
    coarseGrid4D.resize(rows4D);
    spanTree.resize(rows4D);
    for (int i = 0; i < rows4D; i++) {
        coarseGrid4D[i].resize(cols4D);
        spanTree[i].resize(cols4D);
    }
    //Make it all false (reachable cells), and after that we sign 1 when needed
    for (int i = 0; i < rows4D; i++) {
        for (int j = 0; j < cols4D; j++) {
            coarseGrid4D[i][j] = false;
            spanTree[i][j] = new Node();
            spanTree[i][j]->row = i;
            spanTree[i][j]->col = j;
            spanTree[i][j]->isOccupied = false;
            spanTree[i][j]->parent = NULL;
        }
    }

    // Dynamically resize the grid
    grid.resize(rows);
    for (int i = 0; i < rows; i++) {
        grid[i].resize(cols);
    }

    //Sign the occupied/unoccupied in the grid and and in the gridD
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (map.data[currCell] == 0) { // unoccupied cell
                grid[i][j] = false;
            } else{
                grid[i][j] = true; // occupied (100) or unknown cell (-1)
                gridD[i/sizeD][j/sizeD] = true;

             }
            currCell++;
        }
    }
    //Sign the occupied/unoccupied in the coarse grid, which each square contains 4 D's: 2 on 2
    for (int i = 0; i < rowsD; i++) {
        for (int j = 0; j < colsD; j++) {
            if (gridD[i][j] == true) { // unoccupied cell
                coarseGrid4D[i/2][j/2] = true;
                spanTree[i/2][j/2]->isOccupied = true;
            }
        }
    }
    //Find given location of robot according to the grids
    double x = (startX - map.info.origin.position.x) / mapResolution;
    double y = (startY - map.info.origin.position.y) / mapResolution;

    //Set to gridD
    x = x / sizeD;
    y = y / sizeD;

    //Set to coarseGrid
    coarseStartX = x / 2 - 1; //-1 because it will be index
    coarseStartY= y / 2 - 1;  //-1 because it will be index

    //Check if the index is out of the map
    if (coarseStartY < 0 || (coarseStartX < 0) || (coarseStartY >= rows4D) || (coarseStartX >= cols4D)) {
        ROS_INFO("Starting location is out of the map!");
        exit(-1);
    }
//    Node *first = new Node();
//    first->row = coarseStartX;
//    first->col = coarseStartY;
//    first->parent = NULL;

    vector<vector<bool> > visited = coarseGrid4D;
    //Check that the starting position is not on occupied cell
    notOccupied(coarseStartX, coarseStartY, visited);
    //Make the DFS
    DFS(coarseStartX, coarseStartY, visited);
    cout << "StartX: " << coarseStartX << " StartY: " << coarseStartY << endl;

    //Find the super-parent of each cell
//    Node* check = NULL;
//    for (int i = 0; i < rows4D; i++) {
//        for (int j = 0; j < cols4D; j++) {
//            if (spanTree[i][j]!= NULL) { // unoccupied cell
//                if (spanTree[i][j]->parent != NULL) { // unoccupied cell
//                    check = spanTree[i][j];
//                    cout << "r: " << check->row << " c: " << check->col;
//                    while (check->parent != NULL) {
//                        check = check->parent;
//                    }
//                    cout << "   Parent: r: " << check->row << "c: " << check->col << endl;
//                }
//            }
//        }
//    }

}



void printGridToFile() {
    ofstream gridFile;
    gridFile.open("new_grid.txt");
  
    for (int i = coarseGrid4D.size() - 1; i >= 0; i--) {
        for (int j = 0; j < coarseGrid4D[0].size(); j++) {
	    gridFile << (coarseGrid4D[i][j] ? "1" : "0");
        }
        gridFile << endl;
    }
    gridFile.close();
}
void printCoverageStatToFile() {
    ofstream gridFile;
    gridFile.open("coverage_stat.txt");
    gridFile << (availableCells);
    gridFile << endl;
    gridFile << coarseStartX << "," << coarseStartY;
    gridFile.close();
}


void setLocationToParams(){
    char sep;
    istringstream iss(location);
    iss >> startX;
    iss >> sep;
    iss >> startY;
}
//Coordinates for DFS (Right, Left, Up, Down)
int r[4] = { 0, 0, 1, -1 };
int c[4] = { 1, -1, 0, 0 };

//Check if we can go to next cell, or if its already visited/illegal
bool isSafe(int x, int y, vector<vector<bool> > &visited)
{
    //cout << "hi";
    if (x >= 0 && x < rows4D && y >= 0 &&
        y < cols4D && visited[x][y] == 0) {
        return true;
    }
    return false;
}
std::stack<Node*> nodeStack;
//The DFS function
void DFS(int x, int y, vector<vector<bool> > &visited)
{
    // sign it as visited
    visited[x][y] = 1;
    availableCells++;
    for (int k = 0; k < 4; k++) //4 - directions
        if (isSafe(x + r[k], y + c[k], visited)) {
            if(nodeStack.empty()) {
                spanTree[x][y]->parent = NULL;
            } else {
                spanTree[x][y]->parent = nodeStack.top();
            }
            nodeStack.push(spanTree[x][y]);
            DFS(x + r[k], y + c[k], visited);
            nodeStack.pop();
        }
}

//Check if the initial location is not occupied. otherwise, change it to a not-one.
void notOccupied(int &x, int &y, vector<vector<bool> > &visited){

    if (isSafe(x, y, visited)) {
        return;
    }

    int i = 0;
    bool flag = true;
    while (flag) {
        //Coordinates
        int rIndex[4] = {   0,      0, 1 + i, -1 - i };
        int cIndex[4] = {1 + i, -1 - i,     0,      0 };
        for (int k = 0; k < 4; k++) { //4 - directions
            if (isSafe(x + rIndex[k], y + cIndex[k], visited)) {
                x += rIndex[k];
                y += cIndex[k];
                flag = false;
                break;
            }
        }
        i++;
    }
}
