#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <vector>
#include <fstream>
#include <stack>          //
#include <tf/exceptions.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include "../../wander_bot/src/wander_bot.h"


using namespace std;

// Grid map definition
string location = "0,0"; //default location
vector<vector<bool> > grid, gridD, coarseGrid4D;
int rows, cols,  rowsD, colsD, rows4D ,cols4D, coarseStartX, coarseStartY;
double robot_size, sizeD, mapResolution, startX, startY, mapOriginX, mapOriginY;
bool requestMap(ros::NodeHandle &nh);
void readMap(const nav_msgs::OccupancyGrid& msg);
void printCoverageStatToFile();
void DFS(int x, int y, vector<vector<bool> > &visited);
void notOccupied(int &x, int &y, vector<vector<bool> > &visited);
bool isSafe(int x, int y, vector<vector<bool> > &visited);
void startDriving();
void calculateMove(double& x, double& y);

int availableCells = 0;
int nodeIdentity = 0;
int cellsCovered = 0;

ros::Time beginTime;
ros::Time finalTime;

struct Node {
    int row;
    int col;
    int identity;
    bool isOccupied;
    Node* parent;
    vector<Node*> childs;
};
vector<vector<Node*> > spanTree;
bool alreadyHasChild(Node* n, int ident);


struct Station {
    double x;
    double y;
};
vector<Station*> stationList;
void CreateDriveStations(Node *n);
void makeItMove(double row, double col);

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coverage_4_node");
    ros::NodeHandle nh;
    tf::TransformListener listener;
    ros::Rate rate(2.0);

    //listener.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(10.0) );
    listener.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(10.0) );

    tf::StampedTransform transform;
    try {
        listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
        startX = transform.getOrigin().x();
        startY = transform.getOrigin().y();
        cout << "Current position: (" << startX  << "," << startY << ")" << endl;
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
    }
    rate.sleep();


    if (nh.hasParam("robot_size"))
    {
        nh.getParam("robot_size", robot_size);
    } else {
        cout << "\n\n nope!\n\n";
    }

    if (!requestMap(nh))
        exit(-1);

    //Calculate the stations that the robot should pass in the hemilton cycle
    CreateDriveStations(spanTree[coarseStartX][coarseStartY]);
//    for(std::vector<Station*>::iterator it = stationList.begin(); it != stationList.end(); ++it) {
//        cout <<"Station: " << (*it)->x << "," << (*it)->y << endl;
//    }

    printCoverageStatToFile();

    //make the robot move to the stations
    startDriving();
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
    mapOriginX = map.info.origin.position.x;
    mapOriginY = map.info.origin.position.y;


    int currCell = 0;

    sizeD = (double)(robot_size / mapResolution);
    cout << "SizeD: "<< sizeD << endl;

    rowsD = rows / sizeD;
    rows4D = rowsD / 2;
    colsD = cols / sizeD;
    cols4D = colsD / 2;
    cout << "cols4D: "<< cols4D << endl;
    cout << "rows4D: "<< rows4D << endl;

    // Dynamically resize the gridD
    //and make it all false (reachable cells), and after that we sign 1 when needed
    gridD.resize(rowsD);
    for (int i = 0; i < rowsD; i++) {
        gridD[i].resize(colsD);
        for (int j = 0; j < colsD; j++) {
            gridD[i][j] = false;
        }
    }
    // Dynamically resize the coarse-grid (4D)
    coarseGrid4D.resize(rows4D);
    spanTree.resize(rows4D);
    //And make it all false (reachable cells), and after that we sign 1 when needed
    for (int i = 0; i < rows4D; i++) {
        coarseGrid4D[i].resize(cols4D);
        spanTree[i].resize(cols4D);
        for (int j = 0; j < cols4D; j++) {
            coarseGrid4D[i][j] = false;
            spanTree[i][j] = new Node();
            spanTree[i][j]->row = i;
            spanTree[i][j]->col = j;
            spanTree[i][j]->isOccupied = false;
            spanTree[i][j]->parent = NULL;
            spanTree[i][j]->identity = ++nodeIdentity;
        }
    }

    // Dynamically resize the grid
    grid.resize(rows);
    //And Sign the occupied/unoccupied in the grid and and in the gridD
    for (int i = 0; i < rows; i++) {
        grid[i].resize(cols);
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
    coarseStartX = x / 2;
    coarseStartY= y / 2;

    //Check if the index is out of the map
    if (coarseStartY < 0 || (coarseStartX < 0) || (coarseStartY >= rows4D) || (coarseStartX >= cols4D)) {
        ROS_INFO("Starting location is out of the map!");
        exit(-1);
    }

    vector<vector<bool> > visited = coarseGrid4D;
    //Check that the starting position is not on occupied cell
    notOccupied(coarseStartX, coarseStartY, visited);
    //Make the DFS
    DFS(coarseStartX, coarseStartY, visited);
    cout << "StartX: " << coarseStartX << " StartY: " << coarseStartY << endl;
    cout << "\nFinish\n";

}
void printCoverageStatToFile() {
    ofstream gridFile;
    gridFile.open("coverage_stat.txt");
    gridFile << (availableCells);
    gridFile << endl;
    gridFile << cellsCovered;
    gridFile << endl;
    gridFile << (finalTime - beginTime);
    gridFile.close();
}

int r[4] = { 1, -1, 0, 0 };
int c[4] = { 0, 0, 1, -1 };

//Check if we can go to next cell, or if its already visited/illegal
bool isSafe(int x, int y, vector<vector<bool> > &visited)
{
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
    if(nodeStack.empty()) {
        spanTree[x][y]->parent = NULL;
    } else {
        spanTree[x][y]->parent = nodeStack.top();
        // counter++;
        if(!alreadyHasChild(spanTree[x][y]->parent, spanTree[x][y]->identity)) {
            spanTree[x][y]->parent->childs.push_back(spanTree[x][y]);
        }
    }

    availableCells++;
    for (int k = 0; k < 4; k++) //4 - directions
        if (isSafe(x + r[k], y + c[k], visited)) {
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
        int rIndex[4] = { -1 - i, 1 + i, 0, 0 };
        int cIndex[4] = {   0   ,     0,  -1 - i, 1 + i};

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
bool alreadyHasChild(Node* n, int ident) {
    for(int i =0; i< n->childs.size(); i++) {
        if (n->childs[i]->identity == ident) {
            return true;
        }
    }
    return false;
}
void startDriving() {
    beginTime = ros::Time::now();
    cout << "Start Trip time: " << beginTime <<endl;
    for(std::vector<Station*>::iterator it = stationList.begin(); it != stationList.end(); ++it) {
        double x = (*it)->x;
        double y = (*it)->y;
        cout <<"Station: " << x << "," << y;
        calculateMove(x,y);
        cout <<" And after Matching: " << x << "," << y << endl;
        makeItMove(x,y);
        finalTime = ros::Time::now();
        cout << "So far the time of the trip is: " << (finalTime - beginTime) << endl;
    }
}

void calculateMove(double& x, double& y) {
    double afterX = (x*2*sizeD) * mapResolution + mapOriginX;
    double afterY = (y*2*sizeD) * mapResolution + mapOriginY;
    x = afterX;
    y = afterY;
}
void CreateDriveStations(Node *n) {

    for (int i = 0; i < n->childs.size(); i++) {
        Station* s = new Station();
        s->x= n->childs[i]->row;
        s->y = n->childs[i]->col;
        stationList.push_back(s);
        CreateDriveStations(n->childs[i]);
    }
    if (n->childs.empty()) {
        if (n->parent != NULL) {
            Station *s = new Station();
            s->x = n->parent->row;
            s->y = n->parent->col;
            stationList.push_back(s);
        }
    }
}

void makeItMove(double rowX, double colY){
    double x = rowX, y = colY, theta = 180;
    // create the action client
    MoveBaseClient ac("move_base", true);
    // Wait 60 seconds for the action server to become available
    ROS_INFO("Waiting for the move_base action server");
    ac.waitForServer(ros::Duration(60));

    ROS_INFO("Connected to move base server");
    // Send a goal to move_base
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = rowX;
    goal.target_pose.pose.position.y = colY;
    // Convert the Euler angle to quaternion
    double radians = theta * (M_PI/180);
    tf::Quaternion quaternion;
    quaternion = tf::createQuaternionFromYaw(radians);

    geometry_msgs::Quaternion qMsg;
    tf::quaternionTFToMsg(quaternion, qMsg);

    goal.target_pose.pose.orientation = qMsg;

    // Send the goal command
    ROS_INFO("Sending robot to: x = %f, y = %f, theta = %f", rowX, colY, theta);
    ac.sendGoal(goal);
    cellsCovered++;

    // Wait for the action to return
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("You have reached the goal!");
    }
    else
        ROS_INFO("The base failed for some reason");
}
