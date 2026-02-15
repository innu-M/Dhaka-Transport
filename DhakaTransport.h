#ifndef DHAKA_TRANSPORT_H
#define DHAKA_TRANSPORT_H
#include <bits/stdc++.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
using namespace std;

const double P1_CAR_COST = 20.0;
const double P2_CAR_COST = 20.0;
const double P2_METRO_COST = 5.0;
const double P3_CAR_COST = 20.0;
const double P3_METRO_COST = 5.0;
const double P3_BIKOLPO_COST = 7.0;
const double P4_CAR_COST = 20.0;
const double P4_METRO_COST = 5.0;
const double P4_BIKOLPO_COST = 7.0;
const double P4_SPEED = 30.0;
const int P4_SCHEDULE_INTERVAL = 15;
const int P4_SERVICE_START = 6 * 60;
const int P4_SERVICE_END = 23 * 60;
const double P5_SPEED = 10.0;
const int P5_SCHEDULE_INTERVAL = 15;
const int P5_SERVICE_START = 6 * 60;
const int P5_SERVICE_END = 23 * 60;
const double P6_CAR_COST = 20.0;
const double P6_METRO_COST = 5.0;
const double P6_BIKOLPO_COST = 7.0;
const double P6_UTTARA_COST = 10.0;
const double P6_CAR_SPEED = 20.0;
const double P6_METRO_SPEED = 15.0;
const double P6_BIKOLPO_SPEED = 10.0;
const double P6_UTTARA_SPEED = 12.0;
const int P6_METRO_INTERVAL = 5;
const int P6_BIKOLPO_INTERVAL = 20;
const int P6_UTTARA_INTERVAL = 10;
const int P6_METRO_START = 1 * 60;
const int P6_METRO_END = 23 * 60;
const int P6_BIKOLPO_START = 7 * 60;
const int P6_BIKOLPO_END = 22 * 60;
const int P6_UTTARA_START = 6 * 60;
const int P6_UTTARA_END = 23 * 60;
const double WALKING_SPEED = 2.0;
const double WALKING_COST = 0.0;

struct Point 
{
    double lon, lat;
    Point();
    Point(double x, double y);
    bool operator==(const Point& other) const;
};

struct Node 
{
    int id;
    Point pos;
    string name;
};

double deg2rad(double deg);
double calcDistance(double lat1, double lon1, double lat2, double lon2);
int parseTime(string s);
string formatTime(int mins);
int getWaitMinutes(int currentMinutes, int problemNum, int transportType);
bool isServiceRunning(int timeMinutes, int problemNum, int transportType);

class Graph 
{
public:
    map<int, Node> nodes;
    map<int, vector<pair<int, double>>> adj;
    map<pair<int,int>, double> edgeDist;
    map<pair<int,int>, int> edgeType;
    map<string, int> nodeCache;
    int nodeCount;
    Graph();
    string makeKey(Point p);
    int getNode(Point p, string name = "");
    void addEdge(int u, int v, double dist, int type, bool bidirectional);
    void loadRoads(string filename);
    void loadTransit(string filename, string typeName, int typeNum);
    int findNearest(Point p);
    pair<map<int,double>, map<int,int>> dijkstra(int source, int problemNum, int startTime = 0);
    vector<int> getPath(map<int,int>& parent, int dest);
};

void saveKML(string filename, vector<Point>& coords);
void printSolution(Graph& g, vector<int>& path, Point src, Point dst, int problemNum, int startTime = 0, int deadline = 0);

#endif