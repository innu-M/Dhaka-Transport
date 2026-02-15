#include "DhakaTransport.h"

void displayMenu() 
{
    cout << "\nMENU:\n";
    cout << "1. Problem 1: Shortest distance (Car only)\n";
    cout << "2. Problem 2: Cheapest (Car + Metro)\n";
    cout << "3. Problem 3: Cheapest (All modes)\n";
    cout << "4. Problem 4: Cheapest with start time\n";
    cout << "5. Problem 5: Fastest with start time\n";
    cout << "6. Problem 6: Cheapest with deadline\n";
    cout << "0. Exit\n";
    cout << "\nChoice: ";
}

void getSourceDestination(Point& src, Point& dst) 
{
    double slon, slat, dlon, dlat;
    cout << "\nEnter SOURCE (lon lat): ";
    cin >> slon >> slat;
    src = Point(slon, slat);
    cout << "Enter DESTINATION (lon lat): ";
    cin >> dlon >> dlat;
    dst = Point(dlon, dlat);
}

int getStartTime() 
{
    cout << "Enter start time (e.g., '5:43 PM'): ";
    cin.ignore();
    string timeStr;
    getline(cin, timeStr);
    return parseTime(timeStr);
}

int getDeadline() 
{
    cout << "Enter deadline (e.g., '8:12 PM'): ";
    string timeStr;
    getline(cin, timeStr);
    return parseTime(timeStr);
}

void loadGraphData(Graph& g, int problemNum) 
{
    g.loadRoads("Roadmap-Dhaka.csv");
    if (problemNum >= 2) 
    {
        g.loadTransit("Routemap-DhakaMetroRail.csv", "Metro", 3);
    }
    if (problemNum >= 3) 
    {
        g.loadTransit("Routemap-BikolpoBus.csv", "BikolpoBus", 2);
    }
    if (problemNum == 6) 
    {
        g.loadTransit("Routemap-UttaraBus.csv", "UttaraBus", 4);
    }
}

bool findAndValidatePath(Graph& g, Point src, Point dst, int problemNum, int startTime, vector<int>& path, int& sourceNode, int& destNode) 
{
    sourceNode = g.findNearest(src);
    destNode = g.findNearest(dst);
    auto result = g.dijkstra(sourceNode, problemNum, startTime);
    path = g.getPath(result.second, destNode);
    if (path.size() <= 1) 
    {
        cout << "\nNO PATH FOUND!\n";
        return false;
    }
    return true;
}

void generateKML(Graph& g, vector<int>& path, Point src, Point dst, int sourceNode, int destNode, int problemNum) 
{
    vector<Point> coords;
    bool same_o = (fabs(g.nodes[sourceNode].pos.lon - src.lon) < 1e-6 && fabs(g.nodes[sourceNode].pos.lat - src.lat) < 1e-6);
    bool same_d = (fabs(g.nodes[destNode].pos.lon - dst.lon) < 1e-6 && fabs(g.nodes[destNode].pos.lat - dst.lat) < 1e-6);
    
    if (!same_o) 
    {
        if (src.lon >= 90.3 && src.lon <= 90.5 && src.lat >= 23.7 && src.lat <= 23.9)
        {
            coords.push_back(src);
        }
    }
    
    for (int node : path)
    {
        Point p = g.nodes[node].pos;
        if (p.lon >= 90.3 && p.lon <= 90.5 && p.lat >= 23.7 && p.lat <= 23.9)
        {
            coords.push_back(p);
        }
    }
    
    if (!same_d) 
    {
        if (dst.lon >= 90.3 && dst.lon <= 90.5 && dst.lat >= 23.7 && dst.lat <= 23.9)
        {
            coords.push_back(dst);
        }
    }
    
    if (coords.size() >= 2)
    {
        string kmlFile = "problem" + to_string(problemNum) + "_route.kml";
        saveKML(kmlFile, coords);
    }
}

void solveProblem(int problemNum) 
{
    Point src, dst;
    getSourceDestination(src, dst);
    int startTime = 0, deadline = 24 * 60;
    if (problemNum >= 4) 
    {
        startTime = getStartTime();
        if (startTime == -1) 
        {
            cout << "Invalid time format!\n";
            return;
        }
    }
    if (problemNum == 6) 
    {
        deadline = getDeadline();
        if (deadline == -1) 
        {
            cout << "Invalid time format!\n";
            return;
        }
    }
    Graph g;
    loadGraphData(g, problemNum);
    vector<int> path;
    int sourceNode, destNode;
    if (findAndValidatePath(g, src, dst, problemNum, startTime, path, sourceNode, destNode)) 
    {
        printSolution(g, path, src, dst, problemNum, startTime, deadline);
        try 
        {
            generateKML(g, path, src, dst, sourceNode, destNode, problemNum);
        } 
        catch (...) 
        {
        }
    }
}

int main() 
{
    cout << "\n\n Mr. Efficient\n\n";
    while (true)
    {
        displayMenu();
        int choice;
        cin >> choice;
        if (choice == 0) 
        {
            cout << "\nTATA :))\n";
            break;
        }
        if (choice < 1 || choice > 6)
        {
            cout << "Invalid choice!\n";
            continue;
        }
        solveProblem(choice);
        cin.ignore();
        cin.get();
    }
    return 0;
}