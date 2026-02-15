#include "DhakaTransport.h"

Point::Point() : lon(0), lat(0) {}
Point::Point(double x, double y) : lon(x), lat(y) {}
bool Point::operator==(const Point& other) const 
{
    return fabs(lon - other.lon) < 1e-7 && fabs(lat - other.lat) < 1e-7;
}

double deg2rad(double deg) 
{
    return deg * (M_PI / 180.0);
}

double calcDistance(double lat1, double lon1, double lat2, double lon2) 
{
    const double R = 6371.0;
    double dLat = deg2rad(lat2 - lat1);
    double dLon = deg2rad(lon2 - lon1);
    double a = sin(dLat/2) * sin(dLat/2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

int parseTime(string s) 
{
    int h, m;
    char ampm[10];
    if (sscanf(s.c_str(), "%d:%d %s", &h, &m, ampm) == 3)
    {
        string ap = ampm;
        for (char& c : ap) c = toupper(c);
        if (ap == "PM" && h != 12) h += 12;
        if (ap == "AM" && h == 12) h = 0;
        return h * 60 + m;
    }
    if (sscanf(s.c_str(), "%d:%d", &h, &m) == 2) 
    {
        if (h >= 0 && h < 24 && m >= 0 && m < 60) 
        {
            return h * 60 + m;
        }
    }
    return -1;
}

string formatTime(int mins) 
{
    int h = (mins / 60) % 24;
    int m = mins % 60;
    string ap = h >= 12 ? "PM" : "AM";
    int dh = h % 12;
    if (dh == 0) dh = 12;
    char buf[20];
    sprintf(buf, "%d:%02d %s", dh, m, ap.c_str());
    return string(buf);
}

int getWaitMinutes(int currentMinutes, int problemNum, int transportType) 
{
    if (transportType == 1) return 0;
    if (problemNum == 4 || problemNum == 5) 
    {
        int mod = currentMinutes % P4_SCHEDULE_INTERVAL;
        return (mod == 0) ? 0 : (P4_SCHEDULE_INTERVAL - mod);
    }
    else if (problemNum == 6)
    {
        if (transportType == 3) 
        {
            int mod = currentMinutes % P6_METRO_INTERVAL;
            return (mod == 0) ? 0 : (P6_METRO_INTERVAL - mod);
        }
        else if (transportType == 2)
        {
            int mod = currentMinutes % P6_BIKOLPO_INTERVAL;
            return (mod == 0) ? 0 : (P6_BIKOLPO_INTERVAL - mod);
        }
        else if (transportType == 4)
        {
            int mod = currentMinutes % P6_UTTARA_INTERVAL;
            return (mod == 0) ? 0 : (P6_UTTARA_INTERVAL - mod);
        }
    }
    return 0;
}

bool isServiceRunning(int timeMinutes, int problemNum, int transportType) 
{
    if (transportType == 1) return true;
    if (problemNum == 4 || problemNum == 5) 
    {
        return (timeMinutes >= P4_SERVICE_START && timeMinutes < P4_SERVICE_END);
    }
    else if (problemNum == 6) 
    {
        if (transportType == 3) 
        {
            return (timeMinutes >= P6_METRO_START && timeMinutes < P6_METRO_END);
        }
        else if (transportType == 2) 
        {
            return (timeMinutes >= P6_BIKOLPO_START && timeMinutes < P6_BIKOLPO_END);
        }
        else if (transportType == 4) 
        {
            return (timeMinutes >= P6_UTTARA_START && timeMinutes < P6_UTTARA_END);
        }
    }
    return true;
}

Graph::Graph() : nodeCount(0) {}

string Graph::makeKey(Point p) 
{
    char buf[100];
    sprintf(buf, "%.7f,%.7f", p.lon, p.lat);
    return string(buf);
}

int Graph::getNode(Point p, string name) 
{
    string key = makeKey(p);
    if (nodeCache.count(key)) 
    {
        int id = nodeCache[key];
        if (!name.empty() && nodes[id].name.empty()) 
        {
            nodes[id].name = name;
        }
        return id;
    }
    Node n;
    n.id = nodeCount++;
    n.pos = p;
    n.name = name;
    nodes[n.id] = n;
    nodeCache[key] = n.id;
    return n.id;
}

void Graph::addEdge(int u, int v, double dist, int type, bool bidirectional) 
{
    if (dist < 1e-9) return;
    adj[u].push_back({v, dist});
    edgeDist[{u, v}] = dist;
    edgeType[{u, v}] = type;
    if (bidirectional) 
    {
        adj[v].push_back({u, dist});
        edgeDist[{v, u}] = dist;
        edgeType[{v, u}] = type;
    }
}

void Graph::loadRoads(string filename) 
{
    ifstream file(filename);
    if (!file) return;
    string line;
    while (getline(file, line)) 
    {
        if (line.empty()) continue;
        stringstream ss(line);
        string token;
        vector<Point> points;
        getline(ss, token, ',');
        while (getline(ss, token, ',')) 
        {
            double lon = atof(token.c_str());
            if (getline(ss, token, ',')) 
            {
                double lat = atof(token.c_str());
                if (lon >= 90.3 && lon <= 90.5 && lat >= 23.7 && lat <= 23.9)
                {
                    Point p(lon, lat);
                    if (points.empty() || !(points.back() == p)) 
                    {
                        points.push_back(p);
                    }
                }
            }
        }
        for (size_t i = 0; i + 1 < points.size(); i++) 
        {
            int a = getNode(points[i]);
            int b = getNode(points[i+1]);
            double d = calcDistance(points[i].lat, points[i].lon, points[i+1].lat, points[i+1].lon);
            addEdge(a, b, d, 1, true);
        }
    }
    file.close();
}

void Graph::loadTransit(string filename, string typeName, int typeNum)
{
    ifstream file(filename);
    if (!file) return;
    string line;
    while (getline(file, line)) 
    {
        if (line.empty()) continue;
        stringstream ss(line);
        vector<string> tokens;
        string token;
        while (getline(ss, token, ',')) 
        {
            tokens.push_back(token);
        }
        if (tokens.size() < 6) continue;
        string startStation = tokens[tokens.size() - 2];
        string endStation = tokens[tokens.size() - 1];
        vector<Point> points;
        for (size_t i = 1; i + 1 < tokens.size() - 2; i += 2) 
        {
            double lon = atof(tokens[i].c_str());
            double lat = atof(tokens[i+1].c_str());
            if (lon >= 90.3 && lon <= 90.5 && lat >= 23.7 && lat <= 23.9)
            {
                Point p(lon, lat);
                if (points.empty() || !(points.back() == p)) 
                {
                    points.push_back(p);
                }
            }
        }
        if (points.size() < 2) continue;
        for (size_t i = 0; i + 1 < points.size(); i++) 
        {
            string name1 = (i == 0) ? startStation : "";
            string name2 = (i == points.size() - 2) ? endStation : "";
            int a = getNode(points[i], name1);
            int b = getNode(points[i+1], name2);
            double d = calcDistance(points[i].lat, points[i].lon, points[i+1].lat, points[i+1].lon);
            addEdge(a, b, d, typeNum, false);
        }
    }
    file.close();
}

int Graph::findNearest(Point p) 
{
    if (nodes.empty()) return -1;
    int best = nodes.begin()->first;
    double bestDist = calcDistance(p.lat, p.lon, nodes[best].pos.lat, nodes[best].pos.lon);
    for (auto& kv : nodes) 
    {
        double d = calcDistance(p.lat, p.lon, kv.second.pos.lat, kv.second.pos.lon);
        if (d < bestDist) 
        {
            bestDist = d;
            best = kv.first;
        }
    }
    return best;
}

pair<map<int,double>, map<int,int>> Graph::dijkstra(int source, int problemNum, int startTime) 
{
    map<int, double> dist;
    map<int, int> parent;
    map<int, int> arrivalTime;
    priority_queue<pair<double,int>, vector<pair<double,int>>, greater<pair<double,int>>> pq;
    for (auto& kv : nodes) 
    {
        dist[kv.first] = 1e18;
        parent[kv.first] = -1;
        arrivalTime[kv.first] = startTime;
    }
    dist[source] = 0;
    arrivalTime[source] = startTime;
    pq.push({0, source});
    while (!pq.empty()) 
    {
        double d = pq.top().first;
        int u = pq.top().second;
        pq.pop();
        if (d > dist[u]) continue;
        for (auto& edge : adj[u])
        {
            int v = edge.first;
            double distance = edge.second;
            int type = edgeType[{u, v}];
            double weight = 0;
            double speed = 30.0;
            int currentTime = arrivalTime[u];
            if (problemNum == 1) 
            {
                if (type != 1) continue;
                weight = distance;
            }
            else if (problemNum == 2) 
            {
                if (type == 2 || type == 4) continue;
                double costPerKm = (type == 1) ? P2_CAR_COST : P2_METRO_COST;
                weight = distance * costPerKm;
            }
            else if (problemNum == 3) 
            {
                if (type == 4) continue;
                double costPerKm;
                if (type == 1) costPerKm = P3_CAR_COST;
                else if (type == 3) costPerKm = P3_METRO_COST;
                else costPerKm = P3_BIKOLPO_COST;
                weight = distance * costPerKm;
            }
            else if (problemNum == 4) 
            {
                if (type == 4) continue;
                if (type != 1 && !isServiceRunning(currentTime, problemNum, type)) continue;
                double costPerKm;
                if (type == 1) costPerKm = P4_CAR_COST;
                else if (type == 3) costPerKm = P4_METRO_COST;
                else costPerKm = P4_BIKOLPO_COST;
                weight = distance * costPerKm;
                speed = P4_SPEED;
            }
            else if (problemNum == 5) 
            {
                if (type == 4) continue;
                if (type != 1 && !isServiceRunning(currentTime, problemNum, type)) continue;
                speed = P5_SPEED;
                int travelTime = (int)ceil((distance / speed) * 60.0);
                int waitTime = (type != 1) ? getWaitMinutes(currentTime, problemNum, type) : 0;
                weight = travelTime + waitTime;
            }
            else if (problemNum == 6) 
            {
                if (type != 1 && !isServiceRunning(currentTime, problemNum, type)) continue;
                double costPerKm;
                if (type == 1) 
                {
                    costPerKm = P6_CAR_COST;
                    speed = P6_CAR_SPEED;
                }
                else if (type == 3) 
                {
                    costPerKm = P6_METRO_COST;
                    speed = P6_METRO_SPEED;
                }
                else if (type == 2) 
                {
                    costPerKm = P6_BIKOLPO_COST;
                    speed = P6_BIKOLPO_SPEED;
                }
                else
                {
                    costPerKm = P6_UTTARA_COST;
                    speed = P6_UTTARA_SPEED;
                }
                weight = distance * costPerKm;
            }
            if (dist[u] + weight < dist[v]) 
            {
                dist[v] = dist[u] + weight;
                parent[v] = u;
                if (problemNum >= 4)
                {
                    int travelTime = (int)ceil((distance / speed) * 60.0);
                    int waitTime = (type != 1) ? getWaitMinutes(currentTime, problemNum, type) : 0;
                    arrivalTime[v] = currentTime + waitTime + travelTime;
                }
                pq.push({dist[v], v});
            }
        }
    }
    return {dist, parent};
}

vector<int> Graph::getPath(map<int,int>& parent, int dest) 
{
    vector<int> path;
    int cur = dest;
    while (cur != -1) 
    {
        path.push_back(cur);
        cur = parent[cur];
    }
    reverse(path.begin(), path.end());
    return path;
}

void saveKML(string filename, vector<Point>& coords) 
{
    if (coords.size() < 2) return;
    
    vector<Point> validCoords;
    for (auto& p : coords)
    {
        if (!isinf(p.lon) && !isinf(p.lat) && !isnan(p.lon) && !isnan(p.lat))
        {
            if (p.lon >= -180 && p.lon <= 180 && p.lat >= -90 && p.lat <= 90)
            {
                validCoords.push_back(p);
            }
        }
    }
    
    if (validCoords.size() < 2) return;
    
    ofstream f(filename);
    if (!f) return;
    
    f << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    f << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
    f << "  <Document>\n";
    f << "    <name>Route</name>\n";
    f << "    <Style id=\"lineStyle\">\n";
    f << "      <LineStyle>\n";
    f << "        <color>ff0000ff</color>\n";
    f << "        <width>3</width>\n";
    f << "      </LineStyle>\n";
    f << "    </Style>\n";
    f << "    <Placemark>\n";
    f << "      <name>Path</name>\n";
    f << "      <styleUrl>#lineStyle</styleUrl>\n";
    f << "      <LineString>\n";
    f << "        <coordinates>\n";
    
    f << fixed << setprecision(6);
    for (auto& p : validCoords)
    {
        f << "          " << p.lon << "," << p.lat << ",0\n";
    }
    
    f << "        </coordinates>\n";
    f << "      </LineString>\n";
    f << "    </Placemark>\n";
    f << "  </Document>\n";
    f << "</kml>\n";
    f.close();
}

void printSolution(Graph& g, vector<int>& path, Point src, Point dst, int problemNum, int startTime, int deadline) 
{
    cout << "\nProblem no : " << problemNum << "\n";
    bool same_o = (fabs(g.nodes[path[0]].pos.lon - src.lon) < 1e-6 && fabs(g.nodes[path[0]].pos.lat - src.lat) < 1e-6);
    bool same_d = (fabs(g.nodes[path.back()].pos.lon - dst.lon) < 1e-6 && fabs(g.nodes[path.back()].pos.lat - dst.lat) < 1e-6);
    if (problemNum == 2 || problemNum == 3)
    {
        double totalCost = 0;
        for (size_t i = 0; i + 1 < path.size(); i++) 
        {
            int u = path[i];
            int v = path[i+1];
            int type = g.edgeType[{u, v}];
            double distance = g.edgeDist[{u, v}];
            double costPerKm = 0;
            if (type == 1)
            {
                if (problemNum == 2) costPerKm = P2_CAR_COST;
                else costPerKm = P3_CAR_COST;
            }
            else if (type == 3) 
            {
                if (problemNum == 2) costPerKm = P2_METRO_COST;
                else costPerKm = P3_METRO_COST;
            }
            else if (type == 2) 
            {
                costPerKm = P3_BIKOLPO_COST;
            }
            totalCost += distance * costPerKm;
        }
        cout << fixed << setprecision(15) << "Minimum cost is " << totalCost << "৳ \n";
    }
    cout << "Source:  (" << src.lon << ", " << src.lat << ")\n";
    cout << "Destination:  (" << dst.lon << ", " << dst.lat << ")\n";
    int currentTime = startTime;
    int stepNum = 1;
    if (!same_o) 
    {
        double walkDist = calcDistance(src.lat, src.lon, g.nodes[path[0]].pos.lat, g.nodes[path[0]].pos.lon);
        int walkMins = (int)ceil((walkDist / WALKING_SPEED) * 60.0);
        if (problemNum >= 4)
        {
            int hours = currentTime / 60;
            int mins = currentTime % 60;
            cout << "Step " << stepNum << " Time : " << hours << " : " << mins << ". ";
        }
        else
        {
            cout << "Step " << stepNum << " ";
        }
        cout << "Cost: 0৳ . Walk from Source (" << src.lon << ", " << src.lat << ") to (" 
             << g.nodes[path[0]].pos.lon << ", " << g.nodes[path[0]].pos.lat << ")\n";
        currentTime += walkMins;
        stepNum++;
    }
    for (size_t i = 0; i + 1 < path.size(); i++) 
    {
        int u = path[i];
        int v = path[i+1];
        int type = g.edgeType[{u, v}];
        double distance = g.edgeDist[{u, v}];
        string transportName;
        double costPerKm = 0;
        double speed = 30.0;
        if (type == 1)
        {
            transportName = "Car";
            if (problemNum == 1) { costPerKm = P1_CAR_COST; speed = 30.0; }
            else if (problemNum == 2) { costPerKm = P2_CAR_COST; speed = 30.0; }
            else if (problemNum == 3) { costPerKm = P3_CAR_COST; speed = 30.0; }
            else if (problemNum == 4) { costPerKm = P4_CAR_COST; speed = P4_SPEED; }
            else if (problemNum == 5) { costPerKm = P4_CAR_COST; speed = P5_SPEED; }
            else if (problemNum == 6) { costPerKm = P6_CAR_COST; speed = P6_CAR_SPEED; }
        }
        else if (type == 3)
        {
            transportName = "Metro";
            if (problemNum == 2) { costPerKm = P2_METRO_COST; speed = 30.0; }
            else if (problemNum == 3) { costPerKm = P3_METRO_COST; speed = 30.0; }
            else if (problemNum == 4) { costPerKm = P4_METRO_COST; speed = P4_SPEED; }
            else if (problemNum == 5) { costPerKm = P4_METRO_COST; speed = P5_SPEED; }
            else if (problemNum == 6) { costPerKm = P6_METRO_COST; speed = P6_METRO_SPEED; }
        }
        else if (type == 2)
        {
            transportName = "Bikolpo Bus";
            if (problemNum == 3) { costPerKm = P3_BIKOLPO_COST; speed = 30.0; }
            else if (problemNum == 4) { costPerKm = P4_BIKOLPO_COST; speed = P4_SPEED; }
            else if (problemNum == 5) { costPerKm = P4_BIKOLPO_COST; speed = P5_SPEED; }
            else if (problemNum == 6) { costPerKm = P6_BIKOLPO_COST; speed = P6_BIKOLPO_SPEED; }
        }
        else
        {
            transportName = "Uttara Bus";
            costPerKm = P6_UTTARA_COST;
            speed = P6_UTTARA_SPEED;
        }
        double edgeCost = distance * costPerKm;
        int travelMins = (int)ceil((distance / speed) * 60.0);
        if (problemNum >= 4 && type != 1)
        {
            int wait = getWaitMinutes(currentTime, problemNum, type);
            currentTime += wait;
        }
        int startT = currentTime;
        currentTime += travelMins;
        string startName = g.nodes[u].name;
        string endName = g.nodes[v].name;
        cout << "Step " << stepNum << " ";
        if (problemNum >= 4)
        {
            int hours = startT / 60;
            int mins = startT % 60;
            cout << "Time : " << hours << " : " << mins << ". ";
        }
        if (problemNum == 5)
        {
            cout << "Cost: " << (distance / speed * 60.0) << "time . ";
        }
        else
        {
            cout << fixed << setprecision(15) << "Cost: " << edgeCost << "৳ . ";
        }
        if (i == 0 && !same_o)
        {
            cout << " Ride " << transportName << " from ";
        }
        else
        {
            cout << "Ride " << transportName << " from ";
        }
        if (!startName.empty())
            cout << " " << startName;
        cout << "(" << g.nodes[u].pos.lon << ", " << g.nodes[u].pos.lat << ") to ";
        if (!endName.empty())
            cout << " " << endName << " ";
        cout << "(" << g.nodes[v].pos.lon << ", " << g.nodes[v].pos.lat << ")\n";
        stepNum++;
    }
    if (!same_d) 
    {
        cout << "Step " << stepNum << " ";
        if (problemNum >= 4)
        {
            int hours = currentTime / 60;
            int mins = currentTime % 60;
            cout << "Time : " << hours << " : " << mins << ". ";
        }
        if (problemNum == 5)
        {
            double walkDist = calcDistance(g.nodes[path.back()].pos.lat, g.nodes[path.back()].pos.lon, dst.lat, dst.lon);
            cout << "Cost: " << (walkDist / WALKING_SPEED * 60.0) << "Time. ";
        }
        else
        {
            cout << "Cost: 0৳ . ";
        }
        cout << " Walk from ";
        cout << " (" << g.nodes[path.back()].pos.lon << ", " << g.nodes[path.back()].pos.lat << ") to Destination  ";
        cout << "(" << dst.lon << ", " << dst.lat << ")\n";
    }
}