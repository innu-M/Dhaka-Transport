#include <bits/stdc++.h>
using namespace std;

const double EARTH_KM = 6371.0;
const double INF = 1e18;

const double SNAP_INTERSECTION_KM = 0.015; // 15m
const double SNAP_POINT_TO_NODE_KM = 0.05; // 50m
const double PROJECT_MAX_KM = 0.5;         // 500m
const double WALK_SEARCH_KM = 5.0;         // search nearest node within 5km if off-road

const int GRID = 1000;
const long long SHIFT = 1000000LL;

// Problem 4 costs
const double CAR_COST_PER_KM   = 20.0;
const double METRO_COST_PER_KM = 5.0;
const double BUS_COST_PER_KM   = 7.0;

// Problem 4 speeds
const double SPEED_VEHICLE = 30.0; // km/h (car/bus/metro) :contentReference[oaicite:4]{index=4}
const double SPEED_WALK    = 2.0;  // km/h (walk) :contentReference[oaicite:5]{index=5}

// Schedule: every 15 minutes from 6 AM to 11 PM :contentReference[oaicite:6]{index=6}
const int SERVICE_START = 6 * 60;   // 06:00
const int SERVICE_END   = 23 * 60;  // 23:00
const int HEADWAY       = 15;       // minutes

enum Mode { CAR=0, METRO=1, BUS=2, WALK=3 };

struct Point { double lat, lon; };

struct Edge {
    int to;
    double cost;    // for Dijkstra
    double distKm;  // for time printing
    int mode;
};

struct RoadSeg { vector<int> ids; };

vector<Point> nodePt;
vector<vector<Edge>> g;
vector<RoadSeg> roadSegs; // only car roads used for "project onto road"
unordered_map<long long, vector<int>> grid;

double toRad(double d){ return d * M_PI / 180.0; }

double haversine(Point a, Point b) {
    double dlat = toRad(b.lat - a.lat);
    double dlon = toRad(b.lon - a.lon);
    double s = sin(dlat/2)*sin(dlat/2) +
               cos(toRad(a.lat))*cos(toRad(b.lat)) *
               sin(dlon/2)*sin(dlon/2);
    return 2.0 * EARTH_KM * asin(sqrt(s));
}

long long cellKey(double lat, double lon) {
    long long r = (long long)floor(lat * GRID);
    long long c = (long long)floor(lon * GRID);
    return r * SHIFT + c;
}

int addNode(Point p) {
    int id = (int)nodePt.size();
    nodePt.push_back(p);
    g.push_back({});
    return id;
}

void gridAdd(int id) {
    grid[cellKey(nodePt[id].lat, nodePt[id].lon)].push_back(id);
}

void addEdge(int u, int v, double distKm, int mode) {
    double cost = 0.0;
    if (mode == CAR)   cost = distKm * CAR_COST_PER_KM;
    if (mode == METRO) cost = distKm * METRO_COST_PER_KM;
    if (mode == BUS)   cost = distKm * BUS_COST_PER_KM;
    if (mode == WALK)  cost = 0.0; // walking has no cost :contentReference[oaicite:7]{index=7}

    g[u].push_back({v, cost, distKm, mode});
    g[v].push_back({u, cost, distKm, mode});
}

string trimCopy(string s) {
    while (!s.empty() && isspace((unsigned char)s.front())) s.erase(s.begin());
    while (!s.empty() && isspace((unsigned char)s.back()))  s.pop_back();
    return s;
}

int nearestNode(Point p, double maxDistKm) {
    int best = -1;
    double bestD = maxDistKm;

    int range = (int)(maxDistKm * GRID / 111.0) + 2;
    int r0 = (int)floor(p.lat * GRID);
    int c0 = (int)floor(p.lon * GRID);

    for (int dr = -range; dr <= range; dr++) {
        for (int dc = -range; dc <= range; dc++) {
            long long key = (long long)(r0 + dr) * SHIFT + (long long)(c0 + dc);
            auto it = grid.find(key);
            if (it == grid.end()) continue;

            for (int id : it->second) {
                double d = haversine(p, nodePt[id]);
                if (d < bestD) {
                    bestD = d;
                    best = id;
                }
            }
        }
    }
    return best;
}

Point projectOnSegment(Point p, Point a, Point b) {
    double dx = b.lon - a.lon;
    double dy = b.lat - a.lat;
    double len2 = dx*dx + dy*dy;
    if (len2 < 1e-18) return a;

    double t = ((p.lon - a.lon)*dx + (p.lat - a.lat)*dy) / len2;
    t = max(0.0, min(1.0, t));
    return {a.lat + t*dy, a.lon + t*dx};
}

// CAR roads
void loadRoadMap(const string& filename) {
    ifstream file(filename);
    if (!file) { cout << "Cannot open " << filename << "\n"; exit(0); }

    string line;
    while (getline(file, line)) {
        if (line.empty()) continue;

        vector<string> parts;
        string tok;
        stringstream ss(line);
        while (getline(ss, tok, ',')) parts.push_back(trimCopy(tok));

        if ((int)parts.size() < 4) continue;

        vector<Point> pts;
        for (int i = 1; i + 1 <= (int)parts.size() - 3; i += 2) {
            try {
                double lon = stod(parts[i]);
                double lat = stod(parts[i+1]);
                pts.push_back({lat, lon});
            } catch(...) { break; }
        }
        if (pts.size() < 2) continue;

        RoadSeg seg;

        for (auto p : pts) {
            int snap = nearestNode(p, SNAP_INTERSECTION_KM);
            int id;
            if (snap >= 0) id = snap;
            else { id = addNode(p); gridAdd(id); }
            seg.ids.push_back(id);
        }

        for (int i = 0; i + 1 < (int)seg.ids.size(); i++) {
            int u = seg.ids[i], v = seg.ids[i+1];
            if (u == v) continue;
            double dkm = haversine(nodePt[u], nodePt[v]);
            addEdge(u, v, dkm, CAR);
        }

        roadSegs.push_back(seg);
    }

    cout << "Loaded CAR nodes: " << nodePt.size() << "\n";
}

// METRO / BUS route maps
// Format: TransportType, lon,lat, lon,lat, ... , StartName, EndName :contentReference[oaicite:8]{index=8}
void loadRouteMap(const string& filename, int mode) {
    ifstream file(filename);
    if (!file) { cout << "Cannot open " << filename << "\n"; exit(0); }

    string line;
    while (getline(file, line)) {
        if (line.empty()) continue;

        vector<string> parts;
        string tok;
        stringstream ss(line);
        while (getline(ss, tok, ',')) parts.push_back(trimCopy(tok));

        if ((int)parts.size() < 6) continue;

        int last = (int)parts.size();
        int endPairs = last - 2; // last 2 are stop names

        vector<Point> stops;
        for (int i = 1; i + 1 < endPairs; i += 2) {
            try {
                double lon = stod(parts[i]);
                double lat = stod(parts[i+1]);
                stops.push_back({lat, lon});
            } catch(...) { break; }
        }
        if (stops.size() < 2) continue;

        vector<int> ids;
        for (auto p : stops) {
            int snap = nearestNode(p, SNAP_INTERSECTION_KM);
            int id;
            if (snap >= 0) id = snap;
            else { id = addNode(p); gridAdd(id); }
            ids.push_back(id);
        }

        for (int i = 0; i + 1 < (int)ids.size(); i++) {
            int u = ids[i], v = ids[i+1];
            if (u == v) continue;
            double dkm = haversine(nodePt[u], nodePt[v]);
            addEdge(u, v, dkm, mode);
        }
    }

    cout << "Loaded " << filename << " | total nodes: " << nodePt.size() << "\n";
}

// snap / project / off-road walk
int insertPoint(Point p, string label) {
    int snap = nearestNode(p, SNAP_POINT_TO_NODE_KM);
    if (snap >= 0) {
        cout << label << " snapped to node: " << snap << "\n";
        return snap;
    }

    double bestD = INF;
    Point bestProj{};
    int bestA = -1, bestB = -1;

    for (auto &seg : roadSegs) {
        for (int i = 0; i + 1 < (int)seg.ids.size(); i++) {
            int aId = seg.ids[i];
            int bId = seg.ids[i+1];

            Point proj = projectOnSegment(p, nodePt[aId], nodePt[bId]);
            double d = haversine(p, proj);
            if (d < bestD) {
                bestD = d;
                bestProj = proj;
                bestA = aId;
                bestB = bId;
            }
        }
    }

    if (bestA >= 0 && bestD <= PROJECT_MAX_KM) {
        int newId = addNode(bestProj);
        gridAdd(newId);

        addEdge(newId, bestA, haversine(bestProj, nodePt[bestA]), CAR);
        addEdge(newId, bestB, haversine(bestProj, nodePt[bestB]), CAR);

        cout << label << " projected onto road (dist=" << fixed << setprecision(3) << bestD << " km)\n";
        return newId;
    }

    int offId = addNode(p);
    gridAdd(offId);

    int near = nearestNode(p, WALK_SEARCH_KM);
    if (near >= 0) {
        double walkDist = haversine(p, nodePt[near]);
        addEdge(offId, near, walkDist, WALK);
        cout << label << " off-road. WALK to node " << near
             << " (walkDist=" << fixed << setprecision(3) << walkDist << " km, cost=0)\n";
    } else {
        cout << label << " off-road and no nearby node found.\n";
    }
    return offId;
}

void writeKML(const string& filename, const vector<int>& path) {
    ofstream f(filename);
    f << fixed << setprecision(6);

    f << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    f << "<kml xmlns=\"http://earth.google.com/kml/2.1\">\n";
    f << "<Document><Placemark><name>route.kml</name>\n";
    f << "<LineString><tessellate>1</tessellate><coordinates>\n";

    for (int id : path) f << nodePt[id].lon << "," << nodePt[id].lat << ",0\n";

    f << "</coordinates></LineString></Placemark></Document></kml>\n";
}

string modeName(int m) {
    if (m == CAR) return "Car";
    if (m == METRO) return "Metro";
    if (m == BUS) return "Bus";
    return "Walk";
}

// Parse "5:43 PM" or "06:10 AM"
int parseTimeToMinutes(string s) {
    // remove spaces at ends
    s = trimCopy(s);

    // split into "hh:mm" and "AM/PM"
    string hhmm, ap;
    {
        stringstream ss(s);
        ss >> hhmm >> ap;
    }

    int hh = 0, mm = 0;
    {
        auto pos = hhmm.find(':');
        hh = stoi(hhmm.substr(0, pos));
        mm = stoi(hhmm.substr(pos + 1));
    }

    for (auto &ch : ap) ch = (char)toupper(ch);

    if (ap == "AM") {
        if (hh == 12) hh = 0;
    } else { // PM
        if (hh != 12) hh += 12;
    }

    return hh * 60 + mm;
}

string minutesToTime(int t) {
    // keep within 0..1439 for display (same day display)
    t %= (24 * 60);
    if (t < 0) t += 24 * 60;

    int hh = t / 60;
    int mm = t % 60;

    string ap = (hh >= 12) ? "PM" : "AM";
    int h12 = hh % 12;
    if (h12 == 0) h12 = 12;

    stringstream out;
    out << h12 << ":" << setw(2) << setfill('0') << mm << " " << ap;
    return out.str();
}

// Next departure time for bus/metro (every 15 min, 6AM-11PM)
int nextDeparture(int curMin) {
    int day = curMin / (24 * 60);
    int t = curMin % (24 * 60);
    if (t < 0) t += 24 * 60;

    // before service start -> depart at 6:00
    if (t < SERVICE_START) return day * 24 * 60 + SERVICE_START;

    // after service end -> next day 6:00
    if (t > SERVICE_END) return (day + 1) * 24 * 60 + SERVICE_START;

    // round up to next multiple of 15 from 6:00
    int fromStart = t - SERVICE_START;
    int k = (fromStart + HEADWAY - 1) / HEADWAY;
    int dep = SERVICE_START + k * HEADWAY;
    if (dep > SERVICE_END) return (day + 1) * 24 * 60 + SERVICE_START;
    return day * 24 * 60 + dep;
}

int main() {
    cout << "Problem 4: Cheapest cost route with starting time\n";

    loadRoadMap("Roadmap-Dhaka.csv");
    loadRouteMap("Routemap-DhakaMetroRail.csv", METRO);
    loadRouteMap("Routemap-BikolpoBus.csv", BUS);
    loadRouteMap("Routemap-UttaraBus.csv", BUS);

    double sLat, sLon, dLat, dLon;
    cout << "Enter source lat lon: ";
    cin >> sLat >> sLon;
    cout << "Enter destination lat lon: ";
    cin >> dLat >> dLon;

    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    string startTimeStr;
    cout << "Enter starting time (example 5:43 PM): ";
    getline(cin, startTimeStr);

    int startMin = parseTimeToMinutes(startTimeStr);

    int src = insertPoint({sLat, sLon}, "Source");
    int dst = insertPoint({dLat, dLon}, "Destination");

    // Dijkstra on COST (same as Problem 3)
    int n = (int)nodePt.size();
    vector<double> cost(n, INF);
    vector<int> par(n, -1);
    vector<int> parMode(n, -1);
    vector<double> parDist(n, 0.0);

    priority_queue<pair<double,int>, vector<pair<double,int>>, greater<pair<double,int>>> pq;
    cost[src] = 0;
    pq.push({0, src});

    while (!pq.empty()) {
        auto [cc, u] = pq.top();
        pq.pop();
        if (cc > cost[u]) continue;

        for (auto e : g[u]) {
            if (cost[u] + e.cost < cost[e.to]) {
                cost[e.to] = cost[u] + e.cost;
                par[e.to] = u;
                parMode[e.to] = e.mode;
                parDist[e.to] = e.distKm;
                pq.push({cost[e.to], e.to});
            }
        }
    }

    cout << "\nProblem No: 4\n";
    cout << "Source: (" << fixed << setprecision(6) << sLon << ", " << sLat << ")\n";
    cout << "Destination: (" << fixed << setprecision(6) << dLon << ", " << dLat << ")\n";
    cout << "Starting time at source: " << minutesToTime(startMin) << "\n";

    if (cost[dst] >= INF/2) {
        cout << "No route found\n";
        return 0;
    }

    // Build path
    vector<int> path;
    for (int v = dst; v != -1; v = par[v]) path.push_back(v);
    reverse(path.begin(), path.end());

    // Simulate time along the chosen cheapest-cost path
    int curTime = startMin; // minutes from midnight

    // Destination reaching time will be curTime at end
    // Print step lines like the sample in PDF :contentReference[oaicite:9]{index=9}
    cout << "\nSteps:\n";

curTime = startMin;

int i = 1;
while (i < (int)path.size()) {
    int v = path[i];
    int mode = parMode[v];

    int legStartIndex = i - 1; // node index in path
    double legDist = 0.0;
    double legCost = 0.0;

    // Wait only once if starting a BUS/METRO leg
    int departTime = curTime;
    if (mode == BUS || mode == METRO) {
        departTime = nextDeparture(curTime);
    }

    // Accumulate consecutive edges of same mode
    while (i < (int)path.size() && parMode[path[i]] == mode) {
        int vv = path[i];
        legDist += parDist[vv];

        if (mode == CAR)   legCost += parDist[vv] * CAR_COST_PER_KM;
        if (mode == METRO) legCost += parDist[vv] * METRO_COST_PER_KM;
        if (mode == BUS)   legCost += parDist[vv] * BUS_COST_PER_KM;
        if (mode == WALK)  legCost += 0.0;

        i++;
    }

    double speed = (mode == WALK) ? SPEED_WALK : SPEED_VEHICLE;
    int travelMin = (int)ceil((legDist / speed) * 60.0);

    int arriveTime = departTime + travelMin;

    int uNode = path[legStartIndex];
    int vNode = path[i - 1]; // last node in this leg

    cout << minutesToTime(departTime) << " - " << minutesToTime(arriveTime)
         << ", Cost: ৳" << fixed << setprecision(2) << legCost
         << ": " << (mode == WALK ? "Walk" : "Ride " + modeName(mode))
         << " from (" << fixed << setprecision(6)
         << nodePt[uNode].lon << "," << nodePt[uNode].lat << ")"
         << " to (" << nodePt[vNode].lon << "," << nodePt[vNode].lat << ").\n";

    curTime = arriveTime;
}

    cout << "\nDestination reaching time: " << minutesToTime(curTime) << "\n";
    cout << "Total cost: ৳" << fixed << setprecision(2) << cost[dst] << "\n";

    // KML output (same template section) :contentReference[oaicite:10]{index=10}
    writeKML("route4.kml", path);
    cout << "Saved: route4.kml\n";

    return 0;
}