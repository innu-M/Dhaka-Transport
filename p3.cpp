#include <bits/stdc++.h>
using namespace std;

const double EARTH_KM = 6371.0;
const double INF = 1e18;

const double SNAP_INTERSECTION_KM = 0.015; // 15m merge nodes while loading
const double SNAP_POINT_TO_NODE_KM = 0.05; // 50m snap source/dest
const double PROJECT_MAX_KM = 0.5;         // 500m project onto road
const double WALK_SEARCH_KM = 5.0;         // if off-road, walk to nearest node (0 cost)

const int GRID = 1000;
const long long SHIFT = 1000000LL;

// Problem 3 costs: Car 20/km, Metro 5/km, Bus 7/km :contentReference[oaicite:2]{index=2}
const double CAR_COST_PER_KM   = 20.0;
const double METRO_COST_PER_KM = 5.0;
const double BUS_COST_PER_KM   = 7.0;

enum Mode { CAR = 0, METRO = 1, BUS = 2, WALK = 3 };

struct Point { double lat, lon; };

struct Edge {
    int to;
    double cost;   // Dijkstra minimizes this
    double distKm; // for printing
    int mode;
};

struct RoadSeg { vector<int> ids; }; // only CAR road segments for projection case

vector<Point> nodePt;
vector<vector<Edge>> g;
vector<RoadSeg> roadSegs; // only car roads here
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
    if (mode == WALK)  cost = 0.0; // assignment says walk has no cost (and is used when off-road) :contentReference[oaicite:3]{index=3}

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
    // simple projection using lat/lon as x/y (good enough for assignment)
    double dx = b.lon - a.lon;
    double dy = b.lat - a.lat;
    double len2 = dx*dx + dy*dy;
    if (len2 < 1e-18) return a;

    double t = ((p.lon - a.lon)*dx + (p.lat - a.lat)*dy) / len2;
    t = max(0.0, min(1.0, t));
    return {a.lat + t*dy, a.lon + t*dx};
}

// Load CAR roads (Roadmap-Dhaka.csv)
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

    cout << "Loaded CAR road nodes: " << nodePt.size() << "\n";
}

// Load a route-map file (Metro or Bus): Routemap-xxx.csv
// Format: TransportName, lon,lat, lon,lat, ..., StartName, EndName :contentReference[oaicite:4]{index=4}
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
        int endPairs = last - 2; // last two are stop names

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

    cout << "Loaded routes from " << filename << " | total nodes now: " << nodePt.size() << "\n";
}

int insertPoint(Point p, string label) {
    // Case 1: near an existing node (road intersection / bus stop / metro stop)
    int snap = nearestNode(p, SNAP_POINT_TO_NODE_KM);
    if (snap >= 0) {
        cout << label << " snapped to node: " << snap << "\n";
        return snap;
    }

    // Case 2: project onto nearest CAR road segment (middle of a road)
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

    // Case 3: off-road -> walk to nearest node with 0 cost :contentReference[oaicite:5]{index=5}
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

    for (int id : path) {
        f << nodePt[id].lon << "," << nodePt[id].lat << ",0\n";
    }

    f << "</coordinates></LineString></Placemark></Document></kml>\n";
}

string modeName(int m) {
    if (m == CAR) return "CAR";
    if (m == METRO) return "METRO";
    if (m == BUS) return "BUS";
    return "WALK";
}

int main() {
    cout << "Problem 3: Cheapest route (Car + Metro + Bus)\n";

    loadRoadMap("Roadmap-Dhaka.csv");
    loadRouteMap("Routemap-DhakaMetroRail.csv", METRO);
    loadRouteMap("Routemap-BikolpoBus.csv", BUS);
    loadRouteMap("Routemap-UttaraBus.csv", BUS);

    double sLat, sLon, dLat, dLon;
    cout << "Enter source lat lon: ";
    cin >> sLat >> sLon;
    cout << "Enter destination lat lon: ";
    cin >> dLat >> dLon;

    int src = insertPoint({sLat, sLon}, "Source");
    int dst = insertPoint({dLat, dLon}, "Destination");

    // Dijkstra on COST
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

    if (cost[dst] >= INF/2) {
        cout << "No route found\n";
        return 0;
    }

    // Build node path
    vector<int> path;
    for (int v = dst; v != -1; v = par[v]) path.push_back(v);
    reverse(path.begin(), path.end());

    cout << "\nTotal COST = ৳" << fixed << setprecision(2) << cost[dst] << "\n";
    cout << "Path nodes = " << path.size() << "\n";

    cout << "\nSteps:\n";
    for (int i = 1; i < (int)path.size(); i++) {
        int v = path[i];
        int u = par[v];
        int m = parMode[v];
        double dkm = parDist[v];

        double segCost = 0.0;
        if (m == CAR) segCost = dkm * CAR_COST_PER_KM;
        if (m == METRO) segCost = dkm * METRO_COST_PER_KM;
        if (m == BUS) segCost = dkm * BUS_COST_PER_KM;
        if (m == WALK) segCost = 0.0;

        cout << i << ") " << modeName(m)
             << " from (" << nodePt[u].lon << "," << nodePt[u].lat << ")"
             << " to (" << nodePt[v].lon << "," << nodePt[v].lat << ")"
             << " dist=" << fixed << setprecision(3) << dkm << " km"
             << ", cost=৳" << fixed << setprecision(2) << segCost
             << "\n";
    }

    writeKML("route3.kml", path);
    cout << "\nSaved: route3.kml\n";

    return 0;
}