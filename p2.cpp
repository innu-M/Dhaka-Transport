#include <bits/stdc++.h>
using namespace std;

const double EARTH_KM = 6371.0;
const double INF = 1e18;

const double SNAP_INTERSECTION_KM = 0.015; // 15m merge nodes
const double SNAP_POINT_TO_NODE_KM = 0.05; // 50m snap source/dest to node
const double PROJECT_MAX_KM = 0.5;         // 500m project to nearest road segment
const double WALK_SEARCH_KM = 5.0;         // find nearest node if off-road

const int GRID = 1000;
const long long SHIFT = 1000000LL;

const double CAR_COST_PER_KM   = 20.0;
const double METRO_COST_PER_KM = 5.0;

enum Mode { CAR = 0, METRO = 1, WALK = 2 };

struct Point { double lat, lon; };

struct Edge {
    int to;
    double cost;      // <-- Dijkstra will minimize this
    double distKm;    // just for printing
    int mode;         // CAR / METRO / WALK
};

struct RoadSeg { vector<int> ids; };

vector<Point> nodePt;
vector<vector<Edge>> g;
vector<RoadSeg> roadSegs;

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

void addEdge(int u, int v, double distKm, int mode) {
    double cost = 0.0;

    if (mode == CAR)   cost = distKm * CAR_COST_PER_KM;
    if (mode == METRO) cost = distKm * METRO_COST_PER_KM;
    if (mode == WALK)  cost = 0.0; // PDF says walking has no cost

    g[u].push_back({v, cost, distKm, mode});
    g[v].push_back({u, cost, distKm, mode});
}

void gridAdd(int id) {
    grid[cellKey(nodePt[id].lat, nodePt[id].lon)].push_back(id);
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

/************ Load CAR road map ************/
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

/************ Load METRO routes ************/
void loadMetroMap(const string& filename) {
    ifstream file(filename);
    if (!file) { cout << "Cannot open " << filename << "\n"; exit(0); }

    string line;
    while (getline(file, line)) {
        if (line.empty()) continue;

        vector<string> parts;
        string tok;
        stringstream ss(line);
        while (getline(ss, tok, ',')) parts.push_back(trimCopy(tok));

        // format: DhakaMetroRail, lon,lat, lon,lat, ... , StartName, EndName
        if ((int)parts.size() < 6) continue;

        // last two are names
        int last = (int)parts.size();
        int endPairs = last - 2;

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

        // connect consecutive stops with METRO edges
        for (int i = 0; i + 1 < (int)ids.size(); i++) {
            int u = ids[i], v = ids[i+1];
            if (u == v) continue;
            double dkm = haversine(nodePt[u], nodePt[v]);
            addEdge(u, v, dkm, METRO);
        }
    }

    cout << "After METRO load, total nodes: " << nodePt.size() << "\n";
}

/************ Insert Source/Destination: snap / project / walk ************/
int insertPoint(Point p, string label) {
    int snap = nearestNode(p, SNAP_POINT_TO_NODE_KM);
    if (snap >= 0) {
        cout << label << " snapped to existing node: " << snap << "\n";
        return snap;
    }

    // only project onto CAR roads (middle of road case)
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

    // off-road -> walk to nearest node (0 cost)
    int offId = addNode(p);
    gridAdd(offId);

    int near = nearestNode(p, WALK_SEARCH_KM);
    if (near >= 0) {
        double walkDist = haversine(p, nodePt[near]);
        addEdge(offId, near, walkDist, WALK);
        cout << label << " off-road. WALK to nearest node " << near
             << " (walkDist=" << fixed << setprecision(3) << walkDist << " km, cost=0)\n";
    } else {
        cout << label << " off-road and no nearby node found.\n";
    }
    return offId;
}

/************ Output KML ************/
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
    return "WALK";
}

int main() {
    cout << "Problem 2: Cheapest route (Car + Metro only)\n";

    loadRoadMap("Roadmap-Dhaka.csv");
    loadMetroMap("Routemap-DhakaMetroRail.csv");

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

    // build path
    vector<int> path;
    for (int v = dst; v != -1; v = par[v]) path.push_back(v);
    reverse(path.begin(), path.end());

    cout << "\nTotal COST = ৳" << fixed << setprecision(2) << cost[dst] << "\n";
    cout << "Path nodes = " << path.size() << "\n";

    // simple text steps
    cout << "\nSteps:\n";
    for (int i = 1; i < (int)path.size(); i++) {
        int u = par[path[i]];
        int v = path[i];
        int m = parMode[v];

        cout << (i) << ") " << modeName(m)
             << " from (" << nodePt[u].lon << "," << nodePt[u].lat << ")"
             << " to ("   << nodePt[v].lon << "," << nodePt[v].lat << ")"
             << " dist=" << fixed << setprecision(3) << parDist[v] << " km";

        if (m == CAR)   cout << ", cost=৳" << fixed << setprecision(2) << parDist[v] * CAR_COST_PER_KM;
        if (m == METRO) cout << ", cost=৳" << fixed << setprecision(2) << parDist[v] * METRO_COST_PER_KM;
        if (m == WALK)  cout << ", cost=৳0.00";

        cout << "\n";
    }

    writeKML("route2.kml", path);
    cout << "\nSaved: route2.kml\n";

    return 0;
}