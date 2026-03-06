#include <bits/stdc++.h>
using namespace std;

double EARTH_KM = 6371.0;
double INF = 1e18;

double SNAP_INTERSECTION_KM = 0.015;
double SNAP_POINT_TO_NODE_KM = 0.05;
double PROJECT_MAX_KM = 0.5;
double WALK_SEARCH_KM = 5.0;

int GRID = 1000;
long long SHIFT = 1000000LL;


struct Point 
{ double lat, lon; };

struct Edge {
    int to;
    double w;
    int mode; // 0 = CAR, 1 = WALK
};

struct RoadSeg 
{ vector<int> ids; };

vector<Point> nodePt;
vector<vector<Edge>> g;
vector<RoadSeg> roadSegs;

unordered_map<long long, vector<int>> grid;

double toRad(double d)
{ return d * M_PI / 180.0; }

double haversine(Point a, Point b) 
{
    double dlat = toRad(b.lat - a.lat);
    double dlon = toRad(b.lon - a.lon);
    double s = sin(dlat/2)*sin(dlat/2) +
               cos(toRad(a.lat))*cos(toRad(b.lat)) *
               sin(dlon/2)*sin(dlon/2);
    return 2.0 * EARTH_KM * asin(sqrt(s));
}

long long cellKey(double lat, double lon) 
{
    long long r = (long long)floor(lat * GRID);
    long long c = (long long)floor(lon * GRID);
    return r * SHIFT + c;
}

int addNode(Point p) 
{
    int id = (int)nodePt.size();
    nodePt.push_back(p);
    g.push_back({});
    return id;
}

void addEdge(int u, int v, double w, int mode) 
{
    g[u].push_back({v, w, mode});
    g[v].push_back({u, w, mode});
}

void gridAdd(int id) 
{
    grid[cellKey(nodePt[id].lat, nodePt[id].lon)].push_back(id);
}

int nearestNode(Point p, double maxDistKm) 
{
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

Point projectOnSegment(Point p, Point a, Point b) 
{
    
    double dx = b.lon - a.lon;
    double dy = b.lat - a.lat;
    double len2 = dx*dx + dy*dy;
    if (len2 < 1e-18)
     return a;

    double t = ((p.lon - a.lon)*dx + (p.lat - a.lat)*dy) / len2;
    t = max(0.0, min(1.0, t));

    return {a.lat + t*dy, a.lon + t*dx};
}

string trimCopy(string s) 
{
    while (!s.empty() && isspace((unsigned char)s.front())) s.erase(s.begin());
    while (!s.empty() && isspace((unsigned char)s.back()))  s.pop_back();
    return s;
}

void loadRoadMap(const string& filename) {
    ifstream file(filename);
    if (!file) 
    {
        cout << "Cannot open file: " << filename << "\n";
        exit(0);
    }

    string line;
    while (getline(file, line)) 
    {
        if (line.empty()) 
        continue;

        vector<string> parts;
        string tok;
        stringstream ss(line);
        while (getline(ss, tok, ',')) parts.push_back(trimCopy(tok));

        if ((int)parts.size() < 4) 
        continue;

        vector<Point> pts;
       for (int i = 1; i + 1 <= (int)parts.size() - 3; i += 2) {
            try {
                double lon = stod(parts[i]);
                double lat = stod(parts[i+1]);
                pts.push_back({lat, lon});
            } catch(...) {
                break;
            }
        }
        if (pts.size() < 2) continue;

        RoadSeg seg;

       
        for (auto p : pts) {
            int snap = nearestNode(p, SNAP_INTERSECTION_KM);
            int id;
            if (snap >= 0) id = snap;
            else {
                id = addNode(p);
                gridAdd(id);
            }
            seg.ids.push_back(id);
        }

        
        for (int i = 0; i + 1 < (int)seg.ids.size(); i++) {
            int u = seg.ids[i];
            int v = seg.ids[i+1];
            if (u == v) continue;
            addEdge(u, v, haversine(nodePt[u], nodePt[v]), 0);
        }

        roadSegs.push_back(seg);
    }

    cout << "Road nodes loaded: " << nodePt.size() << "\n";
}

int insertPoint(Point p, string label) {
    // Case 1
    int snap = nearestNode(p, SNAP_POINT_TO_NODE_KM);
    if (snap >= 0) {
        cout << label << " snapped to existing node: " << snap << "\n";
        return snap;
    }

    // Case 2
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
        addEdge(newId, bestA, haversine(bestProj, nodePt[bestA]), 0);
        addEdge(newId, bestB, haversine(bestProj, nodePt[bestB]), 0);

        cout << label << " projected onto road (dist=" << fixed << setprecision(3) << bestD << " km)\n";
        return newId;
    }

    // Case 3
    int offId = addNode(p);
    gridAdd(offId);

    int near = nearestNode(p, WALK_SEARCH_KM);
    if (near >= 0) {
        double w = haversine(p, nodePt[near]);
        addEdge(offId, near, w, 1);
        cout << label << " off-road. WALK to nearest node " << near
             << " (walk=" << fixed << setprecision(3) << w << " km)\n";
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

int main() {
    cout << "Problem 1: shortest distance CAR route\n";

    loadRoadMap("Roadmap-Dhaka.csv");

    double sLat, sLon, dLat, dLon;
    cout << "Source lat lon: ";
    cin >> sLat >> sLon;
    cout << "Destination lat lon: ";
    cin >> dLat >> dLon;

    int src = insertPoint({sLat, sLon}, "Source");
    int dst = insertPoint({dLat, dLon}, "Destination");

   
    int n = (int)nodePt.size();
    vector<double> dist(n, INF);
    vector<int> par(n, -1);
    vector<int> parMode(n, 0);

    priority_queue<pair<double,int>, vector<pair<double,int>>, greater<pair<double,int>>> pq;
    dist[src] = 0;
    pq.push({0, src});

    while (!pq.empty()) {
        auto [cd, u] = pq.top();
        pq.pop();
        if (cd > dist[u]) continue;

        for (auto e : g[u]) {
            if (dist[u] + e.w < dist[e.to]) {
                dist[e.to] = dist[u] + e.w;
                par[e.to] = u;
                parMode[e.to] = e.mode;
                pq.push({dist[e.to], e.to});
            }
        }
    }

    if (dist[dst] >= INF/2) {
        cout << "No route found\n";
        return 0;
    }

    
    vector<int> path;
    for (int v = dst; v != -1; v = par[v]) path.push_back(v);
    reverse(path.begin(), path.end());

    cout << "\nTotal distance = " << fixed << setprecision(3) << dist[dst] << " km\n";
    cout << "Path nodes = " << path.size() << "\n";


    cout << "\nText steps:\n";
    for (int i = 0; i + 1 < (int)path.size(); i++) {
        int u = path[i], v = path[i+1];
        string mode = (parMode[v] == 0 ? "CAR" : "WALK");
        double d = haversine(nodePt[u], nodePt[v]);
        cout << i+1 << ") " << mode << " from (" << nodePt[u].lon << "," << nodePt[u].lat << ")"
             << " to (" << nodePt[v].lon << "," << nodePt[v].lat << ")"
             << " dist=" << fixed << setprecision(3) << d << " km\n";
    }

   
    writeKML("route1.kml", path);
    cout << "\nSaved: route1.kml\n";
    cout<<"bello"<<endl;
    return 0;
}