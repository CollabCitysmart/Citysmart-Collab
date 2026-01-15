#include "raylib.h"
#include "raymath.h"
#include "rlgl.h" 
#include <vector>
#include <string>
#include <cstdlib> 
#include <ctime>   
#include <algorithm>
#include <cmath>
#include <map>

// --- YOUR HEADERS ---
#include "GameCommon.h"   
#include "CityMap.h"
#include "Vehicle.h"
#include "EmergencyVehicle.h"
#include "TrafficLight.h"

// --- THEIR HEADER ---
#include "CityGraph.h" 

#define TILE_SIZE 4.0f

class Simulation {
private:
    EmergencyVehicle* ambulance;
    std::vector<Vehicle*> normalTraffic; 
        std::vector<Vector3> loopPath;
std::vector<Vector3> lastPos;
    std::vector<float> stuckTime;
    
    CityMap* cityMap; 
    CityGraph graph;  
    
    bool isMoving;   
    std::vector<Vector3> previewPath;
    bool ambulanceMoving = false; 
    Vector3 hospitalPos;
    bool isReturning;
    std::vector<Vector3> outboundPath;

    std::vector<TrafficLight> lights;
    Vector3 incidentPos{0,0,0};

public:

    // --- Camera follow support helpers ---
    EmergencyVehicle* GetAmbulance() const { return ambulance; }
        std::vector<Vehicle*> GetAllVehicles() const {
        std::vector<Vehicle*> v;
        v.reserve(1 + normalTraffic.size());
        if (ambulance) v.push_back((Vehicle*)ambulance);
        for (auto* car : normalTraffic) v.push_back(car);
        return v;
    }

    size_t GetNormalTrafficCount() const { return normalTraffic.size(); }

    void SpawnNormalVehicle() {
        if (loopPath.empty()) return;

        // Try to pick a start waypoint that is not already occupied (prevents cars spawning on top of each other)
        std::vector<Vector3> p = loopPath;
        int chosenOffset = 0;
        bool found = false;

        for (int attempt = 0; attempt < 12; attempt++) {
            int offset = GetRandomValue(0, (int)p.size() - 1);

            // Temporarily rotate to test this start
            std::vector<Vector3> test = p;
            std::rotate(test.begin(), test.begin() + offset, test.end());
            Vector3 startPos = test[0];

            bool tooClose = false;
            for (auto* car : normalTraffic) {
                if (Vector3Distance(car->getPosition(), startPos) < 3.0f) { tooClose = true; break; }
            }
            if (ambulance && Vector3Distance(ambulance->getPosition(), startPos) < 3.0f) tooClose = true;

            if (!tooClose) { chosenOffset = offset; found = true; break; }
        }

        if (!found) {
            chosenOffset = GetRandomValue(0, (int)p.size() - 1);
        }

        std::rotate(p.begin(), p.begin() + chosenOffset, p.end());

        float spd = 3.5f + (float)GetRandomValue(0, 15) / 10.0f; // 3.5 .. 5.0
        Color cols[] = {BLUE, GREEN, ORANGE, PURPLE, MAROON, DARKBLUE};
        Color c = cols[GetRandomValue(0, (int)(sizeof(cols)/sizeof(cols[0]))-1)];

        Vehicle* v = new Vehicle(p[0], {0,0,0}, spd, c);
        v->setPath(p);
        v->setLooping(true);
        v->setLaneOffset(0.6f);

        normalTraffic.push_back(v);
        lastPos.push_back(v->getPosition());
        stuckTime.push_back(0.0f);
    }

Vector3 GetHospitalPos() const { return hospitalPos; }
    bool AmbulanceAtHospital(float tol = 2.0f) const {
        if (!ambulance) return true;
        return Vector3Distance(ambulance->getPosition(), hospitalPos) <= tol;
    }


    Vector3 Tile(int x, int y) {
        return { (float)x * TILE_SIZE + 2.0f, 0.0f, (float)y * TILE_SIZE + 2.0f };
    }

    Simulation(CityMap* map) {
        cityMap = map;
        srand((unsigned int)time(NULL));
        
        InitGraphNetwork();

        hospitalPos = Tile(8, 6); 
        for (int y = 0; y < CityMap::H; y++) {
            for (int x = 0; x < CityMap::W; x++) {
                if (cityMap->facilityMap[y][x] == HOSPITAL) {
                    hospitalPos = cityMap->tileCenter(y, x);
                    break;
                }
            }
        }
        hospitalPos.y = 0.0f; 

        isReturning = false;
        
        ambulance = new EmergencyVehicle(hospitalPos, {0,0,0}, 5.0f, RED, "Ambulance");
        ambulance->toggleSiren(true);
        ambulance->setLaneOffset(0.2f); 
        ambulance->setLooping(false);

        InitGraphTraffic(); 
        InitTrafficLights();

        for (size_t i = 0; i < normalTraffic.size(); i++) {
            normalTraffic[i]->setLaneOffset(0.6f); 
        }
        
        lastPos.clear();
        stuckTime.clear();
        for (auto v : normalTraffic) { 
            lastPos.push_back(v->getPosition()); 
            stuckTime.push_back(0.0f); 
        }
        isMoving = false; 
    }

    
    // Reset the whole simulation back to the initial state (cars, ambulance, lights, paths).
    void ResetAll() {
        // Cleanup existing vehicles
        if (ambulance) { delete ambulance; ambulance = nullptr; }
        for (auto v : normalTraffic) delete v;
        normalTraffic.clear();

        previewPath.clear();
        outboundPath.clear();
        incidentPos = {0,0,0};

        isMoving = false;
        ambulanceMoving = false;
        isReturning = false;

        // Recreate ambulance at the hospital
        ambulance = new EmergencyVehicle(hospitalPos, {0,0,0}, 5.0f, RED, "Ambulance");
        ambulance->toggleSiren(true);
        ambulance->setLaneOffset(0.2f);
        ambulance->setLooping(false);

        // Recreate traffic + lights
        InitGraphTraffic();
        InitTrafficLights();

        for (auto v : normalTraffic) v->setLaneOffset(0.6f);

        // Reset stuck tracking arrays
        lastPos.clear();
        stuckTime.clear();
        for (auto v : normalTraffic) {
            lastPos.push_back(v->getPosition());
            stuckTime.push_back(0.0f);
        }
    }

~Simulation() { 
        delete ambulance; 
        for(auto v : normalTraffic) delete v;
    }

    void InitGraphNetwork() {
        graph.nodes.clear();
        float roundaboutOffset = 1.3f; 

        // 1. CRÉATION DES NŒUDS
        for (int y = 0; y < CityMap::H; y++) {
            for (int x = 0; x < CityMap::W; x++) {
                
                int t = cityMap->tileMap[y][x];

                bool isRoad = (t == ROAD_H || t == ROAD_V || t == PROAD_V || t == PROAD_V1 ||
                               t == INTERSECTION || t == CURVERTOP || t == CURVERTOP1 || 
                               t == CURVERBOTTOM || t == CURVERBOTTOM1 || t == FCURVERBOTTOM ||
                               t == TROAD || t == TROAD1 || t == ROTROAD);
                
                bool isRoundabout = (t == ROUNDABOUT); 

                std::string id = std::to_string(x) + "_" + std::to_string(y);
                Vector3 center = Tile(x, y);
                center.y = 0.0f;

                if (isRoundabout) {
                    // 4 Nœuds pour le rond-point
                    graph.AddNode(id + "_N", {center.x, 0.0f, center.z - roundaboutOffset}, CIRCULAR);
                    graph.AddNode(id + "_S", {center.x, 0.0f, center.z + roundaboutOffset}, CIRCULAR);
                    graph.AddNode(id + "_E", {center.x + roundaboutOffset, 0.0f, center.z}, CIRCULAR);
                    graph.AddNode(id + "_W", {center.x - roundaboutOffset, 0.0f, center.z}, CIRCULAR);

                    // Connexions internes (Sens giratoire)
                    graph.Connect(id + "_N", id + "_W"); 
                    graph.Connect(id + "_W", id + "_S"); 
                    graph.Connect(id + "_S", id + "_E"); 
                    graph.Connect(id + "_E", id + "_N"); 

                } else if (isRoad) {
                    // 1 Nœud pour route normale
                    NodeType type = DIRECT;
                    if (t == INTERSECTION || t == TROAD || t == TROAD1 || t == ROTROAD) {
                        type = CIRCULAR;
                    }
                    graph.AddNode(id, center, type);
                }
            }
        }

        // 2. CONNEXIONS ENTRE TUILES
        for (int y = 0; y < CityMap::H; y++) {
            for (int x = 0; x < CityMap::W; x++) {
                std::string myId = std::to_string(x) + "_" + std::to_string(y);

                if (graph.nodes.count(myId)) {
                    
                    // NORD
                    std::string nId = std::to_string(x) + "_" + std::to_string(y-1);
                    if (graph.nodes.count(nId)) { graph.Connect(myId, nId); } 
                    else if (graph.nodes.count(nId + "_S")) { 
                        graph.Connect(myId, nId + "_S"); 
                        graph.Connect(nId + "_S", myId);
                    }

                    // SUD
                    std::string sId = std::to_string(x) + "_" + std::to_string(y+1);
                    if (graph.nodes.count(sId)) { graph.Connect(myId, sId); }
                    else if (graph.nodes.count(sId + "_N")) { 
                        graph.Connect(myId, sId + "_N"); 
                        graph.Connect(sId + "_N", myId);
                    }

                    // EST
                    std::string eId = std::to_string(x+1) + "_" + std::to_string(y);
                    if (graph.nodes.count(eId)) { graph.Connect(myId, eId); }
                    else if (graph.nodes.count(eId + "_W")) { 
                        graph.Connect(myId, eId + "_W"); 
                        graph.Connect(eId + "_W", myId);
                    }

                    // OUEST
                    std::string wId = std::to_string(x-1) + "_" + std::to_string(y);
                    if (graph.nodes.count(wId)) { graph.Connect(myId, wId); }
                    else if (graph.nodes.count(wId + "_E")) { 
                        graph.Connect(myId, wId + "_E"); 
                        graph.Connect(wId + "_E", myId);
                    }
                }
            }
        }
    }
    
    std::vector<Vector3> RefinePath(const std::vector<Vector3>& coarsePath) {
        if (coarsePath.size() < 2) return coarsePath;
        
        std::vector<Vector3> refined;
        
        for (size_t i = 0; i < coarsePath.size() - 1; i++) {
            Vector3 start = coarsePath[i];
            Vector3 end = coarsePath[i+1];
            
            if (Vector3Distance(start, end) < 4.5f) {
                 if (refined.empty()) refined.push_back(start);
                 refined.push_back(end);
            } else {
                std::vector<Vector3> segment = cityMap->findFastestPath(start, end, false);
                for(auto& p : segment) p.y = 0.0f; 

                if (refined.empty()) {
                    refined.insert(refined.end(), segment.begin(), segment.end());
                } else {
                    if (segment.size() > 1) {
                        refined.insert(refined.end(), segment.begin() + 1, segment.end());
                    }
                }
            }
        }
        return refined;
    }

   void InitGraphTraffic() {
        normalTraffic.clear();

        std::vector<Vector3> loop;
        auto append = [&](std::string a, std::string b) {
            std::vector<Vector3> p = graph.GetShortestPath(a, b);
            if (!loop.empty() && !p.empty()) p.erase(p.begin());
            loop.insert(loop.end(), p.begin(), p.end());
        };

        append("4_2", "6_2");
        append("6_2", "6_9");
        append("6_9", "4_9");
        append("4_9", "4_2");
        
        for(auto& p : loop) p.y = 0.0f;

        loopPath = loop;
        if (!loop.empty()) {
            Vehicle* v1 = new Vehicle(loop[0], {0,0,0}, 4.0f, BLUE);
            v1->setPath(loop);
            v1->setLooping(true);
            normalTraffic.push_back(v1);

            int half = (int)loop.size() / 2;
            if (loop.size() > half) {
                std::vector<Vector3> p2 = loop;
                std::rotate(p2.begin(), p2.begin() + half, p2.end());
                Vehicle* v2 = new Vehicle(p2[0], {0,0,0}, 4.2f, GREEN);
                v2->setPath(p2);
                v2->setLooping(true);
                normalTraffic.push_back(v2);
            }
        }
    }

    void InitTrafficLights() {
        lights.clear();
        for (int y = 0; y < CityMap::H; y++) {
            for (int x = 0; x < CityMap::W; x++) {
                int t = cityMap->tileMap[y][x];
                if (t == INTERSECTION || t == TROAD || t == TROAD1 || t == ROUNDABOUT || t == ROTROAD) {
                    Vector3 p = cityMap->tileCenter(y, x);
                    p.y = 0.0f; 
                    lights.emplace_back(p, 3.0f); 
                }
            }
        }
    }

    std::string FindClosestNode(Vector3 pos) {
        std::string bestID = "";
        float minDst = 10000.0f;
        for (auto const& [id, node] : graph.nodes) {
            float dst = Vector3Distance(pos, node.position);
            if (dst < minDst) {
                minDst = dst;
                bestID = id;
            }
        }
        return bestID;
    }

    void Update(Camera3D camera) {
        float dt = GetFrameTime();
        
        for (auto &l : lights) {
            l.update(dt);
            if (ambulanceMoving && ambulance->getSirenActive()) {
                l.greenForEmergencyIfApproaching(ambulance->getPosition(), ambulance->getForwardDir());
            }
        }

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            Ray ray = GetMouseRay(GetMousePosition(), camera);
            std::string targetNodeID = "";

            targetNodeID = graph.GetNodeClicked(ray);
            
            if (targetNodeID.empty()) {
                RayCollision collision = GetRayCollisionQuad(ray, {0,0,0}, {0,0,40}, {40,0,40}, {40,0,0});
                if (collision.hit) {
                    targetNodeID = FindClosestNode(collision.point);
                }
            }

            if (!targetNodeID.empty()) {
                std::string startID = FindClosestNode(ambulance->getPosition());
                std::vector<Vector3> rawPath = graph.GetShortestPath(startID, targetNodeID);
                
                if (!rawPath.empty()) {
                    previewPath = RefinePath(rawPath);
                    ambulanceMoving = false; 
                    isReturning = false; 
                    printf("Path found: %s -> %s\n", startID.c_str(), targetNodeID.c_str());
                }
            }
        }

        if (IsKeyPressed(KEY_G) && !previewPath.empty()) {
            outboundPath = previewPath;
            ambulance->setPath(previewPath);
            ambulanceMoving = true;
            isReturning = false;
            if (!outboundPath.empty()) incidentPos = outboundPath.back();
            previewPath.clear();
        }

        if (IsKeyPressed(KEY_SPACE)) isMoving = !isMoving;

        if (isMoving) {
            if (ambulanceMoving) {
                ambulance->step(dt, true);
                if (!cityMap->isDriveableWorld(ambulance->getPosition())) {
                    Vector3 corrected = cityMap->clampToDriveable(ambulance->getPosition());
                    corrected.y = 0.0f; 
                    ambulance->setPosition(corrected);
                }
            }

            for (size_t i = 0; i < normalTraffic.size(); i++) {
                Vehicle* v = normalTraffic[i];
                Vector3 fwd = v->getIntendedDir();
                
                bool stopForRed = false;
                for (const auto& l : lights) {
                    if (l.shouldStop(v->getPosition(), fwd)) { stopForRed = true; break; }
                }

                if (ambulanceMoving) {
                      Vector3 ambDir = ambulance->getIntendedDir();
                      Vector3 rel = Vector3Subtract(v->getPosition(), ambulance->getPosition());
                      float dist = Vector3Length(rel);
                      float ahead = Vector3DotProduct(rel, ambDir);
                      
                      if (dist < 25.0f && ahead > -4.0f) {
                          v->yieldTo(ambulance->getPosition(), ambDir, 10.0f, 30.0f);
                          if (fabsf(v->getLaneOffset()) > 1.3f) v->setSpeedScale(0.0f, 0.1f);
                          else v->setSpeedScale(0.9f, 0.1f);
                      }
                }

                

                // --- Car-car spacing (prevent stacking / overlap) ---
                // If there is another vehicle directly ahead in the same lane, slow down or stop.
                float followScale = 1.0f;

                Vector3 myFwd = fwd;
                if (Vector3Length(myFwd) > 0.001f) myFwd = Vector3Normalize(myFwd);

                auto considerAhead = [&](Vehicle* other) {
                    if (!other || other == v) return;
                    Vector3 rel = Vector3Subtract(other->getPosition(), v->getPosition());
                    rel.y = 0.0f;

                    float ahead = Vector3DotProduct(rel, myFwd);
                    if (ahead <= 0.0f) return; // not ahead

                    // Lateral distance from our forward axis
                    Vector3 proj = Vector3Scale(myFwd, ahead);
                    Vector3 latV = Vector3Subtract(rel, proj);
                    float lateral = Vector3Length(latV);

                    // Same-lane-ish check (tweak if your lanes are wider/narrower)
                    if (lateral > 1.2f) return;

                    // Safe distance: base + extra proportional to speed (simple time headway)
                    float safeDist = 3.0f + v->getSpeed() * 0.7f;   // ~3m + 0.7s headway
                    float stopDist = 1.8f;                          // hard stop distance

                    if (ahead < stopDist) {
                        followScale = std::min(followScale, 0.0f);
                    } else if (ahead < safeDist) {
                        float t = (ahead - stopDist) / (safeDist - stopDist); // 0..1
                        followScale = std::min(followScale, Clamp(t, 0.0f, 1.0f));
                    }
                };

                for (auto* other : normalTraffic) considerAhead(other);
                if (ambulance) considerAhead((Vehicle*)ambulance);

                if (followScale < 1.0f) v->setSpeedScale(followScale, 0.12f);

// --- Anti-deadlock / anti-traffic-jam (unstuck) ---
// Track if this vehicle is not actually moving (position barely changes) for too long.
float moved = Vector3Distance(v->getPosition(), lastPos[i]);
bool isBlocked = stopForRed || (followScale < 0.15f);
if (moved < 0.01f && isBlocked && !v->hasFinishedPath()) {
    stuckTime[i] += dt;
} else {
    stuckTime[i] = 0.0f;
}
lastPos[i] = v->getPosition();

// If stuck for a while, try a gentle lane nudge so one car can slip around instead of locking the whole road.
if (stuckTime[i] > 2.0f) {
    float sign = (i % 2 == 0) ? 1.0f : -1.0f;
    v->nudgeLaneOffset(sign * 1.1f, 1.2f);
    v->setSpeedScale(0.55f, 0.25f); // creep forward a bit if space opens
}

v->step(dt, !stopForRed);
                
                if (!cityMap->isDriveableWorld(v->getPosition())) {
                    Vector3 corrected = cityMap->clampToDriveable(v->getPosition());
                    corrected.y = 0.0f;
                    v->setPosition(corrected);
                }
            }
        }

        // --- Physical separation pass (prevents two cars staying inter-penetrated and blocking forever) ---
        const float minSep = 1.2f; // approx car footprint radius
        for (size_t a = 0; a < normalTraffic.size(); a++) {
            for (size_t b = a + 1; b < normalTraffic.size(); b++) {
                Vector3 pa = normalTraffic[a]->getPosition();
                Vector3 pb = normalTraffic[b]->getPosition();
                Vector3 d = Vector3Subtract(pb, pa);
                d.y = 0.0f;
                float dist = Vector3Length(d);
                if (dist < 0.001f) dist = 0.001f;
                if (dist < minSep) {
                    Vector3 dir = Vector3Scale(d, 1.0f / dist);
                    float push = (minSep - dist) * 0.5f;
                    Vector3 newA = Vector3Subtract(pa, Vector3Scale(dir, push));
                    Vector3 newB = Vector3Add(pb, Vector3Scale(dir, push));
                    newA.y = 0.0f; newB.y = 0.0f;

                    // Keep them on drivable surface
                    if (!cityMap->isDriveableWorld(newA)) newA = cityMap->clampToDriveable(newA);
                    if (!cityMap->isDriveableWorld(newB)) newB = cityMap->clampToDriveable(newB);
                    newA.y = 0.0f; newB.y = 0.0f;

                    normalTraffic[a]->setPosition(newA);
                    normalTraffic[b]->setPosition(newB);
                }
            }
        }
    }

    void Draw() {
        if (ambulance) ambulance->draw();
        for (auto v : normalTraffic) v->draw();
        for (const auto& l : lights) l.draw();
        
        rlDisableDepthTest(); 

        for (auto const& [id, node] : graph.nodes) {
            Vector3 drawPos = node.position;
            drawPos.y = 0.0f; 

            Color nodeColor = (node.type == CIRCULAR) ? BLUE : GREEN;
            
            // --- MODIFICATION ICI : Taille 0.2f (petit) ---
            DrawSphere({drawPos.x, 0.1f, drawPos.z}, 0.2f, nodeColor);
            
            for (const auto& neighborID : node.neighbors) {
                if (graph.nodes.find(neighborID) != graph.nodes.end()) {
                    Vector3 neighborPos = graph.nodes.at(neighborID).position;
                    DrawLine3D(
                        {drawPos.x, 0.05f, drawPos.z}, 
                        {neighborPos.x, 0.05f, neighborPos.z}, 
                        YELLOW
                    );
                }
            }
        }
        
        rlEnableDepthTest(); 

        if (!previewPath.empty() && !ambulanceMoving) {
            Color flashColor = ((int)(GetTime() * 5) % 2 == 0) ? RED : BLUE;
            for (size_t i = 0; i < previewPath.size() - 1; i++) {
                DrawLine3D(
                    Vector3Add(previewPath[i], {0, 0.15f, 0}), 
                    Vector3Add(previewPath[i+1], {0, 0.15f, 0}), 
                    flashColor
                );
            }
        }
    }
    
    // J'ai vidé cette fonction pour ne plus afficher de texte
    void Draw2D(Camera3D camera) {
        // graph.DrawLabels(camera); // COMMENTÉ = PLUS DE TEXTE
    }
};

int main() {
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    int w = (int)(GetMonitorWidth(0) * 0.85f);
    int h = (int)(GetMonitorHeight(0) * 0.85f);
    if (w < 1000) w = 1000;
    if (h < 700) h = 700;
    InitWindow(w, h, "Merged Logic: Graph + Road Adherence");
    SetWindowMinSize(900, 600);
    SetTargetFPS(60);
    
    CityMap city; 
    Simulation sim(&city);

    Camera3D cam = {0};
    cam.target = {20, 0, 20}; 
    cam.up = {0, 1, 0};
    cam.fovy = 45;
    cam.projection = CAMERA_PERSPECTIVE;

    float camAngle = 45.0f;
    float camRadius = 80.0f;
    float camHeight = 40.0f;


    // Follow-camera (toggle with C) for the ambulance until it reaches the hospital
    bool followCamera = false;
    bool followHasLeftHospital = false;
    const float followBack = 18.0f;   // distance behind the ambulance (closer)
    const float followUp   = 10.0f;   // height above the ambulance (closer)
    int followIndex = 0; // 0 = ambulance, then normal cars
    const Vector3 mapCenter = {20, 0, 20};
    // Press R to reset camera to center overview
    while (!WindowShouldClose()) {   
        float rotationSpeed = 60.0f * GetFrameTime();
        float zoomSpeed = 40.0f * GetFrameTime();


        if (IsKeyPressed(KEY_C)) {
            followCamera = !followCamera;
            followHasLeftHospital = false;
        }

        if (IsKeyPressed(KEY_R)) {
            followCamera = false;
            followHasLeftHospital = false;
            camAngle = 45.0f;
            camRadius = 80.0f;
            camHeight = 40.0f;
            cam.target = mapCenter;
        }

        if (IsKeyPressed(KEY_TAB)) {
            // cycle follow target (ambulance + normal cars)
            int total = (int)sim.GetAllVehicles().size();
            if (total > 0) {
                followIndex = (followIndex + 1) % total;
                // If we are following, switching target should restart the hospital leave/return logic
                followHasLeftHospital = false;
            }
        }
        if (!followCamera) {
            if (IsKeyDown(KEY_A)) camAngle -= rotationSpeed;
            if (IsKeyDown(KEY_D)) camAngle += rotationSpeed;
            if (IsKeyDown(KEY_W)) camRadius -= zoomSpeed; 
            if (IsKeyDown(KEY_S)) camRadius += zoomSpeed;
        }
        
        camRadius = Clamp(camRadius, 15.0f, 150.0f);

        // Update camera: either free orbit or follow the ambulance
        if (followCamera) {
            auto all = sim.GetAllVehicles();
            if (!all.empty()) {
                if (followIndex < 0) followIndex = 0;
                if (followIndex >= (int)all.size()) followIndex = 0;
                Vehicle* tgt = all[followIndex];
                if (!tgt) { followCamera = false; }
                Vector3 ambPos = tgt->getPosition();
                Vector3 fwd = tgt->getForwardDir();
float len = Vector3Length(fwd);
                if (len < 0.001f) fwd = { 1.0f, 0.0f, 0.0f };
                else fwd = Vector3Scale(fwd, 1.0f / len);

                cam.target = ambPos;
                cam.position = Vector3Add(Vector3Add(ambPos, {0.0f, followUp, 0.0f}),
                                          Vector3Scale(fwd, -followBack));

                // Prevent the camera from going inside buildings while following
                Vector3 desiredPos = cam.position;
                Vector3 toCam = Vector3Subtract(desiredPos, ambPos);
                float distToCam = Vector3Length(toCam);
                if (distToCam > 0.01f) {
                    Ray ray;
                    ray.position = ambPos;
                    ray.direction = Vector3Scale(toCam, 1.0f / distToCam);

                    float nearestHit = distToCam;
                    bool hitSomething = false;
                    const float margin = 0.8f; // keep a small gap from walls

                    for (const Building& b : city.GetBuildings()) {
                        float halfW = b.width * 0.5f;
                        float halfD = b.depth * 0.5f;
                        BoundingBox box;
                        box.min = { b.pos.x - halfW, b.pos.y, b.pos.z - halfD };
                        box.max = { b.pos.x + halfW, b.pos.y + b.height, b.pos.z + halfD };

                        RayCollision col = GetRayCollisionBox(ray, box);
                        if (col.hit && col.distance < nearestHit) {
                            nearestHit = col.distance;
                            hitSomething = true;
                        }
                    }

                    if (hitSomething) {
                        float safeDist = nearestHit - margin;
                        if (safeDist < 2.0f) safeDist = 2.0f; // don't get too close to the car
                        cam.position = Vector3Add(ambPos, Vector3Scale(ray.direction, safeDist));
                    }
                }


                // Auto-disable follow once the ambulance has left and then returned to the hospital (only when following the ambulance)
                if (followIndex == 0) {
                float dHosp = Vector3Distance(ambPos, sim.GetHospitalPos());
                if (dHosp > 2.0f) followHasLeftHospital = true;
                if (followHasLeftHospital && dHosp <= 2.0f) {
                    followCamera = false;
                    followHasLeftHospital = false;
                }
                }
            } else {
                followCamera = false;
                followHasLeftHospital = false;
            }
        }

        if (!followCamera) {
            cam.position.x = cam.target.x + cosf(camAngle * DEG2RAD) * camRadius;
            cam.position.z = cam.target.z + sinf(camAngle * DEG2RAD) * camRadius;
            cam.position.y = camHeight;
        }

        sim.Update(cam);

        BeginDrawing();
            ClearBackground(SKYBLUE);
            BeginMode3D(cam);
                city.draw(); 
                sim.Draw();
            EndMode3D();

            // --- Simple UI: Spawn Car button (top-right) ---
            const int btnW = 160, btnH = 34, pad = 10;
            Rectangle btn = { (float)(GetScreenWidth() - btnW - pad), (float)pad, (float)btnW, (float)btnH };
            Vector2 mp = GetMousePosition();
            bool hover = CheckCollisionPointRec(mp, btn);
            DrawRectangleRec(btn, hover ? Fade(DARKGRAY, 0.35f) : Fade(DARKGRAY, 0.25f));
            DrawRectangleLinesEx(btn, 2, DARKGRAY);
            DrawText("Spawn Car", (int)btn.x + 18, (int)btn.y + 8, 18, DARKGREEN);
            if (hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                sim.SpawnNormalVehicle();
            }

            
            // --- Simple UI: Reset button (top-right, under Spawn) ---
            Rectangle resetBtn = { (float)GetScreenWidth() - btnW - pad, btn.y + btnH + 10.0f, (float)btnW, (float)btnH };
            bool hoverReset = CheckCollisionPointRec(mp, resetBtn);
            DrawRectangleRec(resetBtn, hoverReset ? Fade(ORANGE, 0.25f) : Fade(ORANGE, 0.15f));
            DrawRectangleLinesEx(resetBtn, 2, ORANGE);
            DrawText("Reset", (int)resetBtn.x + 52, (int)resetBtn.y + 8, 18, ORANGE);

            if (hoverReset && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                sim.ResetAll();
                followCamera = false;
                followIndex = -1; // back to default center view
            }

// --- Camera info ---
            DrawText(TextFormat("Cars: %d | Follow target: %s (TAB to change) | R: Reset", (int)sim.GetNormalTrafficCount(),
                                (followCamera ? (followIndex==0 ? "Ambulance" : "Car") : "None")),
                     10, 40, 18, DARKGREEN);

            // sim.Draw2D(cam); // COMMENTÉ DANS LE MAIN AUSSI
            
            DrawText("L-Click Red/Green Node to Path | G: Go | C: Toggle Follow | TAB: Change Target | R: Reset Camera", 10, 10, 20, DARKGREEN);
        EndDrawing();
    } 
    CloseWindow();
    return 0;
}