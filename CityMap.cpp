#include "CityMap.h"
#include <queue>
#include <map>
#include <algorithm>
#include "rlgl.h"
#include "CityMap.h"
#include <queue>
#include <map>
#include <algorithm>
#include <filesystem>
#include <unordered_map>
#include <string>
#include <cctype>

CityMap::CityMap() {
    initMaps();
    loadTextures();
    buildFacilities();
    cube = LoadModelFromMesh(GenMeshCube(1, 1, 1));
}

CityMap::~CityMap() { UnloadModel(cube); }

void CityMap::initMaps() {
    int tm[10][10] = {
        {PP2,PG3,G,G,G,G,V,G,G,G}, {PP1,G,CT,H,CB1,PP2,V,G,CT,PV1},
        {PV,H,RTT,G,TR,PV,I,H,CB,PP1H}, {G,G,V,G,V,G,V,G,G,PP2H},
        {G,G,V,G,V,G,V,G,G,G}, {H,H,TR1,H,R,H,I,H,CB1,G},
        {G,G,G,G,V,G,V,G,V,G}, {CT,H,H,H,I,H,CB,G,CT1,H},
        {V,G,G,G,V,PP2,G,G,G,PP2}, {V,G,G,G,TR,PV,H,PV,PV,PV}
    };
    FacilityType fm[10][10] = {
        {N,N,S,S1,S2,BKS,N,N,MM,BKE}, {N,GSC,N,N,N,N,N,N,N,N},
        {N,N,N,N,N,N,N,N,N,N}, {N,PS1,N,N,N,B8,N,N,N,N},
        {PS2,PS,N,BQ,N,B9,N,PM,HP3,HP2}, {N,N,N,N,N,N,N,N,N,HP1},
        {REST,N,BC,OC,N,CS,N,N,N,HP}, {N,N,N,N,N,N,N,N,N,N},
        {N,B1,N,B1,N,N,F2,F1,F3,N}, {N,B71,N,B72,N,N,N,N,N,N}
    };
    for(int y=0; y<10; y++) {
        for(int x=0; x<10; x++) {
            tileMap[y][x] = tm[y][x];
            facilityMap[y][x] = fm[y][x];
        }
    }
}

void CityMap::loadTextures() {
    namespace fs = std::filesystem;

    
    std::string assetsDir = "assets";
    if (!fs::exists(assetsDir) && fs::exists("../assets")) assetsDir = "../assets";

    // Case-insensitive lookup: Windows is case-insensitive, macOS/Linux often aren't
    auto toLower = [](std::string s) {
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c){ return (char)std::tolower(c); });
        return s;
    };

    std::unordered_map<std::string, std::string> lut;
    if (fs::exists(assetsDir)) {
        for (const auto& entry : fs::directory_iterator(assetsDir)) {
            if (!entry.is_regular_file()) continue;
            std::string name = entry.path().filename().string();
            lut[toLower(name)] = name;
        }
    }

    auto LoadTex = [&](const char* file) -> Texture2D {
        std::string req(file);
        std::string key = toLower(req);
        std::string actual = lut.count(key) ? lut[key] : req;
        std::string fullPath = assetsDir + "/" + actual;

        if (FileExists(fullPath.c_str())) return LoadTexture(fullPath.c_str());

        // Fallback texture for missing files (prevents invisible/black geometry)
        Image img = GenImageColor(1, 1, LIGHTGRAY);
        Texture2D t = LoadTextureFromImage(img);
        UnloadImage(img);
        return t;
    };

    texG = LoadTex("grass.png"); texRH = LoadTex("road_h.png"); texRV = LoadTex("road_v.png");
    texPR = LoadTex("proad_v.png"); texPR1 = LoadTex("proad_v.png"); texINT = LoadTex("intersection.png");
    texCVT = LoadTex("cvroad_t.png"); texCVT1 = LoadTex("cvroad_t1.png"); texCVB = LoadTex("cvroad_b.png");
    texFCVB = LoadTex("f2cvb.png"); texCVB1 = LoadTex("cvroad_b1.png"); texTRD = LoadTex("t_road.png");
    texTRD1 = LoadTex("t_road1.png"); texROT = LoadTex("rott_road.png"); texPK = LoadTex("parking.png");
    texPK1 = LoadTex("pparking_1.png"); texPK1h = LoadTex("pparking_1.png"); texPK2 = LoadTex("pparking_2.png");
    texPK2h = LoadTex("pparking_2.png"); texRB = LoadTex("roundabout.png"); texPY3 = LoadTex("playground_3.png");

    // Buildings
    texFREST = LoadTex("f_restaurant.png"); texRBREST = LoadTex("rb_restaurant.png");
    texFBCOF = LoadTex("f_bcoffe.png"); texRBBCOF = LoadTex("rb_bcoffe.png");
    texFHT = LoadTex("f1_hospital.png"); texHT = LoadTex("f1_hospital.png"); texHT1 = LoadTex("f2_hospital.png");
    texHTS1 = LoadTex("rs_hospital.png"); texHTS2 = LoadTex("rs_hospital.png"); texPHM = LoadTex("f_pharmacy.png"); texPHMR = LoadTex("r_pharmacy.png");
    texBF1 = LoadTex("b1_f1.png"); texBF2 = LoadTex("b1_f2.png");         texBS1 = LoadTex("b1_s1.png"); texBS2 = LoadTex("b1_s2.png");
    texBS3 = LoadTex("b1_s3.png"); texBS4 = LoadTex("b1_s4.png");         texBR = LoadTex("b1_r.png"); texB2F = LoadTex("b2_f.png");
    texB2F2 = LoadTex("b2_f2.png"); texB2R = LoadTex("b2_r.png");         texB3F = LoadTex("b3_f.png"); texB3R = LoadTex("b3_r.png");
    texB4F = LoadTex("b4_f.png");   texB4R = LoadTex("b4_r.png");         texB5F = LoadTex("b5_f.png"); texB5F1 = LoadTex("b5_f1.png");
    texB5F2 = LoadTex("b5_f2.png"); texB5S = LoadTex("b5_s.png");         texB5S1 = LoadTex("b5_s1.png"); texB5R = LoadTex("b5_r.png");
    texB6F1 = LoadTex("b6_f1.png"); texB6F3 = LoadTex("b6_f3.png");       texB6R = LoadTex("b6_r.png"); texMMF = LoadTex("minim_f.png"); texMMS = LoadTex("minim_s.png");
    texB7F = LoadTex("b6_f1.png");   texB7F1 = LoadTex("b6_f1.png");       texB7F2 = LoadTex("b6_f3.png"); texB7S = LoadTex("b6_f1.png");
    texB7S1 = LoadTex("b6_f3.png"); texB7R = LoadTex("b6_r.png");         texB8F = LoadTex("b5_f.png"); texB8F2 = LoadTex("b5_f2.png");
    texB8S = LoadTex("b5_s.png");   texB8S2 = LoadTex("b5_s1.png");       texB8R = LoadTex("b5_r.png"); texB9F = LoadTex("b2_f.png");
    texB9F1 = LoadTex("b2_f2.png"); texB9S = LoadTex("b2_f.png");         texB9S1 = LoadTex("b2_f2.png"); texB9R = LoadTex("b2_r.png");
    texFS1 = LoadTex("f1_fire.png");texFS2 = LoadTex("f2_fire.png");      texFS3 = LoadTex("f3_fire.png"); texFRS = LoadTex("rs_fire.png");
    texFS = LoadTex("s_fire.png");    texBF = LoadTex("b_fire.png");      texFCS = LoadTex("f_coffes.png"); texF1CS = LoadTex("f1_coffes.png");
    texCSS = LoadTex("s_coffes.png"); texCS1 = LoadTex("s1_coffes.png");  texRCS = LoadTex("rs_coffes.png"); texBK = LoadTex("bakery.png"); texRBY = LoadTex("r_bakery.png");
    texRP = LoadTex("r_police.png");  texFP = LoadTex("f_police.png");    texSP = LoadTex("s_police.png"); texS1P = LoadTex("s1_police.png");
    texFSL = LoadTex("b3_f.png"); texF1SL = LoadTex("b3_f.png"); texFSLS = LoadTex("b3_f.png"); texFSLR = LoadTex("b3_r.png");
    texFGSC = LoadTex("b4_f.png");   texSGSC = LoadTex("b4_f.png");     texRGSC = LoadTex("b4_r.png"); texFBKS = LoadTex("b5_f.png");
    texSBKS = LoadTex("b5_f.png"); texRBKS = LoadTex("b5_r.png"); texFBTQ = LoadTex("b2_f2.png"); texSBTQ = LoadTex("b2_f2.png"); texRBTQ = LoadTex("b2_r.png");

}



void CityMap::DrawTile(Texture2D texture, Vector3 pos, float rotationY) {
    cube.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
    Matrix scale = MatrixScale(TILE, 0.11f, TILE);
    Matrix rot = MatrixRotateY(rotationY * DEG2RAD);
    Matrix trans = MatrixTranslate(pos.x, pos.y, pos.z);
    cube.transform = MatrixMultiply(scale, MatrixMultiply(rot, trans));
    DrawModel(cube, Vector3Zero(), 1.0f, WHITE);
}


// ... CityMap constructor and loadTextures (Keep your existing ones) ...

void CityMap::draw() {
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            float offset = TILE / 2.0f;
            Vector3 pos = { (float)x * TILE + offset, 0, (float)y * TILE + offset };
            DrawTile(texG, pos, 0); 

            switch (tileMap[y][x]) {
                case ROAD_H: DrawTile(texRH, pos, 0); break;
                case ROAD_V: DrawTile(texRV, pos, 0); break;
                case PROAD_V: DrawTile(texPR, pos, 90); break;
                case PROAD_V1: DrawTile(texPR1, pos, -90); break;
                case INTERSECTION: DrawTile(texINT, pos, 0); break;
                case ROUNDABOUT: DrawTile(texRB, pos, 0); break;
                case CURVERTOP: DrawTile(texCVT, pos, 0); break;
                case CURVERTOP1: DrawTile(texCVT1, pos, 0); break;
                case CURVERBOTTOM: DrawTile(texCVB, pos, 180); break;
                case CURVERBOTTOM1: DrawTile(texCVB1, pos, 0); break;
                case TROAD: DrawTile(texTRD, pos, 0); break;
                case TROAD1: DrawTile(texTRD1, pos, 180); break;
                case ROTROAD: DrawTile(texROT, pos, 0); break;
                case PPARKING1: DrawTile(texPK1, pos, 90); break;
                case PPARKING2: DrawTile(texPK2, pos, 90); break;
                case PLAYGROUND3: DrawTile(texPY3, pos, 180); break;
                default: break;
            }
            
        }
    }
    rlDrawRenderBatchActive();

    // Use rlgl to ensure textures don't flicker between tiles and buildings


    
    for (const Building &b : buildings) {
        
        switch (b.type) {
            case RESTAURANT: 
                DrawTallBuilding(b, 2, 3.0f, texFREST, texRBREST, texB3F, texFREST, texB3F, texB3F, texB3F, texB3F, texB3R); 
                break;   
            case BCOFFEE: 
                DrawTallBuilding(b, 3, 3.0f, texRBBCOF, texFBCOF, texFBCOF, texB4F, texB4F, texB4F, texB4F, texB4F, texB4R); 
                break;
            case HOSPITAL: 
                DrawTallBuilding(b, 2, 3.0f, texHT, texHTS2, texHTS1, texHTS1, texHTS2, texHTS2, texHTS2, texHTS2, texPHMR); 
                break;
            case HOSPITAL1: 
                DrawTallBuilding(b, 2, 3.0f, texFHT, texHTS2, texHTS1, texHTS1, texHTS2, texHTS2, texHTS2, texHTS2, texPHMR); 
                break;
            case HOSPITAL2: 
                DrawTallBuilding(b, 2, 3.0f, texHT1, texHTS1, texFHT, texFHT, texHTS2, texHTS2, texHTS2, texHTS2, texPHMR); 
                break;
            case HOSPITAL3: 
                DrawTallBuilding(b, 2, 3.0f, texHTS2, texPHMR, texPHMR, texFHT, texHTS2, texHTS2, texHTS2, texHTS2, texPHMR); 
                break;
            case PHARMACY: 
                DrawTallBuilding(b, 2, 3.0f, texHTS2, texPHM, texHT1, texPHM, texHTS2, texHTS2, texHTS2, texHTS2, texPHMR); 
                break; 
            case BUILDING_b1: 
                DrawTallBuilding(b, 3, 3.0f, texBF1, texBS2, texBS2, texBS4, texBF2, texBS1, texBS1, texBS3, texBR); 
                break;
            case FIRES: 
                DrawBoxBuilding(b, texFS1, texFRS, texFRS, texBF, texFRS); 
                break;
            case FIRES1: 
                DrawTallBuilding(b, 2, 3.0f, texFS3, texFS, texFS, texFS1, texFS, texFS, texFS, texFS, texFRS); 
                break;    
            case FIRES2: 
                DrawTallBuilding(b, 2, 3.0f, texBF, texFS, texFS, texFS2, texFS, texFS, texFS, texFS, texFRS); 
                break; 
            case COFFE_S: 
                DrawTallBuilding(b, 2, 3.0f, texFCS, texCSS, texCSS, texFCS, texF1CS, texCSS, texCSS, texCS1, texRCS); 
                break;
            case BAKERY: 
                DrawTallBuilding(b, 3, 3.0f, texBK, texRBY, texBK, texB2F, texB2F, texB2F, texB2F2, texB2F2, texB2R); 
                break;
            case OFFICE_C: 
                DrawTallBuilding(b, 3, 3.0f, texB5F, texB5F1, texB5F1, texB5F2, texB5S1, texB5F2, texB5S1, texB5S1, texB5R); 
                break;  
            case MINIMARKET: 
                DrawTallBuilding(b, 3, 3.0f, texB6F1, texMMS, texMMF, texMMF, texB6F3, texMMS, texMMS, texMMS, texB6R); 
                break;  
            case POLICES: 
                DrawBoxBuilding(b, texFP, texFP, texSP, texSP, texRP); 
                break;
            case POLICES1: 
                DrawTallBuilding(b, 2, 3.0f, texSP, texS1P, texS1P, texSP, texS1P, texS1P, texS1P, texS1P, texRP); 
                break;    
            case POLICES2: 
                DrawTallBuilding(b, 2, 3.0f, texSP, texS1P, texS1P, texSP, texS1P, texS1P, texS1P, texS1P, texRP); 
                break; 
            case SCHOOL: 
                DrawTallBuilding(b, 2, 3.0f, texFSLS, texFSLS, texFSLS, texFSLS, texFSLS, texFSLS, texFSLS, texFSLS, texFSLR); 
                break; 
            case SCHOOL1: 
                DrawTallBuilding(b, 2, 3.0f, texFSL, texFSLS, texFSLS, texFSLS, texF1SL, texFSLS, texFSLS, texFSLS, texFSLR); 
                break;    
            case SCHOOL2: 
                DrawTallBuilding(b, 2, 3.0f, texFSLS, texFSLS, texFSLS, texFSLS, texFSLS, texFSLS, texFSLS, texFSLS, texFSLR); 
                break;    
            case GSC: 
                DrawBoxBuilding(b, texSGSC, texFGSC, texSGSC, texFGSC, texRGSC); 
                break;
            case BOOKSTORE: 
                DrawBoxBuilding(b, texRBKS, texSBKS, texRBKS, texFBKS, texRBKS); 
                break;
            case BUILDING_b71: 
                DrawTallBuilding(b, 2, 3.0f, texB7S, texB7F, texB7F2, texB7S, texB7S1, texB7S, texB7S1, texB7S, texB7R); 
                break;
            case BUILDING_b72: 
                DrawTallBuilding(b, 2, 3.0f, texB7S, texB7F2, texB7F1, texB7S, texB7S1, texB7S, texB7S1, texB7S, texB7R); 
                break;
            case BUILDING_b8: 
                DrawTallBuilding(b, 2, 3.0f, texB8F2, texB8F2, texB8F2, texB8F, texB8S2, texB8S2, texB8S, texB8S, texB8R); 
                break;
            case BUILDING_b9: 
                DrawTallBuilding(b, 2, 3.0f, texB9F1, texB9S1, texB9F, texB9F, texB9S, texB9S, texB9S, texB9S, texB9R); 
                break;
            case BOUTIQUE: 
                DrawBoxBuilding(b, texFBTQ, texSBTQ, texSBTQ, texFBTQ, texRBTQ); 
                break;
            default: break;
        }
    }
}
void CityMap::buildFacilities() {
    buildings.clear();
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            FacilityType f = facilityMap[y][x];
            // Only place buildings on Grass tiles that have a facility assigned
            if (f == NO || tileMap[y][x] != GRASS) continue;

            Building b;
            b.type = f;
           b.pos = { (float)x * TILE + 2.0f, 0.0f, (float)y * TILE + 2.0f };
            b.width = TILE; 
            b.height = 3.0f; // Standard building height from source
            b.depth = TILE;

            // Apply orientation logic from your main(6).cpp data
            switch(f) {
                case RESTAURANT: b.frontDir = FACE_S; b.side1Dir = FACE_E; break;
                case BCOFFEE:    b.frontDir = FACE_W; b.side1Dir = FACE_N; break;
                case HOSPITAL:   b.frontDir = FACE_W; b.side1Dir = FACE_S; break;
                case POLICES:    b.frontDir = FACE_N; b.side1Dir = FACE_E; break;
                default:         b.frontDir = FACE_S; b.side1Dir = FACE_E; break;
            }

            // Calculate remaining directions automatically
            b.backDir = (b.frontDir == FACE_N) ? FACE_S : (b.frontDir == FACE_S) ? FACE_N : (b.frontDir == FACE_E) ? FACE_W : FACE_E;
            b.side2Dir = (b.side1Dir == FACE_N) ? FACE_S : (b.side1Dir == FACE_S) ? FACE_N : (b.side1Dir == FACE_E) ? FACE_W : FACE_E;
            
            buildings.push_back(b);
        }
    }
}
void CityMap::DrawBoxBuilding(const Building& b, Texture2D frontTex, Texture2D side1Tex, Texture2D side2Tex, Texture2D backTex, Texture2D roofTex) {
    float wallThickness = 0.2f;
    float halfW = b.width / 2.0f;
    float halfD = b.depth / 2.0f;
    float wallOffset = wallThickness / 2.0f;

    auto drawWall = [&](Texture2D tex, Vector3 size, Vector3 wPos) {
        cube.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = tex;
        cube.transform = MatrixMultiply(MatrixScale(size.x, size.y, size.z), MatrixTranslate(wPos.x, wPos.y, wPos.z));
        DrawModel(cube, Vector3Zero(), 1.0f, WHITE);
    };

    // Front/Back Walls
    drawWall(frontTex, {b.width, b.height, wallThickness}, {b.pos.x, b.pos.y + b.height/2, b.pos.z + halfD - wallOffset});
    drawWall(backTex,  {b.width, b.height, wallThickness}, {b.pos.x, b.pos.y + b.height/2, b.pos.z - halfD + wallOffset});
    // Side Walls
    drawWall(side1Tex, {wallThickness, b.height, b.depth}, {b.pos.x + halfW - wallOffset, b.pos.y + b.height/2, b.pos.z});
    drawWall(side2Tex, {wallThickness, b.height, b.depth}, {b.pos.x - halfW + wallOffset, b.pos.y + b.height/2, b.pos.z});
    // Roof
    cube.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = roofTex;
    cube.transform = MatrixMultiply(MatrixScale(b.width, 0.2f, b.depth), MatrixTranslate(b.pos.x, b.pos.y + b.height, b.pos.z));
    DrawModel(cube, Vector3Zero(), 1.0f, WHITE);
}

void CityMap::DrawTallBuilding(const Building& b, int floors, float fHeight, Texture2D fBase, Texture2D s1Base, Texture2D s2Base, Texture2D bBase, Texture2D fMid, Texture2D s1Mid, Texture2D s2Mid, Texture2D bMid, Texture2D roofTex) {
    Building part = b;
    part.height = fHeight;
    // Ground Floor
    DrawBoxBuilding(part, fBase, s1Base, s2Base, bBase, roofTex);
    // Upper Floors
    for (int i = 1; i < floors; i++) {
        part.pos.y = b.pos.y + i * fHeight;
        DrawBoxBuilding(part, fMid, s1Mid, s2Mid, bMid, roofTex);
    }
}
// ... include worldToTile, tileCenter, and findFastestPath ...

Vector3 CityMap::tileCenter(int y, int x) const {
    return Vector3{ (float)x * TILE + 2.0f, 0.1f, (float)y * TILE + 2.0f };
}

bool CityMap::isDriveableTile(int y, int x) const {
    if (y < 0 || y >= H || x < 0 || x >= W) return false;
    int t = tileMap[y][x];
    // Driveable surfaces are all non-grass road-like tiles. Playground should not be driveable.
    return (t != GRASS && t != PLAYGROUND3);
}

bool CityMap::isDriveableWorld(Vector3 world) const { 
    int y, x;
    if (!worldToTile(world, y, x)) return false;
    return isDriveableTile(y, x);
}

Vector3 CityMap::clampToDriveable(Vector3 world) const {
    int y, x;
    if (!worldToTile(world, y, x)) {
        int cx = (int)Clamp(world.x / TILE, 0.0f, (float)(W - 1));
        int cy = (int)Clamp(world.z / TILE, 0.0f, (float)(H - 1));
        return tileCenter(cy, cx);
    }
    if (isDriveableTile(y, x)) return world;

    // Search nearby tiles for the closest driveable spot
    for (int r = 1; r <= 3; r++) {
        for (int dy = -r; dy <= r; dy++) {
            for (int dx = -r; dx <= r; dx++) {
                int ny = y + dy;
                int nx = x + dx;
                if (isDriveableTile(ny, nx)) return tileCenter(ny, nx);
            }
        }
    }
    // Fallback: snap to current tile center
    return tileCenter(y, x);
}


bool CityMap::isDrivableTile(int t, bool emergency) const {
    switch ((TileType)t) {
        case ROAD_H: case ROAD_V: case PROAD_V: 
        case PROAD_V1: case INTERSECTION: case ROUNDABOUT:
        case CURVERTOP: case CURVERTOP1: 
        case CURVERBOTTOM: case CURVERBOTTOM1:
        case TROAD: case TROAD1: case ROTROAD: 
            return true; // Only roads are now drivable
        default: 
            return false;
    }
}
bool CityMap::worldToTile(Vector3 world, int& outY, int& outX) const {
    // world.x corresponds to Columns (W)
    // world.z corresponds to Rows (H)
    outX = (int)floorf(world.x / TILE);
    outY = (int)floorf(world.z / TILE); 
    
    if (outX < 0) outX = 0; if (outX >= W) outX = W - 1;
    if (outY < 0) outY = 0; if (outY >= H) outY = H - 1;
    return true;
}
float CityMap::tileSpeedLimit(int t, bool emergency) const {
    return 5.0f;
}

std::vector<Vector3> CityMap::findFastestPath(Vector3 startWorld, Vector3 goalWorld, bool emergency) const {
    int sy, sx, gy, gx;
    worldToTile(startWorld, sy, sx);
    worldToTile(goalWorld, gy, gx);

    const int N = W*H;
    auto idx = [&](int y,int x){ return y*W+x; };
    const float INF = std::numeric_limits<float>::infinity();
    std::vector<float> dist(N, INF);
    std::vector<int> prev(N, -1);
    struct Node { float d; int id; };
    struct Cmp { bool operator()(const Node& a, const Node& b) const { return a.d > b.d; } };
    std::priority_queue<Node, std::vector<Node>, Cmp> pq;

    int s = idx(sy,sx), g = idx(gy,gx);
    dist[s] = 0.0f;
    pq.push({0.0f, s});
    int dirs[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};

    while(!pq.empty()){
        Node cur = pq.top(); pq.pop();
        if (cur.d > dist[cur.id]) continue;
        if (cur.id == g) break;

        int y = cur.id / W, x = cur.id % W;
        for(auto& d:dirs){
            int ny=y+d[0], nx=x+d[1];
            if(ny<0||ny>=H||nx<0||nx>=W) continue;
            if (!isDrivableTile(tileMap[ny][nx], emergency)) continue;

            float cost = 1.0f;
            int nid = idx(ny,nx);
            if (cur.d + cost < dist[nid]) {
                dist[nid] = cur.d + cost;
                prev[nid] = cur.id;
                pq.push({dist[nid], nid});
            }
        }
    }

    if (dist[g] == INF) return {};
    std::vector<Vector3> wp;
    for(int cur=g; cur!=-1; cur=prev[cur]) {
        int y = cur / W, x = cur % W;
        wp.push_back(tileCenter(y,x));
    }
    std::reverse(wp.begin(), wp.end());
    if (!wp.empty()) wp.back() = tileCenter(gy,gx);
    return wp;
}