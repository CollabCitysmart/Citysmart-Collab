#include "raylib.h"
#include "raymath.h"
#include <vector>

// =====================================================
// ENUMS
// =====================================================

enum TileType {
    GRASS = 0,
    ROAD_H,
    ROAD_V,
    PROAD_V,
    INTERSECTION,
    ROUNDABOUT,
    CURVERTOP,
    CURVERTOP1,
    CURVERBOTTOM,
    CURVERBOTTOM1,
    TROAD,
    TROAD1,
    ROTROAD,
    PARKING,
    PPARKING1,
    PPARKING2,
    LGARDEN , 
    PLAYGROUND1,
    PLAYGROUND2,
    PLAYGROUND3
};

enum ZoneType {
    NONE = 0,
    DOWNTOWN,
    EMERGENCY,
    MEDICAL,
    EDUCATION,
};

enum FacilityType {
    NO = 0,
    RESTAURANT,
    SHOP ,
    BCOFFEE,
    HOSPITAL,
};

enum FacingDirection
{
    FACE_N, // -Z (north)
    FACE_S, // +Z (south)
    FACE_E,  // +X (east)
    FACE_W   // -X (west)
};


// =====================================================
// MAP MACROS
// =====================================================

#define G  GRASS
#define H  ROAD_H
#define V  ROAD_V
#define PV PROAD_V
#define I  INTERSECTION
#define R  ROUNDABOUT
#define CT CURVERTOP
#define CT1 CURVERTOP1
#define CB CURVERBOTTOM
#define CB1 CURVERBOTTOM1
#define TR TROAD
#define TR1 TROAD1
#define RTT ROTROAD
#define P  PARKING
#define PP1 PPARKING1
#define PP2 PPARKING2
#define LG LGARDEN
#define PG1 PLAYGROUND1
#define PG2 PLAYGROUND2
#define PG3 PLAYGROUND3



// =====================================================
// TILE MAP
// =====================================================

static int TILE_MAP[10][10] = {
    {PP2,PG1,G,G,PG3,LG,V,G,G,G},
    {PP1,PG2,CT,H,CB1,PP2,V,G,CT,H},

    {PV,H,RTT,G,TR,PV,I,H,CB,G},
    {G,G,V,G,V,G,V,G,G,G},

    {G,G,V,G,V,G,V,G,G,G},
    {H,H,TR1,H,R,H,I,H,CB1,G},

    {G,G,G,G,V,G,V,G,V,G},
    {CT,H,H,H,I,H,CB,G,CT1,H},

    {V,G,G,G,V,G,G,G,G,G},
    {V,G,G,G,TR,H,H,H,H,H}
};

// =====================================================
// FACILITY MAP
// =====================================================

#define N NO
#define REST RESTAURANT
#define HP HOSPITAL
#define BC BCOFFEE
#define S SHOP

static FacilityType FACILITY_MAP[10][10] = {
    {N,N,N,N,N,N,N,N,N,N},
    {N,N,N,N,N,N,N,N,N,N},

    {N,N,N,N,N,N,N,N,N,HP},
    {N,N,N,BC,N,N,N,N,N,N},

    {N,N,N,N,N,N,N,N,N,N},
    {N,N,N,N,N,N,N,N,N,N},

    {REST,N,N,N,N,N,N,N,N,N},
    {N,N,N,N,N,N,N,N,N,N},

    {N,N,N,N,N,N,N,N,N,N},
    {N,N,N,N,N,N,N,N,N,N}
};

// =====================================================
// DRAW HELPERS
// =====================================================

void DrawTile(Model &model, Texture2D texture, Vector3 pos,
              float width, float height, float depth, float rotationY)
{
    model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;

    Matrix scale = MatrixScale(width, height, depth);
    Matrix rot   = MatrixRotateY(rotationY * DEG2RAD);
    Matrix trans = MatrixTranslate(pos.x, pos.y, pos.z);

    model.transform = MatrixMultiply(scale, MatrixMultiply(rot, trans));
    DrawModel(model, Vector3Zero(), 1.0f, WHITE);
    model.transform = MatrixIdentity();
}

struct Building {
    FacilityType type;
    Vector3 pos;
    float width;
    float height;
    float depth;

    FacingDirection frontDir;
    FacingDirection backDir;  // ← new
    FacingDirection side1Dir;
    FacingDirection side2Dir;

};

void DrawBoxBuilding(Model& cube, const Building& b,
                     Texture2D frontTex,
                     Texture2D sideTex,
                     Texture2D backTex,
                     Texture2D roofTex)
{
    float wallThickness = 0.2f;   // wall thickness
    float offset = 0.01f;         // small gap to avoid Z-fighting
    float halfW = b.width / 2.0f;
    float halfD = b.depth / 2.0f;
    float wallOffset = wallThickness / 2.0f;

    auto drawWall = [&](FacingDirection dir, float lengthX, float lengthZ, Texture2D tex, Vector3 pos)
    {
        cube.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = tex;
        cube.transform = MatrixMultiply(
            MatrixScale(lengthX, b.height, lengthZ),
            MatrixTranslate(pos.x, pos.y + b.height / 2 + offset, pos.z)
        );
        DrawModel(cube, Vector3Zero(), 1.0f, WHITE);
    };

    // --- FRONT WALL ---
    if (b.frontDir == FACE_N || b.frontDir == FACE_S)
        drawWall(b.frontDir, b.width, wallThickness, frontTex,
                 {b.pos.x, b.pos.y, b.pos.z + ((b.frontDir == FACE_N) ? -halfD + wallOffset : halfD - wallOffset)});
    else
        drawWall(b.frontDir, wallThickness, b.depth, frontTex,
                 {b.pos.x + ((b.frontDir == FACE_E) ? halfW - wallOffset : -halfW + wallOffset), b.pos.y, b.pos.z});

    // --- BACK WALL ---
    if (b.backDir == FACE_N || b.backDir == FACE_S)
        drawWall(b.backDir, b.width, wallThickness, backTex,
                 {b.pos.x, b.pos.y, b.pos.z + ((b.backDir == FACE_N) ? -halfD + wallOffset : halfD - wallOffset)});
    else
        drawWall(b.backDir, wallThickness, b.depth, backTex,
                 {b.pos.x + ((b.backDir == FACE_E) ? halfW - wallOffset : -halfW + wallOffset), b.pos.y, b.pos.z});

    // --- SIDE1 WALL ---
    if (b.side1Dir == FACE_N || b.side1Dir == FACE_S)
        drawWall(b.side1Dir, b.width, wallThickness, sideTex,
                 {b.pos.x, b.pos.y, b.pos.z + ((b.side1Dir == FACE_N) ? -halfD + wallOffset : halfD - wallOffset)});
    else
        drawWall(b.side1Dir, wallThickness, b.depth, sideTex,
                 {b.pos.x + ((b.side1Dir == FACE_E) ? halfW - wallOffset : -halfW + wallOffset), b.pos.y, b.pos.z});

    // --- SIDE2 WALL ---
    if (b.side2Dir == FACE_N || b.side2Dir == FACE_S)
        drawWall(b.side2Dir, b.width, wallThickness, sideTex,
                 {b.pos.x, b.pos.y, b.pos.z + ((b.side2Dir == FACE_N) ? -halfD + wallOffset : halfD - wallOffset)});
    else
        drawWall(b.side2Dir, wallThickness, b.depth, sideTex,
                 {b.pos.x + ((b.side2Dir == FACE_E) ? halfW - wallOffset : -halfW + wallOffset), b.pos.y, b.pos.z});

// --- ROOF ---
cube.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = roofTex;

// Slightly larger than walls but much smaller than previous overhang
float roofWidth  = b.width  + wallThickness / 4.0f;  // small extra overhang
float roofDepth  = b.depth  + wallThickness / 4.0f;
float roofHeight = wallThickness; // thickness of the roof

cube.transform = MatrixMultiply(
    MatrixScale(roofWidth, roofHeight, roofDepth),
    MatrixTranslate(b.pos.x, b.pos.y + b.height + roofHeight / 2 + offset, b.pos.z)
);

DrawModel(cube, Vector3Zero(), 1.0f, WHITE);

DrawModel(cube, Vector3Zero(), 1.0f, WHITE);
}


// =====================================================
// MAIN
// =====================================================

int main()
{
    InitWindow(1920, 1080, "Smart City - Emergency Demo");
    SetTargetFPS(60);

    const float TILE = 4.0f;

    Camera3D cam = {0};
    cam.position = {40, 35, 40};
    cam.target   = {20, 0, 20};
    cam.up       = {0, 1, 0};
    cam.fovy     = 60;
    cam.projection = CAMERA_PERSPECTIVE;

    Vector3 camStartPos = cam.position;
    Vector3 camStartTarget = cam.target;
    float camStartFovy = cam.fovy;
    float camAngle = 45.0f;
    float camRadius = 55.0f;
    float camHeight = 30.0f;

    // ---------------- TEXTURES ----------------
    Texture2D g   = LoadTexture("assets/grass.png");
    Texture2D RH  = LoadTexture("road_h.png");
    Texture2D RV  = LoadTexture("road_v.png");
    Texture2D PR  = LoadTexture("proad_v.png");
    Texture2D INT = LoadTexture("intersection.png");
    Texture2D CVT = LoadTexture("cvroad_t.png");
    Texture2D CVT1= LoadTexture("cvroad_t1.png");
    Texture2D CVB = LoadTexture("assets3/cvroad_b.png");
    Texture2D CVB1= LoadTexture("cvroad_b1.png");
    Texture2D TRD = LoadTexture("t_road.png");
    Texture2D TRD1= LoadTexture("t_road1.png");
    Texture2D ROT = LoadTexture("rott_road.png");
    Texture2D PK  = LoadTexture("parking.png");
    Texture2D PK1 = LoadTexture("pparking_1.png");
    Texture2D PK2 = LoadTexture("pparking_2.png");
    Texture2D RB = LoadTexture("roundabout.png");
    Texture2D LN = LoadTexture("lgarden.png");
    Texture2D PY1 = LoadTexture("playground_1.png");
    Texture2D PY2 = LoadTexture("playground_2.png");
    Texture2D PY3 = LoadTexture("playground_3.png");

    

    
    Texture2D FREST = LoadTexture("f_restaurant.png");
    Texture2D RBREST = LoadTexture("rb_restaurant.png");
    Texture2D FBCOF = LoadTexture("f_bcoffe.png");
    Texture2D RBBCOF = LoadTexture("rb_bcoffe.png");

    Model cube = LoadModelFromMesh(GenMeshCube(1,1,1));

   // ---------------- BUILD FACILITIES ----------------
std::vector<Building> buildings;


for (int y = 0; y < 10; y++) {
    for (int x = 0; x < 10; x++) {

        FacilityType f = FACILITY_MAP[y][x];
        if (f == NO) continue;
        if (TILE_MAP[y][x] != GRASS) continue; // only place buildings on grass

        Building b;
        b.type = f;
        b.pos  = { x * TILE, 0.0f, y * TILE };

        // default dimensions to cover full tile
        b.width  = TILE;
        b.height = 3.0f;
        b.depth  = TILE;

        // assign frontDir based on facility
        switch(f)
        {
            case RESTAURANT:
                b.width  = TILE;    // covers entire tile
                b.depth  = TILE;
                b.frontDir = FACE_S;  // front faces NORTH
                b.side1Dir = FACE_E;  // side1 faces EAST
               break;

            case BCOFFEE:
                b.width  = TILE;    
                b.depth  = TILE;
                b.frontDir = FACE_W;  // front faces SOUTH
                b.side1Dir = FACE_N;  // side1 faces EAST
              
                break;


            default:
                b.frontDir = FACE_S;
                b.side1Dir = FACE_E;
                break;
    
        }

        // automatically calculate backDir as opposite
        b.backDir = (b.frontDir == FACE_N) ? FACE_S :
                    (b.frontDir == FACE_S) ? FACE_N :
                    (b.frontDir == FACE_E) ? FACE_W :
                    FACE_E;

        b.side2Dir = (b.side1Dir == FACE_N) ? FACE_S :
             (b.side1Dir == FACE_S) ? FACE_N :
             (b.side1Dir == FACE_E) ? FACE_W :
             FACE_E;            

        buildings.push_back(b);
    }
}

    // ---------------- LOOP ----------------
    while (!WindowShouldClose())
    {   
         float moveSpeed = 20.0f * GetFrameTime();
 
     // Forward / Backward (W / S)
  if (IsKeyDown(KEY_W)) {
    cam.position.z -= moveSpeed;
    cam.target.z   -= moveSpeed;
}
  if (IsKeyDown(KEY_S)) {
    cam.position.z += moveSpeed;
    cam.target.z   += moveSpeed;
}

  // Left / Right (A / D)
  if (IsKeyDown(KEY_A)) {
    cam.position.x -= moveSpeed;
    cam.target.x   -= moveSpeed;
}
if (IsKeyDown(KEY_D)) {
    cam.position.x += moveSpeed;
    cam.target.x   += moveSpeed;
}

// Zoom IN (Z)
if (IsKeyDown(KEY_Z)) {
    cam.fovy -= 40 * GetFrameTime();
    if (cam.fovy < 20) cam.fovy = 20;
}

// Zoom OUT (X)
if (IsKeyDown(KEY_X)) {
    cam.fovy += 40 * GetFrameTime();
    if (cam.fovy > 90) cam.fovy = 90;
}

// Reset camera (R)
if (IsKeyPressed(KEY_R)) {
    cam.position = camStartPos;
    cam.target   = camStartTarget;
    cam.fovy     = camStartFovy;
}

// Rotate
if (IsKeyDown(KEY_LEFT))  camAngle -= 60 * GetFrameTime();
if (IsKeyDown(KEY_RIGHT)) camAngle += 60 * GetFrameTime();


// Zoom
if (IsKeyDown(KEY_UP))   camRadius -= 20 * GetFrameTime();
if (IsKeyDown(KEY_DOWN)) camRadius += 20 * GetFrameTime();
    
// Clamp zoom
camRadius = Clamp(camRadius, 15.0f, 120.0f);

// Update ORBIT camera position
cam.position.x = cam.target.x + cosf(camAngle * DEG2RAD) * camRadius;
cam.position.z = cam.target.z + sinf(camAngle * DEG2RAD) * camRadius;
cam.position.y = camHeight;

        BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode3D(cam);

        // ---- MAP ----
        for (int y = 0; y < 10; y++) {
            for (int x = 0; x < 10; x++) {

                Vector3 pos = { x * TILE, 0, y * TILE };

                if (TILE_MAP[y][x] == GRASS)
                    DrawTile(cube, g, pos, TILE, 0.1f, TILE, 0);

                switch (TILE_MAP[y][x]) {
                    case ROAD_H: DrawTile(cube, RH, pos, TILE, 0.11f, TILE, 0); break;
                    case ROAD_V: DrawTile(cube, RV, pos, TILE, 0.11f, TILE, 0); break;
                    case PROAD_V:DrawTile(cube, PR, pos, TILE, 0.11f, TILE, 90); break;
                    case INTERSECTION: DrawTile(cube, INT, pos, TILE, 0.11f, TILE, 0); break;
                    case ROUNDABOUT: DrawTile(cube, RB, pos, TILE, 0.11f, TILE, 0); break;
                    case CURVERTOP: DrawTile(cube, CVT, pos, TILE, 0.11f, TILE, 0); break;
                    case CURVERTOP1:DrawTile(cube, CVT1,pos,TILE,0.11f,TILE,0); break;
                    case CURVERBOTTOM:DrawTile(cube,CVB,pos,TILE,0.11f,TILE,180); break;
                    case CURVERBOTTOM1:DrawTile(cube,CVB1,pos,TILE,0.11f,TILE,0); break;
                    case TROAD:DrawTile(cube,TRD,pos,TILE,0.11f,TILE,0); break;
                    case TROAD1:DrawTile(cube,TRD1,pos,TILE,0.11f,TILE,180); break;
                    case ROTROAD:DrawTile(cube,ROT,pos,TILE,0.11f,TILE,0); break;
                    case PARKING:DrawTile(cube,PK,pos,TILE,0.11f,TILE,90); break;
                    case PPARKING1:DrawTile(cube,PK1,pos,TILE,0.11f,TILE,90); break;
                    case PPARKING2:DrawTile(cube,PK2,pos,TILE,0.11f,TILE,90); break;
                    case LGARDEN:DrawTile(cube,LN,pos,TILE,0.11f,TILE,0); break;
                    case PLAYGROUND1:DrawTile(cube,PY1,pos,TILE,0.11f,TILE,90); break;
                    case PLAYGROUND2:DrawTile(cube,PY2,pos,TILE,0.11f,TILE,90); break;
                    case PLAYGROUND3:DrawTile(cube,PY3,pos,TILE,0.11f,TILE,-90); break;
                    default: break;
                }
            }
        }

        for (const Building &b : buildings)
{
    switch (b.type)
    {
        case RESTAURANT:
            DrawBoxBuilding(cube, b, FREST, RBREST, RBREST, RBREST);
            break;

        case BCOFFEE:
            DrawBoxBuilding(cube, b, FBCOF, RBBCOF, RBBCOF, RBBCOF);
            break;
/*
        case SHOP:
            DrawBoxBuilding(cube, b, SHOP_FRONT, SHOP_SIDE, SHOP_BACK, SHOP_ROOF);
            break;

        case HOSPITAL:
            DrawBoxBuilding(cube, b, HOSPITAL_WALL, HOSPITAL_WALL, HOSPITAL_WALL, HOSPITAL_ROOF);
            break;*/

        default:
            break;
    }
}



        EndMode3D();

        DrawText("Emergency & Facility Demo READY", 20, 20, 20, DARKGREEN);
        EndDrawing();
    }

    UnloadTexture(g);
    UnloadTexture(RH);
    UnloadTexture(RV);
    UnloadTexture(PR);
    UnloadTexture(INT);
    UnloadTexture(RB);
    UnloadTexture(CVT);
    UnloadTexture(CVT1);
    UnloadTexture(CVB);
    UnloadTexture(CVB1);
    UnloadTexture(TRD);
    UnloadTexture(TRD1);
    UnloadTexture(ROT);
    UnloadTexture(PK);
    UnloadTexture(PK1);
    UnloadTexture(PK2);
    UnloadTexture(FREST);
    UnloadTexture(RBREST);
    UnloadTexture(FBCOF);
    UnloadTexture(RBBCOF);
    UnloadTexture(LN);
    

    CloseWindow();
    return 0;
}
