#ifndef CITYMAP_H
#define CITYMAP_H

#include "GameCommon.h"
#include <vector>

struct Building {
    FacilityType type;
    Vector3 pos;
    float width, height, depth;
    FacingDirection frontDir, backDir, side1Dir, side2Dir;
};

class CityMap {
public:
    static const int W = 10;
    static const int H = 10;
    int tileMap[H][W];
    FacilityType facilityMap[H][W];

    CityMap();
    ~CityMap();

    void draw(); 
    bool worldToTile(Vector3 world, int& outY, int& outX) const;
    Vector3 tileCenter(int y, int x) const;
    bool isDriveableTile(int y, int x) const;
    bool isDriveableWorld(Vector3 world) const;
    Vector3 clampToDriveable(Vector3 world) const;
    std::vector<Vector3> findFastestPath(Vector3 startWorld, Vector3 goalWorld, bool emergency) const;

    // Expose buildings for camera collision (read-only)
    const std::vector<Building>& GetBuildings() const { return buildings; }

private:
    Model cube;
    std::vector<Building> buildings;
    bool isDrivableTile(int t, bool emergency) const;
    float tileSpeedLimit(int t, bool emergency) const;
    
    void initMaps();
    void loadTextures();
    void buildFacilities();
    
    // Member function signatures
    void DrawTile(Texture2D texture, Vector3 pos, float rotationY);
    void DrawBoxBuilding(const Building& b, Texture2D f, Texture2D s1, Texture2D s2, Texture2D bk, Texture2D r);
    void DrawTallBuilding(const Building& b, int floors, float fHeight, 
                          Texture2D fb, Texture2D s1b, Texture2D s2b, Texture2D bkb,
                          Texture2D fm, Texture2D s1m, Texture2D s2m, Texture2D bkm,
                          Texture2D roof);

    // Texture variables - LISTED ONLY ONCE
    Texture2D texG, texRH, texRV, texPR, texPR1, texINT, texRB, texCVT, texCVT1, texCVB, texFCVB, texCVB1, texTRD, texTRD1, texROT, texPK, texPK1, texPK1h, texPK2, texPK2h, texPY3;
    Texture2D texFREST, texRBREST, texFBCOF, texRBBCOF, texFHT, texHT, texHT1, texHTS1, texHTS2, texPHM, texPHMR;
    Texture2D texBF1, texBF2, texBS1, texBS2, texBS3, texBS4, texBR, texB2F, texB2F2, texB2R, texBK, texRBY;
    Texture2D texB3F, texB3R, texB4F, texB4R, texB5F, texB5F1, texB5F2, texB5S, texB5S1, texB5R, texB6F1, texB6F3, texB6R, texMMF, texMMS;
    Texture2D texB7F, texB7F1, texB7F2, texB7S, texB7S1, texB7R, texB8F, texB8F2, texB8S, texB8S2, texB8R, texB9F, texB9F1, texB9S, texB9S1, texB9R;
    Texture2D texFS1, texFS2, texFS3, texFRS, texFS, texBF, texFCS, texF1CS, texCSS, texCS1, texRCS, texRP, texFP, texSP, texS1P;
    Texture2D texFSL, texF1SL, texFSLS, texFSLR, texFGSC, texSGSC, texRGSC, texFBKS, texSBKS, texRBKS, texFBTQ, texSBTQ, texRBTQ;
};

#endif