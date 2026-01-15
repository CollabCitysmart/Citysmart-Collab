#include "EmergencyVehicle.h"
#include "CityMap.h" 
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h" 

EmergencyVehicle::EmergencyVehicle(Vector3 startPos, Vector3 endPos, float vSpeed, Color vColor, std::string vType)
    : Vehicle(startPos, endPos, vSpeed, vColor), type(vType), isSirenActive(false),
      sirenMultiplier(1.5f), normalMultiplier(1.0f) {}

void EmergencyVehicle::setSirenActive(bool on) {
    isSirenActive = on;
    speed = baseSpeed * (isSirenActive ? sirenMultiplier : normalMultiplier);
}

void EmergencyVehicle::toggleSiren(bool on) {
    setSirenActive(on);
}

void EmergencyVehicle::assignIncident(CityMap* map, Vector3 targetPos) {
    setSirenActive(true);
    std::vector<Vector3> newPath = map->findFastestPath(position, targetPos, true);
    if (!newPath.empty()) {
        setPath(newPath);
    }
}

Vector3 EmergencyVehicle::getForwardDir() const {
    Vector3 dir = { sinf(rotation*DEG2RAD), 0, cosf(rotation*DEG2RAD) };
    return Vector3Normalize(dir);
}

void EmergencyVehicle::update() {
    Vehicle::update(); 
}


void EmergencyVehicle::draw() const {
    
    Vehicle::draw(); 

  
    if (getSirenActive()) {
        rlPushMatrix();
        rlTranslatef(position.x, position.y, position.z);
        rlRotatef(rotation, 0, 1, 0);
        rlTranslatef(0, 0.5f, 0); 
        float blink = sinf(GetTime() * 15.0f);
        Color sirenColor = (blink > 0) ? RED : BLUE;
        DrawCube(Vector3Zero(), 0.3f, 0.3f, 0.3f, sirenColor);
        rlPopMatrix();
    }
}