#ifndef EMERGENCYVEHICLE_H
#define EMERGENCYVEHICLE_H

#include "Vehicle.h"
#include <string>
#include <vector>


class CityMap; 

class EmergencyVehicle : public Vehicle {
private:
    std::string type;
    bool isSirenActive;
    float sirenMultiplier;
    float normalMultiplier;
    Vector3 hospitalPos; 
    bool isReturning;

public:
    EmergencyVehicle(Vector3 startPos, Vector3 endPos, float vSpeed, Color vColor, std::string vType);

    void setSirenActive(bool on);
    bool getSirenActive() const { return isSirenActive; }
    void toggleSiren(bool on);

    // UPDATED: Uses CityMap to find path
    void assignIncident(CityMap* map, Vector3 targetPos);

    void update() override;
    void draw() const override;
    
    Vector3 getForwardDir() const;
};

#endif