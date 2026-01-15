#ifndef VEHICLE_H
#define VEHICLE_H

#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <cmath>

class Vehicle {
protected:
    Vector3 position;
    Vector3 destination;
    float speed;
    float baseSpeed;
    Color color;
    bool looping;
    std::vector<Vector3> path;
    int currentWp;
    float rotation;
    float laneOffset;
    float laneOffsetDefault{0.8f};
    float laneOffsetTarget{0.8f};
    float laneOffsetMax{1.2f};
    float yieldHold{0.0f};
    float speedScale{1.0f};
    float speedScaleHold{0.0f};

public:
    // Update vehicle each frame. Use step() to control forward motion while keeping lane/yield smoothing.
    Vehicle(Vector3 startPos, Vector3 endPos, float vSpeed, Color vColor);
    virtual ~Vehicle() = default;
    void setSpeedScale(float scale, float holdSeconds = 0.0f);
    Vector3 getIntendedDir() const;
    void step(float dt, bool allowForward = true);
    void setLooping(bool loop) { looping = loop; }
    void setLaneOffset(float offset);
    
    void nudgeLaneOffset(float targetOffset, float holdSeconds = 0.8f);
    void setPosition(const Vector3& p) { position = p; }

    virtual void update();
    virtual void draw() const;
    Vector3 getDestination() const { return destination; }

    Vector3 getPosition() const { return position; }
    Vector3 getForwardDir() const { return { sinf(rotation * DEG2RAD), 0.0f, cosf(rotation * DEG2RAD) }; }
    float getSpeed() const { return speed; }
    float getBaseSpeed() const { return baseSpeed; }
    float getLaneOffset() const { return laneOffset; }
    bool hasFinishedPath() const { return path.empty() || currentWp >= (int)path.size(); }

    void setPath(const std::vector<Vector3>& newPath);
    void yieldTo(Vector3 emergencyPos, Vector3 emergencyDir, float yieldStrength, float yieldRadius);
};

#endif