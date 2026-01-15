#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H

#include "raylib.h"
#include "raymath.h"
#include <algorithm>
#include <cstdlib> // Required for rand()

enum class LightState { Green, Yellow, Red };

class TrafficLight {
private:
    Vector3 position{};
    LightState state{LightState::Green};
    float timer{0.0f};
    float greenTime{3.0f};      // Default cycle time (matches main.cpp)
    float forcedGreen{0.0f};    

public:
    TrafficLight() = default;

    // This is the constructor used in main.cpp
    TrafficLight(Vector3 pos, float greenSeconds = 3.0f)
        : position(pos), state(LightState::Green), greenTime(greenSeconds), forcedGreen(0.0f) {
        
        // FIX 1: Randomize start time so lights are out of sync
        timer = (float)(rand() % (int)(greenSeconds * 100)) / 100.0f;
        
        // FIX 2: Randomize starting color
        if (rand() % 2 == 0) state = LightState::Red;
    }

    void update(float dt) {
        if (forcedGreen > 0.0f) {
            forcedGreen -= dt;
            state = LightState::Green;
            return;
        }

        timer += dt;

        // Green -> Yellow -> Red -> Green
        if (state == LightState::Green && timer > greenTime) {
            state = LightState::Yellow;
            timer = 0.0f;
        } else if (state == LightState::Yellow && timer > 2.0f) {
            state = LightState::Red;
            timer = 0.0f;
        } else if (state == LightState::Red && timer > greenTime) {
            state = LightState::Green;
            timer = 0.0f;
        }
    }

    void forceGreen(float seconds = 1.2f) {
        forcedGreen = std::max(forcedGreen, seconds);
        state = LightState::Green;
        timer = 0.0f;
    }

    void greenForEmergencyIfApproaching(const Vector3& emPos, const Vector3& emForward, float triggerDist = 12.0f) {
        Vector3 toLight = Vector3Subtract(position, emPos);
        float d = Vector3Length(toLight);
        if (d > triggerDist || d < 0.001f) return;

        Vector3 dir = Vector3Normalize(toLight);
        // If the light is in front of the emergency vehicle, keep it green
        if (Vector3DotProduct(dir, emForward) > 0.5f) {
            forceGreen(1.2f);
        }
    }

    bool isRed() const { return forcedGreen <= 0.0f && state == LightState::Red; }
    LightState getState() const { return state; }
    Vector3 getPosition() const { return position; }

    bool shouldStop(const Vector3& vehPos, const Vector3& vehForward, float stopDist = 2.2f) const {
        if (!isRed()) return false;

        Vector3 toLight = Vector3Subtract(position, vehPos);
        float d = Vector3Length(toLight);
        if (d > stopDist || d < 0.001f) return false;

        Vector3 dir = Vector3Normalize(toLight);
        return Vector3DotProduct(dir, vehForward) > 0.6f;
    }

    void draw() const {
        // Pole
        DrawCube(position, 0.2f, 4.0f, 0.2f, DARKGRAY);

        // Housing
        Vector3 boxPos = { position.x, position.y + 3.0f, position.z };
        DrawCube(boxPos, 0.4f, 1.0f, 0.4f, BLACK);

        // Light color
        Color lightColor = GRAY;
        if (state == LightState::Red) lightColor = RED;
        if (state == LightState::Yellow) lightColor = ORANGE;
        if (state == LightState::Green) lightColor = GREEN;

        Vector3 lightSphere = { position.x, position.y + 3.0f, position.z + 0.35f };
        DrawSphere(lightSphere, 0.25f, lightColor);
    }
};

#endif