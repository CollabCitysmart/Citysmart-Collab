#include "Vehicle.h"
#include "rlgl.h" 
#include <algorithm>

Vehicle::Vehicle(Vector3 startPos, Vector3 endPos, float vSpeed, Color vColor)
    : position(startPos), destination(endPos), speed(vSpeed), baseSpeed(vSpeed), 
      color(vColor), currentWp(0), rotation(0.0f), looping(false) {
        laneOffsetDefault = 0.55f;
        laneOffsetTarget  = 0.55f;
        laneOffsetMax     = 1.5f;
        laneOffset        = laneOffsetDefault;
        speedScale = 1.0f;
        speedScaleHold = 0.0f;
        yieldHold         = 0.0f;
}


void Vehicle::setSpeedScale(float scale, float holdSeconds) {
    // Clamp and hold the scale so we can slow/stop temporarily (e.g., red light, ambulance ahead).
    speedScale = Clamp(scale, 0.0f, 1.5f);
    if (holdSeconds > speedScaleHold) speedScaleHold = holdSeconds;
}

Vector3 Vehicle::getIntendedDir() const {
    // Prefer path direction when available; fallback to current forward.
    if (!path.empty() && currentWp < (int)path.size()) {
        Vector3 d = Vector3Subtract(path[currentWp], position);
        d.y = 0.0f;
        if (Vector3Length(d) > 0.05f) return Vector3Normalize(d);
    }
    return getForwardDir();
}

void Vehicle::step(float dt, bool allowForward) {
    // --- Yield / lane offset smoothing always runs ---
    if (yieldHold > 0.0f) {
        yieldHold -= dt;
        if (yieldHold < 0.0f) yieldHold = 0.0f;
    } else {
        // Smoothly relax back to default instead of snapping (prevents "stuck yielded" lanes)
        laneOffsetTarget = laneOffsetTarget + (laneOffsetDefault - laneOffsetTarget) * Clamp(dt * 6.0f, 0.0f, 1.0f);
    }

    laneOffsetTarget = Clamp(laneOffsetTarget, -laneOffsetMax, laneOffsetMax);
    laneOffset = laneOffset + (laneOffsetTarget - laneOffset) * Clamp(dt * 10.0f, 0.0f, 1.0f);

    // Speed scale relaxation
    if (speedScaleHold > 0.0f) {
        speedScaleHold -= dt;
        if (speedScaleHold < 0.0f) speedScaleHold = 0.0f;
    } else {
        // Return to normal speed smoothly
        speedScale = speedScale + (1.0f - speedScale) * Clamp(dt * 6.0f, 0.0f, 1.0f);
    }

    if (path.empty() || currentWp >= (int)path.size()) return;

    Vector3 logicTarget = path[currentWp];
    logicTarget.y = position.y;

    // --- 1. Direction calculation (stable even when stopped) ---
    Vector3 roadDir = {0,0,1};
    if (currentWp == 0) {
        float distToStart = Vector3Distance(path[0], position);
        if (distToStart < 1.0f && path.size() > 1) roadDir = Vector3Subtract(path[1], path[0]);
        else { roadDir = Vector3Subtract(path[0], position); roadDir.y = 0; }
    } else {
        roadDir = Vector3Subtract(path[currentWp], path[currentWp - 1]);
    }

    if (Vector3Length(roadDir) > 0.001f) roadDir = Vector3Normalize(roadDir);
    else roadDir = getForwardDir();

    // --- 2. Lane offset target (right-hand driving) ---
    Vector3 rightDir = { -roadDir.z, 0.0f, roadDir.x };
    Vector3 driveTarget = Vector3Add(logicTarget, Vector3Scale(rightDir, laneOffset));

    float dist = Vector3Distance(position, driveTarget);

    // --- 3. Waypoint advancement (independent of allowForward) ---
    if (dist < 0.5f) {
        currentWp++;
        if (currentWp >= (int)path.size()) {
            if (looping) currentWp = 0;
            else currentWp = (int)path.size() - 1;
        }
        if (!path.empty()) destination = path[currentWp];
        return; // next frame will steer towards new target
    }

    // --- 4. Movement (optional) ---
    Vector3 dir = Vector3Subtract(driveTarget, position);
    dir.y = 0.0f;
    float distToTarget = Vector3Length(dir);
    if (distToTarget < 0.001f) return;

    Vector3 moveDir = Vector3Normalize(dir);
    rotation = atan2(moveDir.x, moveDir.z) * RAD2DEG;

    if (!allowForward) return;

    float moveAmount = speed * speedScale * dt;
    if (moveAmount > distToTarget) moveAmount = distToTarget;

    Vector3 stepv = Vector3Scale(moveDir, moveAmount);
    position = Vector3Add(position, stepv);
}


void Vehicle::setLaneOffset(float offset) {
    laneOffsetDefault = offset;
    laneOffsetTarget  = offset;
    laneOffset        = offset;
    yieldHold         = 0.0f;
}

void Vehicle::nudgeLaneOffset(float targetOffset, float holdSeconds) {
    laneOffsetTarget = Clamp(targetOffset, -laneOffsetMax, laneOffsetMax);
    yieldHold = std::max(yieldHold, holdSeconds);
}

void Vehicle::setPath(const std::vector<Vector3>& newPath) {
    path = newPath;
    currentWp = 0;
    if (!path.empty()) destination = path[0];
    
}

void Vehicle::update() {
    step(GetFrameTime(), true);
}

void Vehicle::draw() const {
    rlPushMatrix();
    rlTranslatef(position.x, position.y, position.z);
    rlRotatef(rotation, 0, 1, 0); 

    // --- 1. CHASSIS (LOWER BODY) ---
    // Wide, long, and low. Creates the hood and trunk.
    // y = 0.2f (Low to ground) | Height = 0.25f
    DrawCube({0, 0.2f, 0}, 0.65f, 0.25f, 1.1f, color);
    DrawCubeWires({0, 0.2f, 0}, 0.65f, 0.25f, 1.1f, DARKGRAY);

    // --- 2. CABIN (UPPER BODY) ---
    // Narrower and shorter. Sits on top of the chassis.
    // y = 0.45f | Height = 0.2f
    DrawCube({0, 0.42f, -0.05f}, 0.5f, 0.2f, 0.55f, color);
    DrawCubeWires({0, 0.42f, -0.05f}, 0.5f, 0.2f, 0.55f, DARKGRAY);

    // --- 3. GLASS (WINDOWS) ---
    // SKYBLUE rectangles on the front and back of the cabin
    Color glassColor = SKYBLUE;
    
    // Windshield (Front)
    DrawCube({0, 0.42f, 0.23f}, 0.4f, 0.15f, 0.05f, glassColor);
    
    // Rear Window (Back)
    DrawCube({0, 0.42f, -0.33f}, 0.4f, 0.15f, 0.05f, glassColor);

    // --- 4. WHEELS (BLACK) ---
    Color wheelColor = BLACK;
    float wSize = 0.22f;  
    float wY = 0.1f;    
    float wX = 0.32f;    // Stick out slightly
    float wZ = 0.35f;    // Axle position

    DrawCube({ wX, wY,  wZ}, 0.12f, wSize, wSize, wheelColor); // Front Left
    DrawCube({-wX, wY,  wZ}, 0.12f, wSize, wSize, wheelColor); // Front Right
    DrawCube({ wX, wY, -wZ}, 0.12f, wSize, wSize, wheelColor); // Back Left
    DrawCube({-wX, wY, -wZ}, 0.12f, wSize, wSize, wheelColor); // Back Right

    // --- 5. LIGHTS ---
    // Headlights (Front of Chassis)
    DrawCube({ 0.2f, 0.2f, 0.56f}, 0.15f, 0.1f, 0.05f, RAYWHITE);
    DrawCube({-0.2f, 0.2f, 0.56f}, 0.15f, 0.1f, 0.05f, RAYWHITE);

    // Taillights (Back of Chassis)
    DrawCube({ 0.2f, 0.2f, -0.56f}, 0.15f, 0.1f, 0.05f, RED);
    DrawCube({-0.2f, 0.2f, -0.56f}, 0.15f, 0.1f, 0.05f, RED);

    rlPopMatrix();
}

void Vehicle::yieldTo(Vector3 emergencyPos, Vector3 emergencyDir, float yieldStrength, float yieldRadius) {
    Vector3 toMe = Vector3Subtract(position, emergencyPos);
    float d = Vector3Length(toMe);

    if (d > yieldRadius || d < 0.001f) return;

    Vector3 toMeNorm = Vector3Normalize(toMe);
    if (Vector3DotProduct(toMeNorm, emergencyDir) < -0.2f) return;

    Vector3 myForward = { sinf(rotation * DEG2RAD), 0.0f, cosf(rotation * DEG2RAD) };
    if (Vector3DotProduct(myForward, emergencyDir) < 0.0f) return;

   
    float side = (laneOffsetDefault >= 0.0f) ? 1.0f : -1.0f;

   
    float targetOffset = side * 1.3f; 

    float t = 1.0f - (d / yieldRadius);
    if (t < 0.0f) t = 0.0f;

  
    laneOffsetTarget = laneOffsetTarget + (targetOffset - laneOffsetTarget) * Clamp(t * yieldStrength * 4.0f, 0.0f, 1.0f);
    
    yieldHold = std::max(yieldHold, 1.5f);
   
}