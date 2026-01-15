#ifndef CITY_GRAPH_H
#define CITY_GRAPH_H

#include "Node.h"
#include "raymath.h"
#include <map>
#include <queue>
#include <algorithm>
#include "rlgl.h" 

class CityGraph {
public:
    std::map<std::string, Node> nodes;

    void AddNode(std::string id, Vector3 pos, NodeType type) {
        nodes[id] = Node(id, pos, type);
    }

    void Connect(std::string idA, std::string idB) {
        if (nodes.count(idA) && nodes.count(idB)) {
            nodes[idA].neighbors.push_back(idB);
        }
    }

    std::vector<Vector3> GetShortestPath(std::string startID, std::string endID) {
        std::map<std::string, float> distances;
        std::map<std::string, std::string> previous;
        auto cmp = [](std::pair<float, std::string> a, std::pair<float, std::string> b) { return a.first > b.first; };
        std::priority_queue<std::pair<float, std::string>, std::vector<std::pair<float, std::string>>, decltype(cmp)> pq(cmp);

        for (auto const& [id, node] : nodes) distances[id] = 1e6;
        distances[startID] = 0;
        pq.push({0.0f, startID});

        while (!pq.empty()) {
            std::string current = pq.top().second;
            pq.pop();
            if (current == endID) break;

            for (const std::string& neighbor : nodes[current].neighbors) {
                float dist = Vector3Distance(nodes[current].position, nodes[neighbor].position);
                if (distances[current] + dist < distances[neighbor]) {
                    distances[neighbor] = distances[current] + dist;
                    previous[neighbor] = current;
                    pq.push({distances[neighbor], neighbor});
                }
            }
        }

        std::vector<Vector3> path;
        std::string curr = endID;
        if (previous.find(curr) == previous.end() && startID != endID) return path;
        while (curr != startID) {
            path.push_back(nodes[curr].position);
            curr = previous[curr];
        }
        path.push_back(nodes[startID].position);
        std::reverse(path.begin(), path.end());
        return path;
    }

    // --- 3D VISUALS CORRIGÉS (Hauteur 0) ---
    void DrawDebug() {
        rlDisableDepthTest(); // Draw through buildings if needed

        for (auto const& [id, node] : nodes) {
            // Colors based on type
            Color c = GREEN; 
            if (node.type == INVERSE) c = RED;
            if (node.type == CIRCULAR) c = BLUE;

            // CORRECTION : On utilise 0.0f (ou node.position.y qui est 0)
            Vector3 floatPos = { node.position.x, 0.0f, node.position.z };

            // On dessine la sphère
            DrawSphere(floatPos, 0.6f, c); 
            
            for (const std::string& neighborID : node.neighbors) {
                if (nodes.count(neighborID)) {
                    Vector3 nPos = nodes.at(neighborID).position;
                    // CORRECTION : Hauteur 0.05f pour la ligne (éviter le z-fighting)
                    Vector3 floatNPos = { nPos.x, 0.05f, nPos.z };
                    Vector3 startLine = { floatPos.x, 0.05f, floatPos.z };
                    
                    DrawLine3D(startLine, floatNPos, YELLOW);
                }
            }
        }
        rlEnableDepthTest();
    }

    // --- 2D TEXT LABELS (Must be called OUTSIDE BeginMode3D) ---
    void DrawLabels(Camera3D camera) {
        for (auto const& [id, node] : nodes) {
            // CORRECTION : On met le label juste au dessus du sol (0.5f) au lieu de 3.5f
            Vector3 floatPos = { node.position.x, 0.5f, node.position.z }; 
            
            // Convert 3D world position to 2D screen coordinates
            Vector2 screenPos = GetWorldToScreen(floatPos, camera);
            
            // Only draw if it's on screen (simple check)
            if (screenPos.x > 0 && screenPos.x < GetScreenWidth() && 
                screenPos.y > 0 && screenPos.y < GetScreenHeight()) {
                
                // Draw Black text with White outline for readability
                DrawText(id.c_str(), (int)screenPos.x - 10, (int)screenPos.y, 20, BLACK);
                DrawText(id.c_str(), (int)screenPos.x - 11, (int)screenPos.y - 1, 20, WHITE);
            }
        }
    }

    std::string GetNodeClicked(Ray ray) {
        for (auto const& [id, node] : nodes) {
            // CORRECTION : La détection de clic se fait maintenant au sol (0.0f)
            Vector3 floatPos = { node.position.x, 0.0f, node.position.z };
            
            // J'augmente légèrement le rayon de collision (1.5f) pour faciliter le clic
            if (GetRayCollisionSphere(ray, floatPos, 1.5f).hit) return id;
        }
        return "";
    }
};

#endif