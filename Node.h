#ifndef NODE_H
#define NODE_H

#include "raylib.h"
#include <string>
#include <vector>

enum NodeType { DIRECT, INVERSE, CIRCULAR };


struct Node {
    std::string id;
    Vector3 position;
    NodeType type;
    std::vector<std::string> neighbors; 

    Node() {}
    Node(std::string _id, Vector3 _pos, NodeType _type) : id(_id), position(_pos), type(_type) {}
};

#endif