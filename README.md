# 🚦 Smart City Simulation - Hybrid Edition

A sophisticated 3D traffic simulation built with **C++** and **Raylib**. This project represents a technical merger of two distinct simulation architectures: combining a high-fidelity **3D Grid Map** with an abstract **Node-Based Graph System** for intelligent pathfinding.

## ✨ Key Functionalities

### 1. 🧠 Hybrid Navigation System (The "Ghost Graph")
* **Automatic Graph Generation:** The system scans the visual 3D grid and generates an invisible "Ghost Graph" on top of drivable roads.
* **Strict Road Adherence:** Graph nodes are strictly generated only on valid roads (excluding parking spots and grass) to prevent illegal shortcuts.
* **Path Refinement:** Uses a custom `RefinePath` algorithm that converts abstract graph connections (Node A → Node B) into physical tile-by-tile movements, ensuring cars curve correctly around buildings.

### 2. 🎥 Advanced Camera System
* **Orbit Mode:** Free-roaming camera to view the city from any angle.
* **Smart Follow Mode:** Locks onto the Ambulance or any civilian vehicle.
* **Occlusion Detection:** The camera automatically zooms in if a building blocks the view of the target vehicle.
* **Target Cycling:** Seamlessly switch the camera focus between different vehicles in the scene.

### 3. 🚗 Dynamic Traffic & Spawning
* **UI Spawn System:** Interactive button to spawn vehicles dynamically.
* **Smart Placement:** New cars are spawned on valid graph loops, ensuring they don't overlap with existing traffic.
* **Anti-Deadlock Physics:** Vehicles detect if they are stuck behind a stopped car (even if the light is green) and perform a "nudge" maneuver to change lane offsets.

### 4. 🚑 Emergency Vehicle Priority
* **Siren Logic:** When the ambulance moves, nearby traffic detects the siren.
* **Yielding Behavior:** Civilian cars autonomously pull over to the edge of their lane (Lane Offset) and stop to create an emergency corridor.
* **Traffic Light Override:** Lights automatically turn green for the approaching ambulance.

---

## 🎮 Controls

| Key / Input | Action |
| :--- | :--- |
| **W / S** | Zoom Camera In / Out |
| **A / D** | Rotate Camera Left / Right |
| **C** | **Toggle Follow Camera** (Lock onto vehicle) |
| **TAB** | **Cycle Target** (Switch between Ambulance & Cars) |
| **R** | **Reset Camera** to Center View |
| **Left Click** | Set Ambulance Destination (on any road) |
| **G** | **GO!** Start Ambulance Mission |
| **Space** | Pause / Resume Simulation |
| **UI Buttons** | Top-Right clickable buttons to **Spawn** or **Reset** |

---

## 🛠️ Technical Architecture

This project solves the problem of combining visual realism with logical pathfinding through three main components:

1.  **`CityMap` (The Visuals):** Handles the 10x10 grid, 3D building rendering, and texture mapping. It defines *where* things are physically.
2.  **`CityGraph` (The Brain):** An abstract data structure of Nodes and Edges. It calculates the shortest path using Dijkstra's algorithm.
3.  **`Simulation` (The Bridge):**
    * **Init:** Maps 3D tiles to Graph Nodes (e.g., Tile `4_2` becomes Node `"4_2"`).
    * **Update:** Translates the logical path into `Vector3` movement vectors.
    * **Debug:** Visualizes the graph with Green Spheres (Roads) and Blue Spheres (Intersections).

---

## 🚀 How to Run

### Prerequisites
* C++ Compiler (GCC/MSVC)
* CMake
* Raylib

### Build Instructions

1.  **Clone the Repository**
    ```bash
    git clone [https://github.com/RaylibSmartCity-Devs/CitySmart-.git](https://github.com/RaylibSmartCity-Devs/CitySmart-.git)
    cd CitySmart-
    ```

2.  **Build**
    ```bash
    mkdir build
    cd build
    cmake ..
    cmake --build . --config Release
    ```

3.  **Run**
    * Run `CitySmart.exe` from the `build/Release` folder.
    * *Note:* Ensure the `assets/` folder is in the same directory as the executable.

---

## 👥 Authors
**Grp 16 EMERGENCY & PRIORITY MANAGEMENT**
* **Ayoub Chentouf**
* **Ayoub Hmamouchi**
* **Aya Lmasoudi**
* **Charaf Eddine Ifrinchaou**
* **Kerouad Loubna** 
---------------------------
**Grp 3 TRAFFIC CORE**
* **CHOROUK MOUBSSIR**
* **OUSSAMA JABIR**
* **AYA EL ISSAOUI**
* **ZINEB KHBIAZ**