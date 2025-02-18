# Simple Planner Project

This project implements a simple path planner using **Dijkstra's algorithm** with ROS.

## Setup Instructions

### 1. Open 4 terminals and execute the following steps:

#### **Terminal 1: Start ROS Core**
```bash
roscore
```
#### **Terminal 2: Run the Map Server**
```bash
rosrun map_server map_server map.yaml
```
#### **Terminal 3: Launch the Planner Node**
```bash
rosrun simple_planner simple_planner
```
#### **Terminal 4: Start RViz**
```bash
rviz
```
### RViz Setup
Once RViz is open, perform the following steps:

1. **Add topics:**
   - `map`
   - `marker`
2. **Set the start position on the map** using **2D Pose Estimate**.
3. **Set the goal position on the map** using **2D Nav Goal**.
