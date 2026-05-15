# SC2079-MDP-GROUP 24

An autonomous robotics platform developed for the SC2079 Multi-Disciplinary Design Project (MDP). The system integrates an Android control application, Raspberry Pi coordination layer, STM32 motion controller, and navigation/image recognition algorithms to autonomously navigate an arena and detect image targets.

---

# System Architecture

```text
Android Tablet
      ⇅ Bluetooth
Raspberry Pi
      ⇅ Serial
STM32 Microcontroller
```

- **Android App** — User interface, obstacle configuration, telemetry display
- **Raspberry Pi** — Communication hub, image recognition, task coordination
- **STM32** — Motor control and motion execution
- **Algorithm Module** — Path computation and navigation planning

---

# Key Features

## Android Application
- Interactive 2D grid map with drag-and-drop obstacle placement
- Robot position, heading visualisation, and obstacle face annotation
- Real-time robot telemetry updates and an internal task timer
- `START COMPUTATION` functionality for route planning
- One-tap map reset

### Batch-Sync Protocol
Users can configure the entire arena on the tablet before syncing all obstacle and robot data to the Raspberry Pi in a single Bluetooth transmission. This reduces communication overhead and improves setup efficiency.

---

# System Workflow

## Image Recognition Task

1. User connects Android tablet to Raspberry Pi via Bluetooth
2. Obstacles and image-facing directions are configured on the map via touch-and-drag interactions
3. Android sends obstacle data to the RPi
4. User taps `START COMPUTATION`
5. RPi forwards obstacle data to the algorithm module
6. Algorithm computes an optimised navigation path
7. RPi receives computed path and returns `COMPLETED`
8. User taps `START TASK 1`
9. Android sends `START_EXPLORE`
10. Robot navigates arena while Android receives live updates:
    - `ROBOT,x,y,direction`
    - `TARGET,obstacleID,targetID`
11. RPi sends `END` after all images are detected
12. Android stops the internal timer and displays completion status

---

# Navigation Algorithm

Initial movement logic using simple grid-based motions was unreliable due to turning constraints and tight obstacle placements.

The final algorithm:
- Computes efficient navigation paths
- Considers turning radius constraints
- Supports angular turning
- Optimises traversal order and travel distance
- Improves reliability in confined spaces

---

# Raspberry Pi Subsystem

The Raspberry Pi acts as the central coordinator:
- Handles Bluetooth communication with Android
- Communicates with STM32 via serial
- Runs YOLOv11 image recognition trained on self-collected datasets and Roboflow datasets from previous MDP groups
- Processes camera input from PiCamera
- Relays commands and telemetry between subsystems

---

# STM32 Motion Controller

The STM32 is responsible for:
- Motor control
- Speed regulation
- Accurate turning
- Motion correction
- Sensor feedback handling

The controller compensates for:
- Battery voltage variation
- Surface friction
- Robot inertia

to maintain consistent and accurate movement.

---

# Tech Stack

## Android
- Android Studio
- Java
- Bluetooth Serial Communication

## Raspberry Pi
- Python
- OpenCV
- YOLOv11
- PiCamera

## STM32
- Embedded C
- UART Serial Communication
- PID Motion Control

## Algorithm
- Hamiltonian Path Planning
- Heuristic Optimisation
- Constraint-Based Navigation

---

# Future Improvements
- Dynamic obstacle avoidance
- Faster image recognition pipeline
- Continuous movement during image detection
- Live camera streaming to Android
- Smarter path recomputation

---

# Conclusion

The project combines robotics, embedded systems, computer vision, and mobile application development into a fully autonomous exploration and image recognition platform capable of efficient navigation and real-time system coordination.
