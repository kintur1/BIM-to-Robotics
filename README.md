
# 📐🤖 BIM to Robotics for Intelligent Building Automation

**Master’s Thesis – RWTH Aachen University**  
*M.Sc. Civil Engineering (ADCOM)*

---

## 🧠 Abstract

This thesis bridges the gap between **Building Information Modeling (BIM)** and **robotic automation** to lay the foundation for intelligent building construction and maintenance workflows. The project focuses on the **conversion of IFC-based BIM models into simulation-ready robot environments**, enabling mobile and industrial manipulators to interact with and assemble building components in both virtual and real-world environments.

The main objective is to develop a fully functional **pipeline from BIM to Robotics**, using **IFC → URDF/SDF → ROS 2 → MoveIt → Gazebo Ignition**, enabling robotic task planning, path execution, and realistic manipulation of building elements.

---

## 🎯 Objectives

- Export and preprocess BIM data in **IFC format**
- Convert **IFC → SDF/URDF** for simulation compatibility
- Design a **ROS 2-based middleware** to control robot interaction with BIM environments
- Develop and test **Pick-and-Place** pipelines in simulation using **MoveIt**, **Gazebo Ignition**, and **ROS 2 Actions**
- Enable **object manipulation** (attach/detach) and **collision checking** using both **RViz** and **Gazebo**

---

## 🛠️ Technologies & Tools

| Tool/Technology        | Purpose                                        |
|------------------------|------------------------------------------------|
| **IFC**                | Building Information Modeling data             |
| **IFC Open Shell**     | Python-based IFC parsing and processing        |
| **SDF / URDF**         | Environment and robot description formats      |
| **ROS 2 Humble**       | Robot middleware (Python and C++)              |
| **MoveIt 2**           | Motion planning and manipulation               |
| **Gazebo Ignition**    | Physics-based simulation                       |
| **RViz 2**             | Robot visualization and PlanningScene editing  |
| **Ubuntu 22.04**       | Development environment                        |
| **Python 3 / C++**     | Code implementation                            |

---

## 🧩 Key Components & Workflow

### 1. IFC Export and Preprocessing

- Exported IFC models from **Revit**/BIM tools
- Parsed using `ifcopenshell` to extract:
  - Geometries (walls, pipes, columns)
  - Transformations, metadata, and hierarchy
- Converted solids into `.dae`/`.stl` formats for use in URDF/SDF

---

### 2. IFC to URDF/SDF Conversion

- Created **custom Python scripts** to:
  - Generate **URDFs** for robot-compatible models
  - Generate **SDFs** for full environmental structures
  - Handle link/joint assignments, origins, and visuals
- Batch processing support added for large IFC files

---

### 3. ROS 2 Integration

- Built ROS 2 packages for:
  - **Launching robots and environments**
  - **Publishing component transforms**
  - **Attaching/detaching components**
- Used **RCLPy + RCLCpp** to:
  - Control and monitor robot state
  - Connect Gazebo and MoveIt environments

---

### 4. Gazebo Ignition Setup

- Imported SDF models into **Gazebo Ignition Fortress**
- Developed custom **C++ plugin** and **ROS 2 interface** for:
  - `attach_link`, `detach_link` services
  - Simulating physics-based grasp and release
- Linked pipe models to robot gripper for accurate behavior

---

### 5. Robot Setup and MoveIt Planning

- Used **UR5e robot + custom gripper**
- Designed full URDF with `transmission`, `controller`, and `ros2_control` tags
- Configured **MoveIt 2** with:
  - Motion planning pipelines
  - Cartesian planners
  - Interactive markers and motion groups

---

### 6. Pick-and-Place Application

- End-to-end task sequence:
  - Load BIM-based pipe models into planning scene
  - Compute approach and grasp poses
  - Plan & execute pick and place using `MoveGroupInterface`
- ROS 2 Actions:
  - `MoveGroupActionClient` for path planning
  - `ExecuteTrajectoryActionClient` for trajectory following

---

### 7. Attach/Detach in RViz and Gazebo

- **RViz Integration**:
  - Used `PlanningSceneInterface` to add/remove collision objects
  - Attached and detached objects from `wrist_3_link`
- **Gazebo Integration**:
  - Developed C++ plugin with Ignition Physics API
  - Exposed ROS 2 services for:
    - `attach_pipe_to_gripper`
    - `detach_pipe_from_gripper`
  - Tested in real-time simulation with visual confirmation

---

## 🧪 Testing and Validation

- ✅ Validated full task pipeline in **RViz 2** and **MoveIt 2**
- ✅ Confirmed physical accuracy in **Gazebo Ignition**
- ✅ Verified TF synchronization between robot and objects
- ✅ Stress-tested with various object sizes and positions
- ✅ Handled edge cases: unreachable poses, collision paths

---

## 💻 Repository Structure

```bash
bim_to_robotics/
│
├── ifc_parser/                 # Scripts to convert IFC to URDF/SDF
├── urdf/                       # Robot description files
├── sdf/                        # BIM-based environments
├── src/
│   ├── pick_place/             # ROS 2 node for pick-and-place
│   ├── attach_detach/          # Services for Gazebo link control
│   └── moveit_interface/       # Action clients and MoveGroup control
├── launch/                     # Launch files for RViz, Gazebo, robot
├── config/                     # RViz & MoveIt config
├── gazebo_plugins/             # C++ plugins for attachment simulation
├── meshes/                     # STL/DAE geometry converted from IFC
└── README.md                   # Project documentation
```

---

## 🧱 Challenges and Solutions

| Challenge | Solution |
|----------|----------|
| IFC file complexity and missing physics metadata | Used `ifcopenshell.util` for metadata and manually enriched object properties |
| Lack of IFC to SDF/URDF converters | Developed custom converter script using geometry exporters |
| No native attach/detach plugin in Ignition | Built custom C++ plugin with Gazebo Ignition Physics API |
| PlanningScene collisions and object sync | Tuned padding, updated scene at runtime via PlanningSceneInterface |
| IK failures on complex poses | Used Cartesian planning and iterative re-planning |

---

## 📽️ Demo

> Videos and GIFs showcasing robot picking and placing BIM components in Gazebo and RViz (TBD)

---

## 📌 Conclusion

This project establishes a working prototype for integrating **BIM models into robotic control systems**, allowing **IFC-based building elements to be manipulated in realistic simulation environments**. It paves the way for future smart construction workflows, where robots interact with design data directly to perform real-world assembly, inspection, and maintenance.

---

## 📬 Contact

**Author:** [Your Name]  
**Email:** [your.email@rwth-aachen.de]  
**Institution:** RWTH Aachen University, Germany

---

## 📄 License

This repository is released under the MIT License. See `LICENSE` file for details.
