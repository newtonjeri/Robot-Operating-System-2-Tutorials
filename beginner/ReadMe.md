# **ROS 2 Beginner Tutorials** 

This folder contains step-by-step tutorials to master the fundamentals of ROS 2. No prior ROS experience required!  

---

## **Prerequisites**  
Before starting, ensure you have:  
1. **Ubuntu 22.04 (ROS 2 Humble)** or **Ubuntu 24.04 (ROS 2 Jazzy)**.  
2. **ROS 2 Installed**: Follow the official guides:  
   - [ROS 2 Humble (Ubuntu 22.04)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)  
   - [ROS 2 Jazzy (Ubuntu 24.04)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)  
3. Basic knowledge of:  
   - Linux terminal commands.  
   - Python **or** C++ (examples provided for both).  

---

## **Course Overview**  
### **Module 1: ROS 2 Setup & Basics**  
1. **Introduction to ROS 2**  
   - What is ROS 2? Key concepts (Nodes, Topics, Services).  
2. **Verify Installation**  
   - Test ROS 2 with `ros2 run demo_nodes_cpp talker/listener`.  
3. **Workspace & Build System**  
   - Create a workspace, build packages with `colcon`.  

### **Module 2: Core Concepts**  
4. **Your First ROS 2 Package**  
   - Structure, dependencies (`package.xml`, `CMakeLists.txt`).  
5. **Nodes & Topics**  
   - Publisher/Subscriber in **Python** and **C++**.  
6. **Services**  
   - Service/Client in **Python** and **C++**.  

### **Module 3: Custom Interfaces & Parameters**  
7. **Custom Messages/Services**  
   - Define `.msg` and `.srv` files.  
8. **Parameter Management**  
   - Declare and update parameters dynamically.  

### **Module 4: Tools & Debugging**  
9. **ROS 2 CLI Tools**  
   - `ros2 node`, `topic`, `service`, `param`.  
10. **Debugging**  
    - `ros2doctor`, `rqt_graph`, and common pitfalls.  

### **Module 5: Mini Project**  
11. **Turtlesim Controller**  
    - Combine topics/services to control a simulated robot.  

---

## **How to Use This Tutorial**  
1. **Follow Along**:  
   - Start with [Module 1](./1_workspace_basics/) and progress sequentially.  
   - Code examples are provided in **Python** and **C++** (choose your preference).  
2. **Need Help?**  
   - Check the [ROS 2 Documentation](https://docs.ros.org/) or open an issue.  

---

## **Folder Structure**  
```bash
beginners/  
├── 1_workspace_basics/       # Verify installation, colcon, workspace setup  
├── 2_publisher_subscriber/   # Topics (Python/C++)  
├── 3_services/               # Services (Python/C++)  
├── 4_custom_interfaces/      # Custom msg/srv  
├── 5_parameters/             # Dynamic parameters  
├── 6_tools_debugging/        # CLI tools, ros2doctor  
└── 7_mini_project/           # Turtlesim controller project  
```

---

## **Troubleshooting**  
- **ROS 2 Not Sourced?** Run:  
  ```bash
  source /opt/ros/<distro>/setup.bash  # Replace <distro> with 'humble' or 'jazzy'
  ```  
- **Package Not Found?** Ensure your workspace is built and sourced:  
  ```bash
  colcon build && source install/setup.bash
  ```  

---

## **License**  
This content is licensed under the [MIT License](../LICENSE). Contributions welcome!  

---

### **Ready to Start?**  
Jump to [Module 1: ROS 2 Setup & Basics](./1_workspace_basics/)!  

---