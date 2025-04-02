.. _module1-setup-basics:

Module 1: ROS 2 Setup & Basics
==============================

This module covers the essential first steps to start working with ROS 2.

.. contents:: Topics Covered
   :local:
   :depth: 2

1. Introduction to ROS 2
------------------------

What is ROS 2?
~~~~~~~~~~~~~~
ROS 2 (Robot Operating System 2) is a framework for developing robot software. It provides:

- Communication tools (Nodes, Topics, Services, Actions)
- Hardware abstraction
- Package management
- Cross-platform support

Key Concepts:
~~~~~~~~~~~~~
.. list-table::
   :header-rows: 1
   :widths: 20 50

   * - Concept
     - Description
   * - Nodes
     - Independent executables that perform specific tasks
   * - Topics
     - Asynchronous pub/sub communication channels
   * - Services
     - Synchronous request/reply interactions
   * - Actions
     - Long-running tasks with feedback

2. Verify Installation
----------------------

Before proceeding, ensure ROS 2 is properly installed:

.. code-block:: bash

   # Test C++ demo nodes
   ros2 run demo_nodes_cpp talker
   # In another terminal
   ros2 run demo_nodes_cpp listener

Expected Output:
~~~~~~~~~~~~~~~
You should see the talker publishing messages and the listener receiving them.

Troubleshooting:
~~~~~~~~~~~~~~~
- If commands aren't found, verify sourcing:
  
  .. code-block:: bash

     source /opt/ros/humble/setup.bash  # or 'jazzy' for Ubuntu 24.04

3. Workspace & Build System
---------------------------

Create a Workspace:
~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws

Build System (colcon):
~~~~~~~~~~~~~~~~~~~~~
ROS 2 uses colcon as its build tool. Basic commands:

.. code-block:: bash

   # Build all packages in workspace
   colcon build
   # Build specific package
   colcon build --packages-select <package_name>
   # Build with symlink (recommended for development)
   colcon build --symlink-install

Source the Workspace:
~~~~~~~~~~~~~~~~~~~~
After building:

.. code-block:: bash

   source ~/ros2_ws/install/setup.bash

Next Steps:
-----------
Proceed to :doc:`Module 2: Core Concepts </Module2/module2>` for a deeper dive.
More content `ROS 2 Begginers: CLI tools<https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html>`_
Explore the `ROS 2 Documentation <https://docs.ros.org/>`_

.. note::
   Remember to source your ROS 2 installation and workspace in every new terminal!
