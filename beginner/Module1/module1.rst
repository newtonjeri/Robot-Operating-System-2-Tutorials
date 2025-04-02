.. _module1-setup-basics:

Module 1: ROS 2 Setup & Basics
==============================

This module covers the essential first steps to start working with ROS 2, including its architectural foundations.

.. contents:: Topics Covered
   :local:
   :depth: 2

1. Introduction to ROS 2
------------------------

What is ROS 2?
~~~~~~~~~~~~~~
ROS 2 is a **meta-operating system** for robotics that provides:

- **Middleware**: DDS-based communication (Fast DDS, Cyclone DDS, etc.)
- **Tooling**: Build system (colcon), debugging tools (rqt, ros2doctor)
- **Ecosystem**: Standardized interfaces and packages

Key Differences from ROS 1:
^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. list-table::
   :header-rows: 1
   :widths: 30 30 30
   
   * - Feature
     - ROS 1
     - ROS 2
   * - Middleware
     - Custom TCPROS/UDPROS
     - DDS (Standardized)
   * - Real-time
     - Limited
     - Fully supported
   * - Platform
     - Linux only
     - Cross-platform
   * - Security
     - None
     - Built-in (SROS)

Architecture Overview
~~~~~~~~~~~~~~~~~~~~
.. mermaid::
   graph TD
     A[Node] -->|Publish| B[Topic]
     A -->|Call| C[Service]
     A -->|Execute| D[Action]
     B --> E[Subscriber Node]
     C --> F[Service Server]
     D --> G[Action Server]

Key Concepts (In-Depth):
~~~~~~~~~~~~~~~~~~~~~~~~

Nodes
^^^^^
- **Fundamental processes** in ROS 2
- Single-purpose (e.g., sensor driver, controller)
- Can be written in Python, C++, or other supported languages
- Communicate via **Topics**, **Services**, or **Actions**

Topics
^^^^^^
- **Publish-Subscribe** pattern
- Asynchronous, many-to-many communication
- Data type defined by `.msg` files
- QoS (Quality of Service) policies configurable

Services
^^^^^^^^
- **Request-Reply** pattern
- Synchronous communication (client blocks)
- Defined by `.srv` files (request + response)
- Use cases: configuration changes, calculations

Actions
^^^^^^^
- **Asynchronous** with feedback
- Three-part interface (Goal, Feedback, Result)
- Built on top of topics and services
- Ideal for long-running tasks (e.g., navigation)

2. Verify Installation
----------------------

[... rest of your existing installation verification section ...]

3. Workspace & Build System
---------------------------

[... rest of your existing workspace section ...]

DDS Implementation Details
~~~~~~~~~~~~~~~~~~~~~~~~~
ROS 2 supports multiple DDS implementations:

.. list-table::
   :header-rows: 1
   :widths: 25 25 50
   
   * - DDS Vendor
     - Default For
     - Characteristics
   * - Fast DDS
     - Humble
     - Balance of performance/features
   * - Cyclone DDS
     - Galactic
     - Lightweight, real-time focus
   * - Connext DDS
     - Enterprise
     - Commercial-grade reliability

Quality of Service (QoS) Policies
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Critical for real-world robotics:

.. code-block:: yaml

   Reliability: RELIABLE  # Or BEST_EFFORT
   Durability: VOLATILE  # Or TRANSIENT_LOCAL
   History: KEEP_LAST  # Or KEEP_ALL
   Depth: 10  # Queue size

Next Steps:
-----------
- Proceed to :ref:`Module 2: Core Concepts <module2-core-concepts>`
- Deep dive: `ROS 2 Architecture Whitepaper <https://design.ros2.org/>`_
- Explore: `DDS and ROS 2 <https://docs.ros.org/en/rolling/Concepts/About-Different-Middleware-Vendors.html>`_

.. note::
   For production systems, always:
   - Set appropriate QoS profiles
   - Consider DDS vendor performance characteristics
   - Use security enclaves (SROS) for networked robots