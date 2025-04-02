.. _module2-core-concepts:

Module 2: Core Concepts
=======================

This module dives into fundamental ROS 2 concepts with hands-on implementations.

.. contents:: 
   :local:
   :depth: 2

4. Your First ROS 2 Package
---------------------------

Package Structure
~~~~~~~~~~~~~~~~~
A typical ROS 2 package contains:

.. code-block:: text

   my_first_package/
   ├── CMakeLists.txt
   ├── package.xml
   ├── include/
   ├── src/
   └── launch/

Creating a Package
~~~~~~~~~~~~~~~~~
Python package:

.. code-block:: bash

   ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy

C++ package:

.. code-block:: bash

   ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp

Key Files Explained
~~~~~~~~~~~~~~~~~~

package.xml
^^^^^^^^^^^
.. code-block:: xml

   <!-- Example for Python -->
   <exec_depend>rclpy</exec_depend>
   <!-- Example for C++ -->
   <depend>rclcpp</depend>

CMakeLists.txt (C++)
^^^^^^^^^^^^^^^^^^^^
Essential sections:

.. code-block:: cmake

   find_package(rclcpp REQUIRED)
   add_executable(my_node src/my_node.cpp)
   ament_target_dependencies(my_node rclcpp)
   install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})

5. Nodes & Topics
-----------------

Publisher-Subscriber Model
~~~~~~~~~~~~~~~~~~~~~~~~~
.. figure:: /images/pubsub_model.png
   :width: 60%
   :align: center
   :alt: Pub/Sub Model

Python Implementation
~~~~~~~~~~~~~~~~~~~~

Publisher (Python):
^^^^^^^^^^^^^^^^^^
.. code-block:: python

   #!/usr/bin/env python3
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String

   class MyPublisher(Node):
       def __init__(self):
           super().__init__('my_publisher')
           self.publisher = self.create_publisher(String, 'topic_name', 10)
           timer_period = 0.5
           self.timer = self.create_timer(timer_period, self.timer_callback)
       
       def timer_callback(self):
           msg = String()
           msg.data = 'Hello ROS 2'
           self.publisher.publish(msg)

Subscriber (Python):
^^^^^^^^^^^^^^^^^^^
.. code-block:: python

   class MySubscriber(Node):
       def __init__(self):
           super().__init__('my_subscriber')
           self.subscription = self.create_subscription(
               String,
               'topic_name',
               self.listener_callback,
               10)
       
       def listener_callback(self, msg):
           self.get_logger().info(f'I heard: "{msg.data}"')

C++ Implementation
~~~~~~~~~~~~~~~~~~

Publisher (C++):
^^^^^^^^^^^^^^^
.. code-block:: cpp

   #include "rclcpp/rclcpp.hpp"
   #include "std_msgs/msg/string.hpp"

   class MyPublisher : public rclcpp::Node {
   public:
       MyPublisher() : Node("my_publisher") {
           publisher_ = this->create_publisher<std_msgs::msg::String>("topic_name", 10);
           timer_ = this->create_wall_timer(
               500ms, std::bind(&MyPublisher::timer_callback, this));
       }
   private:
       void timer_callback() {
           auto message = std_msgs::msg::String();
           message.data = "Hello ROS 2";
           publisher_->publish(message);
       }
       rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
       rclcpp::TimerBase::SharedPtr timer_;
   };

Subscriber (C++):
^^^^^^^^^^^^^^^^
.. code-block:: cpp

   class MySubscriber : public rclcpp::Node {
   public:
       MySubscriber() : Node("my_subscriber") {
           subscription_ = this->create_subscription<std_msgs::msg::String>(
               "topic_name", 10,
               std::bind(&MySubscriber::topic_callback, this, _1));
       }
   private:
       void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
           RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
       }
       rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
   };

6. Services
-----------

Service-Client Model
~~~~~~~~~~~~~~~~~~~
.. figure:: /images/service_model.png
   :width: 60%
   :align: center
   :alt: Service Model

Python Implementation
~~~~~~~~~~~~~~~~~~~~

Service Server:
^^^^^^^^^^^^^^
.. code-block:: python

   from example_interfaces.srv import AddTwoInts

   class MyService(Node):
       def __init__(self):
           super().__init__('my_service')
           self.srv = self.create_service(
               AddTwoInts, 'add_two_ints', self.add_callback)
       
       def add_callback(self, request, response):
           response.sum = request.a + request.b
           self.get_logger().info(f'Incoming request: {request.a} + {request.b}')
           return response

Service Client:
^^^^^^^^^^^^^^
.. code-block:: python

   class MyClient(Node):
       def __init__(self):
           super().__init__('my_client')
           self.client = self.create_client(AddTwoInts, 'add_two_ints')
           
       def call_service(self, a, b):
           while not self.client.wait_for_service(timeout_sec=1.0):
               self.get_logger().info('service not available, waiting...')
           req = AddTwoInts.Request()
           req.a = a
           req.b = b
           future = self.client.call_async(req)
           rclpy.spin_until_future_complete(self, future)
           return future.result()

C++ Implementation
~~~~~~~~~~~~~~~~~~

Service Server:
^^^^^^^^^^^^^^
.. code-block:: cpp

   #include "example_interfaces/srv/add_two_ints.hpp"

   class MyService : public rclcpp::Node {
   public:
       MyService() : Node("my_service") {
           service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
               "add_two_ints",
               std::bind(&MyService::add_callback, this, _1, _2));
       }
   private:
       void add_callback(
           const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
           std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
           response->sum = request->a + request->b;
           RCLCPP_INFO(this->get_logger(), "Incoming request: %ld + %ld", request->a, request->b);
       }
       rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
   };

Service Client:
^^^^^^^^^^^^^^
.. code-block:: cpp

   class MyClient : public rclcpp::Node {
   public:
       MyClient() : Node("my_client") {
           client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
       }
       auto call_service(int64_t a, int64_t b) {
           while (!client_->wait_for_service(1s)) {
               RCLCPP_INFO(this->get_logger(), "service not available, waiting...");
           }
           auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
           request->a = a;
           request->b = b;
           auto future = client_->async_send_request(request);
           return future;
       }
   private:
       rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
   };

Best Practices
--------------
- **Naming Conventions**: Use snake_case for packages and nodes
- **QoS Settings**: Configure Quality of Service for critical applications
- **Error Handling**: Always check service availability before calls
- **Logging**: Use appropriate log levels (DEBUG, INFO, WARN, ERROR)

Next Steps
----------
- :ref:`Module 3: Custom Interfaces <module3-interfaces>`
- `ROS 2 Core Concepts Documentation <https://docs.ros.org/en/humble/Concepts.html>`_
