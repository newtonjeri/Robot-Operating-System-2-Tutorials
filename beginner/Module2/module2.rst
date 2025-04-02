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

A typical ROS 2 package structure varies slightly depending on the build type:

ament_cmake (C++ Packages)
^^^^^^^^^^^^^^^^^^^^^^^^^^
.. code-block:: text

   package_name/
   ├── CMakeLists.txt          # Build configuration
   ├── package.xml             # Package metadata and dependencies
   ├── include/                # Header files (public interface)
   │   └── package_name/       # Namespaced headers
   ├── src/                    # Source files
   └── launch/                 # Launch files

ament_python (Python Packages)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. code-block:: text

   package_name/
   ├── package.xml             # Package metadata and dependencies
   ├── setup.py                # Python build configuration
   ├── setup.cfg               # Additional build settings
   ├── resource/               # Package resources
   ├── test/                   # Test files
   ├── launch/                 # Launch files
   └── package_name/           # Python module
       ├── __init__.py        # Module initialization
       └── nodes/             # Node implementations

Creating a Package
~~~~~~~~~~~~~~~~~
Python package:

.. code-block:: bash

   ros2 pkg create <package_name> --build-type ament_python --dependencies rclpy

C++ package:

.. code-block:: bash

   ros2 pkg create <package_name> --build-type ament_cmake --dependencies rclcpp

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
.. figure:: https://docs.ros.org/en/jazzy/_images/Topic-MultiplePublisherandMultipleSubscriber.gif
   :width: 80%
   :align: center
   :alt: Pub/Sub Model

Python Implementation
~~~~~~~~~~~~~~~~~~~~

Reference: `Writing a simple publisher and subscriber (Python) <https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html>`_

Publisher (Python):
^^^^^^^^^^^^^^^^^^
.. code-block:: python

   import rclpy
   from rclpy.node import Node

    from std_msgs.msg import String

    class MinimalPublisher(Node):

        def __init__(self):
            super().__init__('minimal_publisher')
            self.publisher_ = self.create_publisher(String, 'topic', 10)
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.i = 0

        def timer_callback(self):
            msg = String()
            msg.data = 'Hello World: %d' % self.i
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1


    def main(args=None):
        rclpy.init(args=args)

        minimal_publisher = MinimalPublisher()

        rclpy.spin(minimal_publisher)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_publisher.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

Subscriber (Python):
^^^^^^^^^^^^^^^^^^^
.. code-block:: python

    import rclpy
    from rclpy.node import Node

    from std_msgs.msg import String


    class MinimalSubscriber(Node):

        def __init__(self):
            super().__init__('minimal_subscriber')
            self.subscription = self.create_subscription(
                String,
                'topic',
                self.listener_callback,
                10)
            self.subscription  # prevent unused variable warning

        def listener_callback(self, msg):
            self.get_logger().info('I heard: "%s"' % msg.data)


    def main(args=None):
        rclpy.init(args=args)

        minimal_subscriber = MinimalSubscriber()

        rclpy.spin(minimal_subscriber)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

C++ Implementation
~~~~~~~~~~~~~~~~~~
Reference `Writing a simple publisher and subscriber (C++) <https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html>`_

Publisher (C++):
^^^^^^^^^^^^^^^
.. code-block:: cpp

    #include <chrono>
    #include <functional>
    #include <memory>
    #include <string>

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"

    using namespace std::chrono_literals;

    /* This example creates a subclass of Node and uses std::bind() to register a
    * member function as a callback from the timer. */

    class MinimalPublisher : public rclcpp::Node
    {
    public:
        MinimalPublisher()
        : Node("minimal_publisher"), count_(0)
        {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
        }

    private:
        void timer_callback()
        {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
    };

    int main(int argc, char * argv[])
    {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
    }

Subscriber (C++):
^^^^^^^^^^^^^^^^
.. code-block:: cpp

    #include <memory>

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"
    using std::placeholders::_1;

    class MinimalSubscriber : public rclcpp::Node
    {
    public:
        MinimalSubscriber()
        : Node("minimal_subscriber")
        {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
        }

    private:
        void topic_callback(const std_msgs::msg::String & msg) const
        {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    };

    int main(int argc, char * argv[])
    {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
    }

6. Services
-----------

Service-Client Model
~~~~~~~~~~~~~~~~~~~
.. figure:: https://docs.ros.org/en/jazzy/_images/Service-MultipleServiceClient.gif
   :width: 80%
   :align: center
   :alt: Service Model

Python Implementation
~~~~~~~~~~~~~~~~~~~~
Reference `Writing a simple service and client (Python) <https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html#writing-a-simple-service-and-client-python>`_

Service Server:
^^^^^^^^^^^^^^
.. code-block:: python

    from example_interfaces.srv import AddTwoInts

    import rclpy
    from rclpy.node import Node

    class MinimalService(Node):

        def __init__(self):
            super().__init__('minimal_service')
            self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

        def add_two_ints_callback(self, request, response):
            response.sum = request.a + request.b
            self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

            return response


    def main():
        rclpy.init()

        minimal_service = MinimalService()

        rclpy.spin(minimal_service)

        rclpy.shutdown()


    if __name__ == '__main__':
        main()

Service Client:
^^^^^^^^^^^^^^
.. code-block:: python

    import sys

    from example_interfaces.srv import AddTwoInts
    import rclpy
    from rclpy.node import Node


    class MinimalClientAsync(Node):

        def __init__(self):
            super().__init__('minimal_client_async')
            self.cli = self.create_client(AddTwoInts, 'add_two_ints')
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.req = AddTwoInts.Request()

        def send_request(self, a, b):
            self.req.a = a
            self.req.b = b
            return self.cli.call_async(self.req)


    def main():
        rclpy.init()

        minimal_client = MinimalClientAsync()
        future = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
        rclpy.spin_until_future_complete(minimal_client, future)
        response = future.result()
        minimal_client.get_logger().info(
            'Result of add_two_ints: for %d + %d = %d' %
            (int(sys.argv[1]), int(sys.argv[2]), response.sum))

        minimal_client.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()

C++ Implementation
~~~~~~~~~~~~~~~~~~
Reference `Writing a simple service and client (C++) <https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html>`_

Service Server:
^^^^^^^^^^^^^^
.. code-block:: cpp

    #include "rclcpp/rclcpp.hpp"
    #include "example_interfaces/srv/add_two_ints.hpp"

    #include <memory>

    void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
            std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
    {
    response->sum = request->a + request->b;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                    request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
    }

    int main(int argc, char **argv)
    {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
        node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    }

Service Client:
^^^^^^^^^^^^^^
.. code-block:: cpp 

    #include "rclcpp/rclcpp.hpp"
    #include "example_interfaces/srv/add_two_ints.hpp"

    #include <chrono>
    #include <cstdlib>
    #include <memory>

    using namespace std::chrono_literals;

    int main(int argc, char **argv)
    {
    rclcpp::init(argc, argv);

    if (argc != 3) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
        return 1;
    }

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
        node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = atoll(argv[1]);
    request->b = atoll(argv[2]);

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }

    rclcpp::shutdown();
    return 0;
    } 

Best Practices
--------------
- **Naming Conventions**: Use snake_case for packages and nodes
- **QoS Settings**: Configure Quality of Service for critical applications
- **Error Handling**: Always check service availability before calls
- **Logging**: Use appropriate log levels (DEBUG, INFO, WARN, ERROR)

Next Steps
----------
:ref:`Module 3: Custom Interfaces <module3-interfaces>`
  
`ROS 2 Core Concepts Documentation <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html`_
