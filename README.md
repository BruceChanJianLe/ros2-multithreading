# ROS2 Multi-Threading

This repository demonstrates the usage and note on multi-threading in ROS2.

The example is just to prove that multi threading is indeed working, however, putting pub and sub2 nodes in the same node may not be desirable.

WARNING: Be aware that are nodes in the graph that share an exact name, this can have unintended side effects.

## Reference
- https://github.com/ros2/examples/blob/master/rclcpp/executors/multithreaded_executor/multithreaded_executor.cpp
- https://github.com/ros2/examples/tree/galactic/rclcpp/executors/cbg_executor
- https://docs.ros.org/en/galactic/Concepts/About-Executors.html
- https://answers.ros.org/question/401066/ros2-how-to-do-multi-threading-subscription-callbacks-spinning-executors/
- https://discourse.ros.org/t/how-to-use-callback-groups-in-ros2/25255
- https://github.com/ros2/examples/issues/208
