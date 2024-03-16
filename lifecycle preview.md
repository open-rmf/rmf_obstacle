## rmf_obstacle_detector_laserscan _waiting to configure_

If while running ``` ros2 run rmf_obstacle_detector_laserscan laserscan_detector ``` you encounter **waiting to configure** as shown below:

![WhatsApp Image 2024-03-16 at 4 58 31 PM](https://github.com/Avisheet/rmf_obstacle_detectors/assets/59338120/5621b525-e7d0-4df8-b0db-59ed5a923e10)

To configure and activate the node, you may enter the following commands:

```
# to configure
ros2 lifecycle set /laserscan_obstacle_detector configure
```
```
# to activate
ros2 lifecycle set /laserscan_obstacle_detector activate
```

This document outlines a managed life cycle for nodes in the Robot Operating System (ROS) environment. The life cycle consists of four primary states: Unconfigured, Inactive, Active, and Finalized, along with several transition states. The transitions between states are controlled by external supervisory processes or by the node itself in certain circumstances.

Key points:

1. **Primary States**: Nodes exist in one of four primary states: Unconfigured, Inactive, Active, and Finalized.
   
2. **Transition States**: There are six transition states: Configuring, CleaningUp, ShuttingDown, Activating, Deactivating, and ErrorProcessing. These states represent intermediate stages during state transitions.

3. **Transitions**: There are seven transitions exposed to supervisory processes: create, configure, cleanup, activate, deactivate, shutdown, and destroy.

4. **Behavior in Each State**:
   - Unconfigured: Initial state after instantiation or after an error. Node configuration happens here.
   - Inactive: Node is not processing anything. It can be configured or transitioned to active state.
   - Active: Node performs processing, responds to requests, etc.
   - Finalized: Terminal state before destruction, used for debugging and introspection.

5. **Management Interface**: Nodes expose an interface for management, allowing for transitions between states and reporting of life cycle events. This interface includes services for each transition and a latched topic for broadcasting life cycle state changes.

6. **Node Management**: State transitions are typically managed by external tools, but nodes can also manage themselves in certain cases. Local management tools and self-management by nodes are possible but discouraged.

7. **Extensions**: The defined life cycle is not intended to be extended with additional states. It is expected to be supported throughout the ROS toolchain.

Overall, this managed life cycle for nodes allows greater control over the state of the ROS system. It will enable roslaunch to ensure that all components have been instantiated correctly before allowing any component to begin executing its behavior. Additionally, it will facilitate the online restarting or replacement of nodes.

This structured approach to node management enhances system reliability, ensures proper initialization, and supports dynamic adaptation to changes in the environment or system configuration.

For more details refer :
- Design: https://design.ros2.org/articles/node_lifecycle.html
- Demos: https://github.com/ros2/demos/tree/rolling/lifecycle
