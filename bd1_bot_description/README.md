# BD1 Bot

<p align = "justify">
The BD1 Bot is re-modeled for self-balancing tasks from scratch using Fusion360. The CAD Model of our design is shown in the image. </p>

<div style="text" align="center">
    <img src="https://github.com/hari-vickey/ROS2-Self-Balancing-Bot/blob/main/documents/images/bd1_bot_fusion.png" />
</div>

Note: The steps mentioned below are only for users to develop their custom designs and test them on the Gazebo simulator.

After, completing the entire design of the CAD model, the design is converted to the ROS2 package using the [fusion2urdf-ros2](https://github.com/dheena2k2/fusion2urdf-ros2.git) repository. Instructions to install and use this plugin are mentioned on the repository itself.

To export the CAD model to the ROS2 package there are some important rules to follow such as
1. One of the components should have base_link as the component name.
2. Design should not have references to other CAD Models
3. To color individual parts of the model, they should be attached as separate components.
4. Do not move the components after creating joints in the model.

## Test the Package

After exporting the design, there are some steps to add color and gazebo plugins.
Get to know some of the plugins available on ROS Gazebo package using the [link](https://medium.com/@bytesrobotics/a-review-of-the-ros2-urdf-gazebo-sensor-91e947c633d7)
1. Copy-Paste the exported ROS2 package into the ROS2 workspace.
2. Open urdf/bd1_bot.gazebo. To add or change color to the component. Edit this file as shown in the image.
    ![](https://github.com/hari-vickey/ROS2-Self-Balancing-Bot/blob/main/documents/images/change_color.png)
3. To add gazebo plugins visit this [link](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki). An example to add a camera gazebo plugin is shown in the image.
   ![](https://github.com/hari-vickey/ROS2-Self-Balancing-Bot/blob/main/documents/images/add_plugin.png)
4. Once all the changes are made, build the package and source it.

Run the command to test the package.

    # Open the design on Rviz
    ros2 launch bd1_bot_description display.launch.py
    # Open the design on Gazebo
    ros2 launch bd1_bot_description display.launch.py

Verify the added plugins.

    # Check whether all plugins are available to use
    ros2 topic list
    # This design has a lidar, camera and differential drive plugin
    # Use this command to echo the messages from appropriate topics
    ros2 topic echo /topic # Make sure that the gazebo is not paused
    # You can also use rqt GUI to test the topics
    rqt

<div style="text" align="center">
    <img src="https://github.com/hari-vickey/ROS2-Self-Balancing-Bot/blob/main/documents/images/test_description_pkg.png" />
</div>