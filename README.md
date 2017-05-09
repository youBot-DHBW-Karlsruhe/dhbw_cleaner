# dhbw_cleaner 
A KUKA youBot roboter platform, which is performing simple clean up tasks with one manipulator.


## Contents
| Folder                    | Description                                                                                                                                                       |
|---------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [0-Documentation][0]      | Folder containing some docs about the environment setup, using this software etc.                                                                                 |
| [cleaner\_alpha][1]       | Main application, where everything is put together.                                                                                                               |
| [laser\_to\_cloud][2]     | Not used anymore. Now the laser_assembler is used for this logic.                                                                                                 |
| [object\_finder\_2d][3]   | Node for calculating the nearest point from the youBot front.                                                                                                     |
| [object\_recognition][4]  | Node using [find_object_2d]-ROS package for object detection and position assignment. After that position information is transformed and manipulated to fit our needs using TF. |


## Dependencies
Install these packages as binaries:
- cmake\_modules
- sick\_tim
- laser\_pipeline
- openni2\_launch
- openni2\_camera
- [find_object_2d] and all its dependencies

These packages must be installed from source:
- [torque_control] and all its dependencies. Please follow the installation description in the README-File of the author.

A more detailled setup guide is available [here](https://github.com/youBot-DHBW-Karlsruhe/dhbw_cleaner/tree/master/0-Documentation/EnvironmentSetup.md).


## Installation
You can download this application by running 
```
cd ~/catkin_ws/src
git clone https://github.com/youBot-DHBW-Karlsruhe/dhbw_cleaner.git
```
After that, you can simply use catkin to build the package
```
cd ~/catkin_ws
catkin_make
```

## Testing the installation and running the application
coming soon...



[0][https://github.com/youBot-DHBW-Karlsruhe/dhbw_cleaner/tree/master/0-Documentation]
[1][https://github.com/youBot-DHBW-Karlsruhe/dhbw_cleaner/tree/master/cleaner_alpha]
[2][https://github.com/youBot-DHBW-Karlsruhe/dhbw_cleaner/tree/master/laser_to_cloud]
[3][https://github.com/youBot-DHBW-Karlsruhe/dhbw_cleaner/tree/master/object_finder_2d]
[4][https://github.com/youBot-DHBW-Karlsruhe/dhbw_cleaner/tree/master/object_recognition]

[find_object_2d](https://github.com/introlab/find-object)
[rpg_youbot_torque_control](https://github.com/uzh-rpg/rpg_youbot_torque_control)
[laser_pipeline](https://github.com/ros-perception/laser_pipeline)
