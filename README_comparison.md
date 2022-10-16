# Approach to Robotics testing, a Comparison

The scope of this brief is to compare the traditional / manual testing approach that is typical within robotics context with a more modern / automated approach using the Artefacts platform.

## Overview

The usual test loop / life cycle (assuming a robotics test in simulation) is:

![ArtefactLifecycle](./images/LifecycleDiagram.svg)

## Traditional / manual case:

1. Succession of manual commands from the user:
- stand up the simulation (e.g launch gazebo/turtlesim/others)
- setup test parameters (e.g place the robot at the test-start position)
- start test (e.g launch user ROS nodes, for example to compute rover odometry and send commands to the rover to execute a standard trajectory)
- start recording (e.g start a rosbag recorder)

This leads to a multitude of terminal windows open:
- High cognitive load
- Prone to timing/order mistakes (e.g developper realizes he/she finds out at the end of the test that the data recorder failed)

2. End-of-test criteria is often decided by the user. This means the user must keep watching the simulation.

This repetitive, time consuming task can be acceptable at the start of development for ‘understanding the system’ but cannot be tolerated for :
- tuning algorithm parameters / exploring the design space
- regression testing
- simulation environments with a low real time factor

3. Compare results across tests

The user checks the logs on the terminal / watches the simulation / creates ad-hoc plots of metrics:
- tedious process
- a lot of context switching
- leads to forgetting or misremembering the result of a previous test (set of parameters) → draw wrong conclusion / iterate development in the wrong direction

## Benefits of using artefacts for each of the above use cases:

1. Single command + configuration file to orchestrate the entire process in the diagram above.
Removes the cognitive load + guarantees timing and proper execution of all steps + provides repeatability → the configuration file is version controlled.

2. One time effort to define (code) the end-of-test criteria unlocks the ability to:
- automatically stop the test / no need to watch the test itself (can grab a coffee instead)
- mainly: launch several tests one after the other without user intervention (can easily check the results of all the tests that ran overnight)

3. Test results and artifacts are automatically logged (in the cloud) in a central location (Dashboard):
- data is preserved without effort
- unified interface to compare tests and their various types of artifacts (graphs / videos / scalar metrics)
- the developper can perform a quick look in the Dashboard. Then, as needed, he/she can deep dive into the recorded rosbags with the help of automated launch of dedicated tools (Foxglove / Rviz / Plotjuggler). The appropriate configs can also be hosted in the cloud and made shareable between team members.

## Example use cases for Arteface and rover odometry:

- Tune PID gains of motor controllers
- Tune a kalman filter (e.g [ROS robot localization package](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html)) by doing several tests to explore the parameter space
- Evaluate rover odometry performance across several trajectories (some trajectories are shorter/longer, with craters/hills/obstacles
- ... many others!

Shows the gain in development time + quality + enjoyment of using artefacts vs traditional manual way (see the three ‘benefit’ points above)
-> simple demo of the above is available in the [demo-ros1-turtlesim repository](https://github.com/art-e-fact/demo-ros1-turtlesim/).
