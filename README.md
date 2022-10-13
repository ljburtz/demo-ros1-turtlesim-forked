# demo-ros1-turtlesim
## Overview: a development environment for demontrating the Artefact platform with ROS 1.

For maximum simplicity, this demo uses the turtlesim simulator.
![Turtlesim](./images/Turtlesim.png)

To demonstrate a foundational robotics use case, this demo implements the computation of the trajectory of the turtle, similar to any robot's odometry.

![Turtlesim](./images/TurtleTrajectory.png)

Artefact strives to enable managing and automating the entire lifecycle of robot feature development. The following lifecycle features are implemented in this demo:

![ArtefactLifecycle](./images/LifecycleDiagram.svg)

After a one-time investment into setting up appropriate configurations (the artefact client and one .yaml config file), the functions in this diagram are all automated by artefact and launched with a single command by the user:
- **Stand-up simulation:** the turtlesim simulator is launched
- **Setup Test Params:** User-specified test-specific test parameters are reflected in the simulator, e.g. the turtle is teleported to the starting position
- **Start Test**: User-specified test commands are executed
- **Record Data**: During the test, data e.g. published ROS topics are recorded in a ROSbag.
- **Compute metrics**: During the test, additional nodes to compute metrics are executed
- **Stop Test**: User-specified test stop criteria or timeout defines the end of the tests and processes exit gracefully
- **Post Process**: User-specified post-processing script runs. Currently, its input is the ROSbag of recorded data. The output can be any file (e.g an html figure)
- **Log Results**: All artifacts created during the test are uploaded to the cloud and displayed in a Dashboard. E.g the ROSbag can be conveniently downloaded for future analysis / html figures are rendered. A share-able link can be used to invite collaborators to view test results.
- **Iterate tests over several parameters**: Parameters in the .yaml config file can be specified as a list of possible values. Then all of the above process will run for each combination of the parameters (grid search). All the data from each test will be automatically viewable in the cloud Dashboard.

For more insight into how artefact simplifies manual robotics development tasks see [TODO](https://TODO)



## Install and First Steps
1. Clone this repo and the artefact client repo side by side on your PC
```
git clone git@github.com:art-e-fact/demo-ros1-turtlesim.git
git clone git@github.com:art-e-fact/warp-client.git
```
2. Build and run the docker container with this demo setup: this includes ROS1 dependencies, the setup of a ROS workspace with the turtle_odometry package sourced, as well as proper mounting of volumes to make development easier and aliases.
```
cd demo-ros1-turtlesim
docker-compose run --rm turtle
```
Now you are within the docker container, where all development can happen (ROS commands / jupyter notebook server / all the code in the ros_workspace/src folder is synced with your host so you can use your desktop IDE).

3. Now setup the Artefact client (here, done in development mode):
```
cd /warp-client && pip install -e . && cd
```
(or use the convenience alias, `ww`)

Then, create an account and run Step 1 and Step 2 of the instructions at `https://app.artefacts.com` (everything always within the docker container). This is the Dashboard in the cloud.

Finally, go to the folder containing our warp.yaml config file and run your first test with :
```
cd `ros_workspace/src/turtle_odometry/test/`
warpcli run basic_turtle
```

You will see the turtlesim window appear and the turtle perform a square trajectory. You will see a printout of the result of the test such as `SUCCESS`. Head over to the Dashboard to see the test results and all logged data.

You can play around and change the parameters of the warp.yaml file: for example change the starting position of the turtle or the segment_length of its trajectory (keep in mind that the size of the turtlesim environment is just 11 by 11m meters, with x,y = 0,0 at the bottom left).
