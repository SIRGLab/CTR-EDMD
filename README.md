# CTR-EDMD
Data-driven Steering of Concentric Tube Robots in Unknown Environments via Dynamic Mode Decomposition




This repo uses C++ in ROS environment to simulate data-driven steering of CTRs via Dynamic Mode Decomposition. 
You need to copy the folders into the src folder in your workspace. Then you can run it by the following command:  roslaunch ctr_main ctr_main.launch

The simulation environment shows each tube separately as well as the overall shape of the CTR.

In 'manual' mode the robot can be moved manually by altering beta and alpha values.

In 'koopman' mode first the 'initial learning' task needs to be selected to learn the dynamics of the system.
Then selecting the 'square' task steers the robot's tip along a square trajectory.
