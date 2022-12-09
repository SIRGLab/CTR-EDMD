# CTR-EDMD
Repository for Simulation and Control of Concentric Tube Robots. It implements the methodology presented in presented in the following paper: "Data-driven Steering of Concentric Tube Robots in Unknown Environments via Dynamic Mode Decomposition
". Balint Thamo, David Hanley Kevin Dhaliwal, Mohsen Khadem. IEEE Robotics and Automation Letters, 2022.



This repo uses C++ in ROS Melodic environment to simulate data-driven steering of CTRs via Dynamic Mode Decomposition. 
You need to copy the folders into the src folder in your workspace. Then you can run it by the following command:  roslaunch ctr_main ctr_main.launch

The simulation environment shows each tube separately as well as the overall shape of the CTR.

In 'manual' mode the robot can be moved manually by altering beta and alpha values.

In 'koopman' mode first the 'initial learning' task needs to be selected to learn the dynamics of the system.
Then selecting the 'square' task steers the robot's tip along a square trajectory.
