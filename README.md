#Introduction

COOPERA (COst-effective Open-source Platform for Empirical Robotics and Arficial Intelligence) is a robotic platform based on the Bioloid} robot and it took inspiration from the iCub project. Yarp, a middleware software developed in the context of the iCub project, was used as a middleware layer to implement a modular and efficient interface between the user and the robot. 

With a modest economic investment (the overall cost of the platform is less than 1500\$) it is possible to own a small and robust 
humanoid with 19 degrees of freedom with a camera, a 32-bit processing unit running linux-based kernel, and a YARP-based interface. 
COOPERA is able to demonstrate experiments involving artificial intelligence and machine learning algorithms applied to humanoid 
robots. In spite of its intrinsical limitations, COOPERA is an useful research tool.

In particular, we have used this platform in order to program a robot that is able to learn directly on the real robot (in-vivo) how to stand up using a Reinforcement Learning approach instead of using simulators, as usually done in research.

#Brief explanation of the modules

##Encoders
It can be used to get the encoders corresponding to a specific robot configuration or to set a new configuration by passing the angles that must be gained by the motors (for instance it was used to devise the BehaviorModule's reset procedures)

##Torques
It can be used to get the torques corresponding to a specific robot configuration

##Sensors
It can be used to get the sensors feedback corresponding to a specific robot configuration

##MotorsController
It provides the system demo: pose estimation(linear classification), turning (SARSA) and standing up (scripted)

##TrainTorques
It is used to build the training test for the linear classification, used in MotorsController.

##SarsaLearner
Learning (with SARSA algorithm) of a policy for turning the robot, used in MotorsController

##SarsaPolicy
It is used to debug the SarsaLearner module

##Actions
It is used to create (or test) actions, which are then used in MotorsController

##BehaviorModule
It runs the SARSA algorithm to learn a task in vivo

##More info
If you're interested in replicate the platform, contact me to get the robot firmware required for the modules to work. 

See manual.pdf for full documentation
