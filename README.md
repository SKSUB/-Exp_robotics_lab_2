# Exp_robotics_lab Assignment2
## INTROCUCTION
Purpose of this repository is to develop the assignment 2 of Experimental Robotics Laboratory. In this assignment robot moves between four position and oracle position in the gazebo simulation. There are markers in that four position, whenever robot place the cluedo link of its arm, the hint is generated. The generated hint is checked for consistency and complete hypothesis. If it is a complete and consistent hypothesis robot moves to the oracle position to check for the solution. If the solution is correct the game ends or otherwise it go to the other points to collect hints.

## SOFTWARE ARCHITECTURE
In order to better understand the software architecture we can utilise the UML component diagram

![CLUEDO_SIM](https://user-images.githubusercontent.com/82164428/220006145-f78bf9fe-5464-4f06-bde2-0c75e81ae07f.jpg)

The ontology_interface node which receives the solution and hint from the simulation node. The simulation node. Reset planner is the node which gives all the information to the ROSplan kowledge with the help of planner class about the PDDL problem. ROSplan with the aid of PDDL generate a plan and dispatch the plan to the action interface. The actions which are defined in the domain of PDDL are go_to_waypoint, movearm, update_ontology and check_consistency. go_to_waypoint sends the goal to reach to the go_to_point node(i.e marker coordinates posistion in the map). go_to_point moves the robot from intial to the goal position. Once it reaches the goal the move_arm node is initiated to move the robotic arm to the marker position.
