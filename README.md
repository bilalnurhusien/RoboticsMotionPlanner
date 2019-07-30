### Description:
	The Probabilistic Roadmap Algorithm (PRM) is used to create a motion planner for a 2D polygon robot.
	Dijkstra's algorithm is used to calculate the shortest path between nodes in the PRM tree. An R-tree
	data structure is used to determine the neighboring nodes and the Boost Polygon library is utilized to
	determine if a point or line segment intersects an obstacle.

### Build:
	
	$ make

### Usage:

	$ ./pathplanning <num-of-robot-vertices> <num-of-obstacles>
	
### Library Installation:

	For rendering images, please install SFML library.

	$ sudo apt-get install libsfml-dev

### Examples: 

#### Triangle robot and fifteen obstacles:
	$ ./pathplanning 3 15

![alt text](https://github.com/bilalnurhusien/PathPlanning/blob/master/images/PathPlanningTenObstacles.PNG)

#### Right click to see the PRM graph in C-space

![alt text](https://github.com/bilalnurhusien/PathPlanning/blob/master/images/PathPlanningTenObstaclesCSpace.PNG)
