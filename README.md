### Description:
	The probabilistic roadmap planner (PRM) is a motion planning algorithm in robotics
	used to determine a path between start and goal configurations of a robot, while
	avoiding collisions. In this project, an obstacle-based probabilistic roadmap planner
	(OBPRM) is used to create a motion planner for a 2D robot that is able to translate
	and rotate freely.

### Build:
	
	$ make

### Usage:
	$ ./pathplanning -r <num-of-robot-vertices> -o <num-of-obstacles> | -i <csv-input-file>

		-r: Number of robot vertices
		-o: Number of obstacles
		-f: Full screen
		-i: CSV input file
		-n: Number of nodes in PRM graph
		-d: Minimum distance between PRM nodes
		Example:
			# Input CSV file
			$./pathplanning -i inputconfig.xml

			# For a triangle robot with 5 obstacles in full screen workspace with 500 nodes in PRM
			$./pathplanning -r 3 -o 5 -f -n 500

	
### Library Installation:

	For rendering images, please install SFML library.

	$ sudo apt-get install libsfml-dev

### Examples: 

#### Rectangle robot with narrow passage workspace
	$ ./pathplanning -i tests/configs/narrowbottomconfig.xml

![alt text](https://github.com/bilalnurhusien/RoboticsMotionPlanner/blob/master/images/NarrowPassage.jpg)

#### Left click on the window to to cause the robot to move to the selected position
![alt text](https://github.com/bilalnurhusien/RoboticsMotionPlanner/blob/master/images/NarrowPassagePath.jpg)

#### Right click to see the PRM graph in C-space

![alt text](https://github.com/bilalnurhusien/RoboticsMotionPlanner/blob/master/images/NarrowPassageCSpace.jpg)

#### Rectangle robot with more difficult narrow passage workspace
	$ ./pathplanning -i tests/configs/narrowuconfig.xml

![alt text](https://github.com/bilalnurhusien/RoboticsMotionPlanner/blob/master/images/NarrowUPassage.jpg)

#### Left click on the window to to cause the robot to move to the selected position
![alt text](https://github.com/bilalnurhusien/RoboticsMotionPlanner/blob/master/images/NarrowUPassagePath.jpg)

#### Right click to see the PRM graph in C-space

![alt text](https://github.com/bilalnurhusien/RoboticsMotionPlanner/blob/master/images/NarrowUPassageCSpace.jpg)


