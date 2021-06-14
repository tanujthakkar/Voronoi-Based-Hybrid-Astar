# Voronoi Based Hybrid A*


The following project is an extension of my bachelor's thesis conducted under the guidance of 
[Prof. Arpita Sinha](https://scholar.google.com/citations?user=ROSN3WgAAAAJ&hl=en&oi=ao) at Indian Institute of Technology Bombay (IITB) and submitted at 
Charotar University of Science and Technology. The goal of the project was to implement a suitable path planning strategy for a tractor-trailer system. In our 
implementation, we extend the [Hybrid A* algorithm](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf), first by introducing novelties to the 
heuristic search to suit the tractor-trailer model and then present a modified analytical expansion step based on voronoi graph to guide the search towards 
the goal faster.

The code in this repository is specifically tested on <b>Ubuntu 18.04</b> and <b>ROS Melodic</b>. The implementation is based on a 
[Clearpath Robotics Ridgeback](https://clearpathrobotics.com/ridgeback-indoor-robot-platform/) robot towing a custom designed cart, however it can be adapted
to other tractor-trailer systems easily.

<h3>Installation</h3>

<h4>Dependencies</h4>
<ul>
<li>Ubuntu 18.04</li>
<li>ROS Melodic</li>
<li><a href="http://ompl.kavrakilab.org/">Open Motion Planning Library (OMPL)</a></li>
 
```
sudo apt install libompl-dev
```
  
<li><a href="http://wiki.ros.org/jsk_recognition_msgs">jsk_reconition_msgs</a></li>

```
sudo apt-get install ros-melodic-jsk-recognition-msgs
```

<li><a href="https://github.com/tuw-robotics/tuw_multi_robot">tuw_multi_robot</a></li>
Follow INSTALL.md given in the repository. Refer to this <a href="https://github.com/tuw-robotics/tuw_multi_robot/issues/29">issue</a> if compilation error exists. Install it in the same catkin workspace as Voronoi based Hybrid A*.
</ul>

<h4>Setup</h4>

```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/tanujthakkar/Voronoi-Based-Hybrid-Astar.git
cd ..
catkin_make
source devel/setup.bash
roslaunch hybrid_astar hybrid_astar.launch
```

Wait for the voronoi graph to be computed and visualized in RViz. After that, 2D Pose Estimate and 2D Nav Goal tools can be used to specify start and goal 
positions respectively for the planner.

<h3>Citation</h3>
We are currently working on a paper which will soon be added here to cite.

<h3>References</h3>
<ul>
<li><a href="https://github.com/karlkurzer/path_planner">Hybrid A* by Karl Kurzer</a></li>
<li><a href="https://github.com/AndrewWalker/Dubins-Curves">Dubins Curves by Andrew Walker</a></li>
</ul>
