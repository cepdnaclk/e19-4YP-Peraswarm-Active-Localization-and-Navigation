---
layout: home
permalink: index.html

# Please update this with your repository name and title
repository-name: e19-4YP-Peraswarm-Active-Localization-and-Navigation
title: Active localization and navigation
---

[comment]: # "This is the standard layout for the project, but you can clean this and use your own template"

# Project Title
Active localization and navigation

#### Team

- e19167, H.D.N.S. JAYAWARDENA, [email](e19167@eng.pdn.ac.lk)
- e19423, Weerasingha W.A.C.J, [email](e19423@eng.pdn.ac.lk)



#### Supervisors

- Prof. Roshan Ragel, [email](roshanr@eng.pdn.ac.lk)
- Dr. Isuru Nawinna, [email](isurunawinne@eng.pdn.ac.lk)
- Ms. Narmada Balasooriya, [email](narmadabalasooriya@gmail.com)

#### Table of content

1. [Abstract](#abstract)
2. [Related works](#related-works)
3. [Methodology](#methodology)
4. [Experiment Setup and Implementation](#experiment-setup-and-implementation)
5. [Results and Analysis](#results-and-analysis)
6. [Conclusion](#conclusion)
7. [Publications](#publications)
8. [Links](#links)

---

<!-- 
DELETE THIS SAMPLE before publishing to GitHub Pages !!!
This is a sample image, to show how to add images to your page. To learn more options, please refer [this](https://projects.ce.pdn.ac.lk/docs/faq/how-to-add-an-image/)
![Sample Image](./images/sample.png) 
-->


## Abstract
This research aims to develop a navigation and path-planning algorithm that integrates visual SLAM with reinforcement learning for a ground-aerial multi-vehicle system. Autonomous vehicles have become essential for extreme environments such as search and rescue, disaster relief, and infrastructure inspection, where human presence is limited. The combination of unmanned ground vehicles (UGVs) and unmanned aerial vehicles (UAVs) enhances mission efficiency by leveraging the complementary strengths of both systems—UGVs provide endurance and payload capacity, while UAVs offer aerial surveillance and rapid maneuverability.

Localization and mapping are crucial for swarm robotic systems to achieve coordinated autonomy in dynamic environments. Unlike single-robot SLAM, swarm robotics requires decentralized or cooperative approaches to share positional and environmental data among multiple agents efficiently. Techniques such as multi-agent SLAM, topological mapping, and probabilistic localization enable robust navigation while minimizing computational and communication overhead. Sensor fusion, incorporating LiDAR, IMU, and vision-based methods, enhances accuracy in GPS-denied environments. Focus on implementing localization and mapping without using GPS for swarm system which includes UGVs and UAUs. Advanced strategies like collaborative loop closure, distributed pose graph optimization, and reinforcement learning improve adaptability and resilience in uncertain conditions. The integration of machine learning and neural networks further refines localization accuracy, making swarm robotic systems more efficient for applications in search-and-rescue, exploration, and industrial automation.

## Related works

1. [Sensors](#Sensors)
2. [SLAM](#SLAM)
3. [Localization](#Localization)
4. [ROS&Gazebo](#ROS&Gazebo)
5. [Mathamatics](#Mathamatics)

## Sensors

What are sensors?
Sensors are devices that measure and record various properties. In robotics, we use sensors to gather information about a robot’s internal parameters and its environment. Essentially, sensors function as the robot’s “eyes” and “ears”.
Sensor types
 Proprioceptive – Sensors that measure internal parameters of the system.(gyroscopes, accelerometers)
Exteroceptive – sensors that measure environmental parameters. (Cameras, microphones, LiDAR)
Passive – sensors that measure the energy entering the sensor. (cameras, microphones)
Active – sensors that measure a parameter by emitting energy to the environment. (sonar, radar)
Challenges
1. Environmental Sensitivity
•	LiDAR: Affected by reflective or transparent surfaces like glass and mirrors, as well as adverse weather conditions (fog, rain, or snow).
•	Infrared Sensors: Performance can degrade due to strong ambient light or heat sources.
•	Ultrasonic Sensors: Susceptible to environmental noise and temperature variations that impact sound speed.
2. Limited Accuracy and Precision
•	Sensors like ultrasonic and infrared often struggle with fine precision, especially when detecting small objects or measuring long distances.
3. Line of Sight Requirements
•	Many sensors, such as cameras, LiDAR, and infrared sensors, require an unobstructed line of sight to gather accurate data.
4. Dependency on Calibration
•	Sensors need frequent calibration to maintain accuracy, especially in dynamic or harsh environments.
5. Energy Consumption
•	High-power sensors, such as LiDAR and vision cameras, may drain batteries quickly in mobile robots, impacting operational time.

IMU Sensor
An IMU is an electronic device that integrates an accelerometer, gyroscope, and magnetometer to measure an object's acceleration, angular velocity, and direction of the geomagnetic field. The Inertial Navigation System (INS) is a dead-reckoning navigation system commonly used for mobile robot navigation. The trajectory of an indoor AMR is determined by integrating acceleration and angular velocity based on the initial position, direction, and velocity. IMUs have advantages such as high output update rate and immunity to external interference, but they have limitations when applied to mobile robots. These disadvantages include the calculation process relying on initial conditions and the need for double integration of measurements. To overcome these limitations, IMUs are often used in fusion with other sensors and as auxiliary measurements in other navigation methods.

Ultrasonic Sensor
Ultrasonic sensors, operating at frequencies above 20 kHz, are essential for distance measurement and navigation in mobile robots. They use two primary methods:
1.	Reflection-type: The sensor emits a signal that reflects off obstacles, with the time-of-flight determining the distance.
2.	Unidirectional: A transmitter and receiver placed separately measure the distance directly between them.
These sensors rely on time-of-flight, phase differences, and acoustic vibrations for measurements. They are widely used for indoor localization methods like the three-sided localization principle and omnidirectional ultrasonic sensing.
While prone to environmental noise and long-range signal degradation, ultrasonic sensors excel at detecting reflective or transmissive objects, such as glass and mirrors, which challenge optical sensors. Their accuracy and versatility make them reliable for obstacle detection and navigation in various conditions.

LiDAR
LiDAR (Light Detection and Ranging) uses laser beams to measure the position and distance of objects by analyzing reflected signals. Operating on the time-of-flight (ToF) principle, LiDAR provides high accuracy, speed, and range compared to conventional radar. In mobile robot navigation, 2D LiDAR is commonly used for indoor applications, while 3D LiDAR is preferred in autonomous driving for detailed environmental mapping.
Key advancements include SLAM (Simultaneous Localization and Mapping) techniques such as Gmapping, Hector SLAM, Karto SLAM, and Cartographer, which improve mapping accuracy and efficiency. Recent developments integrate neural networks, intensity-based methods, and multi-sensor fusion to enhance localization and obstacle detection. LiDAR's limitations, such as errors with glass or mirror surfaces, can be mitigated by fusing ultrasonic sensors.

## SLAM

Simultaneous Localization and Mapping (SLAM) is very important topic in robotics and automation. When operating in unknown environments SLAM provide privilege to while operating also update the localization and map the environment in parallel. SLAM give more efficiency and the autonomy of the robotic system. The algorithms are varied with the sensors which used for the system and the hardware and computational power used on robots. In localization and mapping reducing the uncertainties is a major thing it can be achieved by increasing the accuracy of sensor data but in real world we can’t handle everything as operator needed like drift, wind, slipping and noises from environment etc. So, in case of increasing the accuracy and confidentiality of a localization and mapping system SLAM algorithm act a critical role. Methods like feature extraction, feature matching, extended Kalman Filter, gaussian filters, sign distance function and probability distribution models used to overcome that challenges and increase the efficiency of the autonomous systems. SLAM algorithms have evolved significantly, driven by the increasing demands for accuracy, robustness, and real-time performance in robotics and automation. Feature extraction, probabilistic modeling, and sensor fusion are key trends enabling robust SLAM systems. Modern techniques, such as those leveraging visual-inertial data (e.g., VINS-Mono, ORB-SLAM2) and collaborative approaches, demonstrate the power of integrating diverse sensor data and multi-agent collaboration. Direct methods like DSO and LSD-SLAM have gained prominence for their ability to work effectively in dynamic or low-texture environments. Emerging trends also include integrating LiDAR and visual data, enhancing SLAM performance in low-light or large-scale settings. The field is shifting towards lightweight, scalable solutions capable of handling heterogeneous sensors, real-time constraints, and privacy concerns while maintaining high accuracy in challenging scenarios.

## Localization

What is localization?
Localization refers to the process by which a robot determines its location and understands its surroundings. This is similar to how humans identify their position using environmental cues gathered through their senses, such as sight and sound. 
Humans can interpret what they perceive and make decisions based on that information. In contrast, robots receive these inputs as signals, which humans can program the robots to analyze and act upon.
Typically, humans are quite adept at accurately identifying their surroundings. However, robots can encounter errors in determining their location and environment.
When a robot identifies its position, it may be in a known space—where it already possesses a map—or in an unknown space—where it constructs a map as it navigates.
The robot should have a belief about its current position. This belief can be about a single position or the robot can identify a set of possible positions of it. These two methods are called
•	Single hypothesis belief
•	Multiple hypothesis belief
Each methods have their pros and cons
Single hypothesis belief
Pros: No position ambiguity
Cons:  robot motion often induces uncertainty due to affective and sensory noise.
Multiple hypotheses belief
This allows the robot to track not just a single possible position but a set of possible positions.
 Pros: the robot can reason about reaching a particular goal and consider the future trajectory of its belief state.
Cons: the calculations require more computational power. some of the robot’s possible positions imply a motion trajectory that is inconsistent with some of its other possible positions

Map representations
1. Continuous Representation Mapping
•	Definition: Represents the environment in a continuous space, often capturing precise geometric details.
•	Techniques:
o	Occupancy Grid Maps:
	Divide the environment into a grid of cells.
	Each cell represents the probability of being occupied or free.
	Examples: 2D or 3D grids.
o	Feature-Based Maps:
	Represent the environment using features like lines, corners, or landmarks (e.g., visual or geometric features).
	Example: Simultaneous Localization and Mapping (SLAM) algorithms with feature-based backends.
o	Signed Distance Functions (SDF):
	Stores the shortest distance from any point to the nearest obstacle or surface.
	Common in 3D modeling and robotics for accurate collision checking.
________________________________________
2. Abstract/Discrete Representations
•	Definition: Represent the environment in a simplified, abstract, or symbolic manner.
•	Techniques:
o	Topological Maps:
	Represent the environment as a graph where nodes are places (e.g., rooms or intersections) and edges are paths connecting them.
	Example: Used in environments where connectivity is more important than metric precision.
o	Voronoi Diagrams:
	Partition the space into regions based on proximity to a set of predefined points (e.g., obstacles or landmarks).
	Useful for path planning and navigation.
o	Semantic Maps:
	Augment spatial representations with semantic information (e.g., "this is a door," "this is a chair").
	Used in scenarios where contextual understanding is critical.
________________________________________
3. Hybrid Representations
•	Definition: Combine continuous and abstract representations to balance precision and computational efficiency.
•	Techniques:
o	Hybrid Metric-Topological Maps:
	Combine the precision of metric maps with the scalability of topological maps.
	Example: Metric submaps connected by a topological graph.
o	Octree-Based Maps:
	Use a hierarchical tree structure to represent the environment at different resolutions.
	Example: OctoMap (3D mapping).
o	Graph-SLAM:
	Represent the robot's trajectory and landmarks as nodes and their relationships as edges.
	Optimized to reduce errors over time.
________________________________________
4. Probabilistic Approaches
•	Definition: Represent uncertainty in the environment or robot's position using probabilities.
•	Techniques:
o	Bayesian Occupancy Filters:
	Update grid cells' occupancy probabilities based on sensor measurements.
o	Particle Filters:
	Maintain multiple hypotheses of the robot’s position.
	Used in Monte Carlo Localization (MCL).
o	Gaussian Process Maps:
	Model the environment as a continuous function, allowing for smooth interpolation between measurements.
________________________________________
5. Specialized Representations
•	Definition: Tailored for specific tasks or environments.
•	Examples:
o	Elevation Maps:
	Represent 3D terrain as a 2D grid where each cell stores height information.
	Useful for navigation in outdoor environments.
o	Point Clouds:
	Raw 3D data obtained from sensors like LiDAR or stereo cameras.
	Used for detailed modelling and mapping.


## ROS&Gazebo
Robot Operating System (ROS)
ROS (Robot Operating System) is an open-source framework widely used in robotics research and development. It provides a collection of tools, libraries, and conventions that simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. ROS follows a modular design where different components (or nodes) communicate with each other through topics, services, and actions. It supports a publish-subscribe model for efficient data exchange and allows seamless integration of sensor data, actuator control, and state estimation. In this project, ROS acts as the middleware enabling coordination between perception, mapping, navigation, and decision-making components, especially when dealing with multiple robots in a shared environment.

Gazebo Simulation
Gazebo is a powerful robotics simulator that integrates with ROS to provide a realistic 3D environment for developing and testing robotic algorithms. It offers dynamic physics simulation, sensor modeling (like LiDAR, IMU, and cameras), and accurate visualization of robot behavior in a simulated world. By replicating real-world conditions such as gravity, friction, and obstacles, Gazebo allows researchers to test algorithms before deploying them to physical hardware, reducing risk and development time. In this project, Gazebo is used to simulate multiple TurtleBot3 robots navigating and mapping an environment collaboratively. The simulation provides synthetic sensor data and odometry, which are fed into ROS nodes for tasks like SLAM, path planning, and multi-agent coordination.


## Mathamatics

## Methodology
To investigate active localization and navigation in a multi-robot system, we designed and implemented a simulation-based framework using the Robot Operating System (ROS) Noetic and Gazebo 11 as the core middleware and simulation environment, respectively. The system consists of Unmanned Ground Vehicles (UGVs) and Unmanned Aerial Vehicles (UAVs), each simulated in separate virtual machines (VMs) and interfaced through a centralized SLAM architecture.
Simulation Environment and Robot Platforms
The UGV platform selected for this study was the TurtleBot3, a compact differential-drive robot widely used in academic robotics research. It was chosen primarily due to its built-in visual sensors, which are compatible with visual SLAM methods. The UAV platform was based on the PX4 flight stack, utilizing the Iris drone model equipped with depth cameras to enable three-dimensional environmental perception. PX4 was integrated with ROS through the MAVROS interface, allowing communication between the flight controller and the ROS-based SLAM system.
Centralized SLAM Architecture
Our solution follows a centralized SLAM paradigm, in which sensor data from multiple agents is transmitted to a central server responsible for constructing and maintaining a global map. This architecture allows for data fusion across heterogeneous platforms (UAVs and UGVs), enabling coordinated mapping and localization. The key benefit of a centralized SLAM approach is the consistency of the global map, which facilitates collaboration between robots. However, it introduces challenges in communication latency and requires robust synchronization mechanisms.
Reinforcement Learning 
Integrating a Reinforcement Learning (RL) module for navigation offers a more adaptive and learning-driven alternative to classical path planning algorithms like A*. Unlike rule-based planners that rely heavily on explicit maps and deterministic heuristics, RL enables robots to learn optimal navigation policies through trial and error, even in partially observable or dynamic environments. This allows for smoother decision-making, better obstacle avoidance in cluttered spaces, and adaptability to new layouts without reprogramming. RL can also optimize behaviors that go beyond reaching a goal—such as energy efficiency, smooth trajectories, or multi-objective tasks.
However, RL comes with several trade-offs. Training the model is computationally intensive and time-consuming, often requiring simulated environments or large amounts of interaction data. It may suffer from sample inefficiency, local minima, or instability during training. Moreover, once trained, the RL agent might not generalize well to new environments without further fine-tuning, unlike classical planners which can immediately adapt to new maps. RL agents also lack guaranteed optimality and explainability, which makes safety-critical applications more challenging.

## Experiment Setup and Implementation

Due to hardware constraints, the system was distributed across three virtual machines (VMs), each running Ubuntu 18.04 LTS. One VM simulated the UGVs, another simulated the UAVs, and a third functioned as the centralized SLAM server. This setup offered modularity and facilitated easier configuration of inter-process communication through virtual networks. We adopted this VM-based deployment due to its scalability and the ease of establishing isolated yet connected simulation environments under limited computational resources.
Multi-Robot Communication Framework
To enable inter-VM communication across the distributed ROS environments, we utilized the fkie_multimaster package. This toolset allows multiple ROS masters to discover each other and share selected topics across machines, thus supporting a loosely coupled architecture suitable for our centralized SLAM framework. Despite its flexibility, we encountered performance limitations when attempting to run all three VMs simultaneously on the available hardware. As a result, SLAM development for UAVs and UGVs was conducted separately, and their outputs were later integrated offline.

UGV SLAM
To achieve accurate robot localization in noisy environments, each robot employs an Extended Kalman Filter (EKF) to fuse data from onboard Inertial Measurement Units (IMU) and 2D LIDAR sensors. The EKF prediction step models motion dynamics using Jacobians and covariance matrices, assuming small time intervals (~1s) between updates. Linear accelerations and angular velocities from the IMU are used to predict the robot's new pose, while the update step incorporates LIDAR-based corrections. Pose estimates are used to transform sensor readings into the robot’s local coordinate frame. A local occupancy grid of size 60×60 cells (0.1m resolution) is constructed using Bresenham’s algorithm, with LIDAR beams marking free and occupied cells. This local map is updated at 1 Hz and published via the /local_map ROS topic for global merging.


Due to the resource constraints and partial progress in developing a full SLAM pipeline, a hybrid approach was adopted for final validation. RTAB-Map, a graph-based SLAM method well-suited for RGB-D input, was applied to the UGV simulation to generate local maps. A custom ROS-based script was then developed to merge these local maps into a cohesive global map. This served as a proof of concept for the centralized map integration strategy envisioned for the full multi-robot system.

UAV slam

The aerial component of our system was simulated using PX4-based Iris drones within the Gazebo 11 environment. The UAVs were equipped with simulated Intel RealSense depth cameras to enable 3D perception. PX4 SITL (Software-In-The-Loop) was used to provide flight control, while MAVROS acted as a middleware to bridge PX4 with ROS Noetic.
We deployed the UAV simulation in a dedicated Ubuntu 18.04 VM. To support scalable multi-drone simulation, each UAV instance was launched with a unique MAVROS namespace and TF prefix. Visual SLAM was implemented using RTAB-Map, leveraging the UAVs’ depth and odometry data to build 3D Octomaps.
A consistent TF tree was enforced by disabling MAVROS’s default TF publisher and integrating a custom transform pipeline to align all frames (e.g., map, odom, base_link, camera_link) under a centralized SLAM coordinate system. Each UAV published data to a separate ROS namespace, allowing structured topic handling.
Due to hardware limitations, the UAV SLAM was developed and tested separately from the UGV system. Each UAV’s local Octomap was generated on the central server, and subsequently merged into a unified 3D map. Map merging was based on known initial positions and odometry, avoiding the need for computationally expensive loop closure or global frame registration.

A*-Based Hybrid Path Planning*
The navigation strategy integrates global and local planning using a hybrid A* approach. A centralized planner assigns each robot a global goal based on the merged occupancy map, while the local planner operates within a 6×6m (60×60 cells) dynamic window. A* search is performed in this window using a Euclidean-distance heuristic. If the global goal is visible, a direct path is computed; otherwise, the planner selects the best edge cell that leads toward the goal, promoting continuous movement even under partial observability. Planning is recalculated at every timestep, enabling real-time reaction to local changes. The planner comprises modular sub-functions, including extract_local_map(), get_neighbors(), a_star(), and find_edge_local_goal(), all integrated in local_global_planner().

DQN-Based Reinforcement Learning Navigation
To complement the rule-based A* planner, a Deep Q-Network (DQN) was developed for learning-based navigation. The model takes as input a 20×20 local occupancy grid and a 4D pose vector representing the robot’s position and goal coordinates. The grid input is processed via a convolutional layer (16 filters, 3×3 kernel, tanh activation) followed by max pooling and flattening. The pose vector passes through a fully connected layer with 32 ReLU units. The outputs are concatenated and fed into a dense layer with 128 ReLU units, with a final linear layer producing Q-values for four discrete actions: move forward, left, right, or stay. The model is trained using mean squared error (MSE) loss and the Adam optimizer with a learning rate of 1e-3. A target network is used to stabilize training, with weights periodically synchronized from the policy network. The reward function incentivizes reaching the goal, penalizes collisions and looping, and includes shaping based on heuristic progress toward the goal.

## Results and Analysis

## Conclusion

## Publications
[//]: # "Note: Uncomment each once you uploaded the files to the repository"

<!-- 1. [Semester 7 report](./) -->
<!-- 2. [Semester 7 slides](./) -->
<!-- 3. [Semester 8 report](./) -->
<!-- 4. [Semester 8 slides](./) -->
<!-- 5. Author 1, Author 2 and Author 3 "Research paper title" (2021). [PDF](./). -->


## Links

[//]: # ( NOTE: EDIT THIS LINKS WITH YOUR REPO DETAILS )

- [Project Repository](https://github.com/cepdnaclk/repository-name)
- [Project Page](https://cepdnaclk.github.io/repository-name)
- [Department of Computer Engineering](http://www.ce.pdn.ac.lk/)
- [University of Peradeniya](https://eng.pdn.ac.lk/)

[//]: # "Please refer this to learn more about Markdown syntax"
[//]: # "https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet"
