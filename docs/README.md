---
layout: home
permalink: index.html

# Please update this with your repository name and title
repository-name: eYY-4yp-project-template
title: Actite localization and natigation
---

[comment]: # "This is the standard layout for the project, but you can clean this and use your own template"

# Project Title
Actite localization and natigation

#### Team

- e19167, H.D.N.S. JAYAWARDENA, [email](e19167@eng.pdn.ac.lk)
- e19423, Weerasingha W.A.C.J, [email](e19423@eng.pdn.ac.lk)



#### Supervisors

- Prof. Roshan Ragel, [email](roshanr@eng.pdn.ac.lk)
- Dr. Isuru Nawinna, [email](isurunawinne@eng.pdn.ac.lk)

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

Localization and mapping are crucial for swarm robotic systems to achieve coordinated autonomy in dynamic environments. Unlike single-robot SLAM, swarm robotics requires decentralized or cooperative approaches to efficiently share positional and environmental data among multiple agents. Techniques such as multi-agent SLAM, topological mapping, and probabilistic localization enable robust navigation while minimizing computational and communication overhead. Sensor fusion, incorporating LiDAR, IMU, and vision-based methods, enhances accuracy in GPS-denied environments.Focus to implement localization and mapping without using GPS for swarm system which includes UGC's and UAU's.Advanced strategies like collaborative loop closure, distributed pose graph optimization, and reinforcement learning improve adaptability and resilience in uncertain conditions. The integration of machine learning and neural networks further refines localization accuracy, making swarm robotic systems more efficient for applications in search-and-rescue, exploration, and industrial automation.

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

## Mathamatics

## Methodology

## Experiment Setup and Implementation

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
