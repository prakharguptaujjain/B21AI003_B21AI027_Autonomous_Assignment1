# Multi-Agent Campus Tour System Using ROS2-foxy and CrewAI

## Authors

- Adarsh Raj Shrivastava (B21AI003)
- Prakhar Gupta (B21AI027)

## Introduction

This project implements a multi-agent system designed to facilitate campus tours using ROS2-foxy and the CrewAI framework. The system consists of three main agent types: Campus Incharge (CI), Building Incharge (BI), and Visitor agent. These agents interact with each other and the environment to guide visitor agents from the campus entrance to their required room within a building.

## System Architecture

The system is implemented in ROS2-foxy, with each node being a separate package:

1. `ci_agent`: Campus Incharge agent package
2. `bi_agent`: Building Incharge agent package
3. `visitor_agent`: Visitor agent package
4. `campus_map`: Campus map package
5. `map_publisher`: Map publisher package
6. `agent_interfaces`: Agent interface package

## Agent Communication Protocol

Agents communicate using two main services:

1. `EscortRequest.srv`: Used by the visitor agent to request escort from the campus incharge agent
2. `NavigationRequest.srv`: Used by the campus incharge agent to request a path to the required room within the building from the building incharge agent

## Message Flow

1. Visitor agent sends an escort request to the CI agent
2. CI agent escorts the visitor agent to the required building
3. CI agent sends a navigation request to the BI agent
4. BI agent provides the path to the required room
5. CI agent escorts the visitor agent to the required room

## Simulation Setup and Visualization

The system is simulated and visualized using RViz. The map publisher publishes the campus on the `/map` topic, which is visualized in RViz. Agents are spawned in the environment, and their paths are visualized with different colors:

- Dark blue: Campus incharge agent
- Light blue: Visitor agent

## Scenarios

The project includes various scenarios to demonstrate the system's functionality:

1. Standard escort to an authorized room
2. Escort request denial due to unauthorized access
3. Out-of-Service (OOS) request handling
4. Multiple visitor agents with a single CI agent

## Performance

Performance metrics for both CI and BI agents are provided, including:

- Successful escorts
- Average response time
- Request handling efficiency
- Access denials
- Visitor interaction time

## Installation and Setup

```bash
Refer Ros2 installation guide for installation of ROS2-foxy

Refer Rviz2 installation guide for installation of Rviz2
```

## Usage

To run a specific scenario:

```bash
ros2 run rviz2 rviz2
ros2 run map_publisher map_publisher
ros2 run bi_agent bi_agent
ros2 run ci_agent ci_agent
ros2 run visitor_agent visitor_agent
```

## Conclusion

This multi-agent system efficiently facilitates campus tours by coordinating CI, BI, and Visitor agents through a well-defined communication protocol. Using ROS2 Foxy and RViz for visualization, the system enables real-time simulation and evaluation of agent performance.

## Code and Resources

- Simulation Videos: Links to scenario videos are available in the full project report

For more detailed information, please refer to the full project report [here](https://github.com/prakharguptaujjain/B21AI003_B21AI027_Autonomous_Assignment1/blob/main/B21AI003_B21AI027_Assignment.pdf)