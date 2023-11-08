
## Extend High Level Software
This document aims to describe to process of how to extend the high level control software.

Before starting to develop and adding code to the high level control software you need to first make sure you need to add something here. 

If you;

* Want to do something with the digital twin
* Work on high level hardware agnostic autonomous drive algorithms
* Evaluate autonomous drive algorithms
* Are NOT adding new hardware
* Are NOT interfacing with the physical platform

Then you are in the right spot!! 

If not, take a look at `Hardware_Interface_Low_Level_Computer` or `CAN_Nodes_Microcontroller_Code`, maybe you intended to add functionality there!

### Prerequisites

In order to start adding functionality it is recommended to have a basic understanding of:

* C++ OR Python development
* docker containers (How to start, stop, restart and configure)
* Linux - The container software environment is mainly navigated in through a terminal
* Robot Operating System 2 (how to create packages and start new nodes)

Software wise, you need to have the following installed:

* docker
* git
* VSCode (recommended but any IDE may be suitable)

Hardware wise, it is recommended you have:

* Linux based x86 host computer, preferably with dedicated graphics (not a must) 


### Add a New Functionality

First of all make sure you have read the general design principles document for autonomous platform located at `autonomous_platform/HOW_TO_EXTEND.md`. This document takes precedence over anything written in this document in order to unify the development process across all software layers.


Software functionality is created inside ROS2 packages. These can be seen as code libraries that are configured to run and perform a specific task within a ROS2 network. 



