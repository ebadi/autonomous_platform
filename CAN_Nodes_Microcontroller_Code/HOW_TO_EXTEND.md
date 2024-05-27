## How to extend Embedded Software

This document aims to describe to process of how to extend the hardware interface and low level software.

Before starting to develop and adding code to the embedded software you need to first make sure you need to add something here.

If you;

- Want to add an embedded sensor / actuator
- Need to add a new CAN signal or frame to existing sensor / actuator

Then you are in the right spot!!  If not, take a look at `Hardware_Interface_Low_Level_Computer` or `High_Level_Control_Computer`, maybe you intended to add functionality there!

### Prerequisites

In order to start adding functionality it is recommended to have a basic understanding of:

- C++ OR Python development
- Embedded Development
- CAN bus
- docker containers (How to start, stop, restart and configure)
- Basic Linux - The container software environment is mainly navigated in through a terminal

Software wise, you need to have the following installed:

- docker
- git
- VSCode (recommended but any IDE may be suitable)
- PlatformIO extension

Hardware wise, it is recommended you have:

- Linux based x86 host computer

### Add New Functionalities

First of all make sure you have read the general design principles document for autonomous platform generation 4 located at `autonomous_platform/HOW_TO_EXTEND.md`. This document takes precedence over anything written in this document in order to unify the development process across all software layers.

### Standard base ECU

Autonomous platform generation 4 uses a standard base ECU (Electronic Control Unit) meaning every embedded software should use the same hardware. The ECU or node box has a standardized Input and Output. ECUs are connected together in a CAN bus network, this network contains the Hardware Interface hardware, the Raspberry Pi 4b, which links the network with higher level software. Onto the base ECU specific hardware for a specific function can be added. For example, for the SPCU (Steering Propulsion Control Unit) there are DACs and Motor Controller hardware connected to the ECU.

Inputs:

- CAN bus passthrough (Through DB 9 connector)
- 12v Power (Through XT60 connector)

Outputs:

- CAN bus passthrough (Through DB 9 connector)
- 5v Power
- 3.3v Power
- GPIO pins

It is preferred to build a new ECU node when adding functionality to autonomous platform to prevent breaking existing functionality. ECUs should preferably

If new functionality cannot be implemented on an existing ECU node, a new hardware node must be built. See [How to Build new ECU](#how-to-build-a-new-ecu).

### build a new ECU

1. Start 3D printing generic ECU components (top, bottom, internal component holders)
   Please check `CAD` folder for the list of components that need to be printed.

The estimated material consumption (with reasonable slicer settings) is around 200 grams. And total print time is around 16-18 hours. Grab some coffee and leave it printing overnight! Make sure the first layers stick properly to the printing bed before leaving.

Note: Make sure you add supports manually for the print-in-place XT60 connector clampdowns. Any other support is not strictly necessary. These are circled in red below.

![Slicer supports](Resources/slicing_preperarations_supports.png){ width=70% }

2. Make sure all components are working individually before mounting them inside the node.
   I.e flashing the bluepill, measuring output from dc-dc converters, making sure the MCP2515 can bus board is working

A schematic illustration of the components inside the generic ECU can be seen below.

![ECU - speed sensor block diagram](Shared_HW_Node_Libraries/SpeedSensorInterface/documentation_and_research/diagrams/ECU-speed_sensor_wiring_overlay.drawio.png){ width=70% }

3. Once 3D print is complete, remove added support material for the XT60 clampdowns

![Slicer supports](Resources/assembling_new_ecu/1.jpg){ width=70% }

![Slicer supports](Resources/assembling_new_ecu/2.jpg){ width=70% }

4. Insert 12 M3 nuts in the hex shaped holes on the underside

Note: The hole circled in blue shall also have a m3 nut.

![Slicer supports](Resources/assembling_new_ecu/3.jpg){ width=70% }

5. Assemble power distribution and mount inside HW node.

A schematic for the power distribution inside the HW node can be found below.

![Slicer supports](Resources/assembling_new_ecu/4.jpg){ width=70% }

6. Mount power distribution inside node.

The XT60 contacts are inserted into the 3D printed clamp. The DC-DC converters are screwed in with 2 M3x10mm screws respectively.

![Slicer supports](Resources/assembling_new_ecu/5.jpg){ width=70% }

![Slicer supports](Resources/assembling_new_ecu/6.jpg){ width=70% }

7. Assemble CAN module and D-Sub 9 connectors.
   A wiring diagram can be found below.

![Slicer supports](Resources/assembling_new_ecu/7.jpg){ width=70% }

![Slicer supports](Resources/assembling_new_ecu/8.jpg){ width=70% }

8. Mount CAN module and D-Sub 9 connectors inside HW node.

D-Sub 9 connectors are screwed in from outside. The MCP2515 card is screwed in with two M3x10mm screws.

![Slicer supports](Resources/assembling_new_ecu/9.jpg){ width=70% }

![Slicer supports](Resources/assembling_new_ecu/10.jpg){ width=70% }

9. Insert 4 M3 nuts inside  STM32 hold down pillars. Make sure the hole in the pillars align with the inserted nuts.

![Slicer supports](Resources/assembling_new_ecu/13.jpg){ width=70% }

10. Wire programming pins for STM32 Bluepill

The programming wires shall have the following colours:

| **STM32F103C8T6 Bluepill Pin** | **Wire  Colour** |
| :----------------------------: | :--------------: |
|              3.3v              |       Red        |
|               IO               |      Brown       |
|              SCLK              |      White       |
|              GND               |      Black       |

![Slicer supports](Resources/assembling_new_ecu/14.jpg){ width=70% }

11. Using 4 STM32 bluepill clampdown (3D printed) tabs, clamp down the STM32 bluepill microcontroller.

![Slicer supports](Resources/assembling_new_ecu/15.jpg){ width=70% }

The four wiring cables shall be passed through to the side of the HW node.

![Slicer supports](Resources/assembling_new_ecu/16.jpg){ width=70% }

Cramp JST SM 2.54 female connectors on the outgoing programming wires. Insert a heat shrink cable over the cables.

![Slicer supports](Resources/assembling_new_ecu/16.1.jpg){ width=70% }

Insert the female JST SM 2.54 crimped connectors into a male JS connector as seen below, it is important that the colour and placement are the same between ECU nodes.

![Slicer supports](Resources/assembling_new_ecu/16.2.jpg){ width=70% }

12. Screw down the outgoing cable hold-down (3D printed component). Use two M3x16mm screws.

![Slicer supports](Resources/assembling_new_ecu/17.jpg){ width=70% }

13. Installing 12v dc fan for cooling. A cooling fan should be screwed into the top cover of the ECU using 4 M3 screws and nuts.

First, cut the fan cable in two pieces.

![Slicer supports](Resources/assembling_new_ecu/18.jpg){ width=70% }

Connect the two cables together using JST SM 2.54 2 pin connector. This is to make the top cover completely removable if desired.

![Slicer supports](Resources/assembling_new_ecu/19.jpg){ width=70% }

Solder the fan +12v and GND to the power in. (After the fuse). This can be done by connecting the wires to the dc-dc converters in terminals.

![Slicer supports](Resources/assembling_new_ecu/20.jpg){ width=70% }

The fan shall then be mounted to the top cover as illustrated below. Using 4 M3x16mm screws and 4 M3 nuts.

![Slicer supports](Resources/assembling_new_ecu/20.1.jpg){ width=70% }

![Slicer supports](Resources/assembling_new_ecu/20.2.jpg){ width=70% }

14. Power up module

To verify that the previous steps have been done correctly, connect 12v to the XT60 12v in plug on the front.

The expected result would be that the fan turns on and the dc-dc converters light up with a led shining.

The center dc-dc converter should output 3.3v and the dc-dc converter closest to the wall should output 5v.

If it does not power up correctly;

- Make sure you have inserted a 2A, 3A or 5A fuse into the fuse holder
- Check soldering connections using a multimeter

If dc-dc converter out voltage is not correct, adjust the blue potentiometers with a screwdriver until desired voltage is reached.g

![Slicer supports](Resources/assembling_new_ecu/21.jpg){ width=70% }

15. Internal wiring of standard components. This can be done with either jumper wires or soldering new wires between internal components.

Here is the wiring diagram for a generic ECU HW node.

The internal wiring of a ECU can be seen bellow. This has to be setup in order to run software on the bluepill microcontroller and communicate over CAN bus.

![Internal wiring schematic of generic ECU](Resources/EasyEDA/Schematic_generic_ecu_diagram_image.png){ width=70% }

16. Mount top cover onto ECU node.

The top cover can be mounted as pictured below using 4 M4x10mm screws and 4 M4 nuts.

![Slicer supports](Resources/assembling_new_ecu/22.jpg){ width=70% }

17. Connect any needed embedded sensor(s)

Note done the extra wiring required and how you connected it and add to the node specific documentation.

18. Flash new software

How to flash new software to the STM32 bluepill can be found in the `HOW_TO_PROGRAM_A_BLUEPILL.md` document.

### Extend with new function specific ECUs

These are the general guidelines and tips when adding a new function (or sensor) to a generic ECU base.

Questions to ask yourself;

- How can the sensor be connected to a microcontroller? Does it require any communication protocol or can you use a digital or analog pin?
- What power requirements does it have? High power or low power? If the sensor/actuator requires more power (and or different voltage) from what the generic ECU can supply you need to take this into consideration
- Am I adding new functionality to an existing ECU or am I adding completely new functionality? Maybe it is not necessary to build a new ECU node from scratch
- What parts do I have in inventory?
- What parts do I have to buy?

Very briefly explained, here are the steps you should take to implement new functionality with embedded sensors or actuators.

- Add specific hardware modules

- Built a small test-rig with a microcontroller & breadboard to verify functionalities and software libraries needed.

- Build, solder and connect a generic ECU base (see SPCU). Interface the specific hardware module.

- Create a CAD model of the new sensor and how it should be mounted to the platform. (Make sure it fits the standardized hole pattern if it should be mounted to an ECU or aluminum sheet)

- If needed, update the CAN protocol by editing the dbc file and autogenerate new database in c-format.

- Copy the Template code for ECUs.

- Flash both the ECU and central master computer (update the CAN to ROS2 topic converter) with the latest CAN protocol. See `HOW_TO_EXTEND.md` in `Hardware_Interface_Low_Level_Computer` directory

- Add generic ECU code in the embedded software, follow the structure of the SPCU.

- When the embedded software of the ECU is verified, connect the to the rest of AP4 using a db9 Female-Female cable and a xt60 outlet to supply.

### Unified CAN database and how to modify

The CAN database follows dbc format, [CAN and dbc guide](https://www.csselectronics.com/pages/can-dbc-file-database-intro). The dbc file can be edited using a dbc-editor an example of this is the [KVASER-DATABASE-EDITOR](https://kvaser-database-editor.software.informer.com/2.4/). When opening the dbc file for AP4 in kvaser, it will look like the figure below:

![Images/kvaser.png](Resources/kvaser.png)

1. **Message Name** in the database. This is the frames defined for a CAN message. What id, Data length (bytes) etc.
1. **Signals** in each message. User defined regarding name, placement in the data-field, offsets, factor and so on.
1. **Graphical Illustration of datafield** for the selected message/frame in the database.

In order to make sense of the dbc files, the embedded software needs the information of the dbc file in a c format. Thus AP4 converts dbc file into c-functions and data types, in order to simply encode and decode. Using [howerj dbcc repo](https://github.com/howerj/dbcc) to convert.

![Images/dbc_to_c_illustration.png](Resources/dbc_to_c_illustration.png)

When the dbc is edited and saved, run the following command in a **Linux Machine** whilst located in path: `CAN_Nodes_Microcontroller_Code\CAN_LIBRARY_DATABASE`

```bash
. create_database_from_dbc_script.sh
```

Down below is a simple flowchart of the way of working with the CAN protocols auto generation and dbc editing.

![Images/CAN.png](Resources/CAN.png)

Down below can be seen how all the Software components refers to the same database meaning that a change in the CAN protocol at a single place will affect the whole system.

![Images/can_sw_com_detailed.png](Resources/can_sw_com_detailed.png)

Also can be seen above how the central master computer, converts CAN messages to ROS2 topics to be later used in higher level algorithms. The *CAN signals To ROS2 Topic Converter* is illustrated in the figure below, implemented on the Raspberry Pi4b that acts as an interface in the central unit. The converter also converts topics to CAN frames that are then transmitted onto the CAN network.

![Images/can_translator_illustration.png](Resources/can_translator_illustration.png)

### Adding New CAN Frames And Signals

When adding new CAN frames and signals, changes have to be made in the "CAN_DB.dbc" file, new C code has to generated as described in `CAN_Nodes_Microcontroller_Code\CAN_LIBRARY_DATABASE\README.md`. Lastly, the software in hardware interface and low level software has to be adjusted for the newly added frames. This is described in `Hardware_Interface_Low_Level_Computer\HOW_TO_EXTEND.md`.

When adding new CAN frames it is recommended to add new frames and signals instead of changing existing frames/signals. IF EXISTING CAN FRAMES/SIGNALS ARE CHANGED, EXISTING ECUS MUST BE REFLASHED WITH NEW SOFTWARE. If one appends new signals and frames to the database it not strictly necessary to re-flash existing ECUs.

Adding new CAN frames and or signals is described in detail in `CAN_Nodes_Microcontroller_Code\HOW_TO_EXTEND.md`
