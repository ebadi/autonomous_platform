"Improve wiring."

What should be improved?

Currently, the connections are hard to access, not very robust and hard to review.
Several components are dangling loose.

The level converters seem not necessary
3.3V unnecessary, can be directly retrieved from bluepill board.

So what need to be done?

Think that the solution should be:

Generic, modular, inspectable. Separation and marking
board/compoents should be attached to the box chassis/bottom/sidewall

- fusehholder ie...

No dangling components or connection points. Or components should be mounted directly to the node box or sub assembly components.

- like the fuse, level converter, bluepill board or wage clamps.

Multiple cables to the same board should be integrated in one multi pin conector
no overflow of cable length (max lenght of any cable should be max lenght of the box)

All wires directly routed from and to their connection.
No itntermediate connection points other then directly on a PCB.

When connecting a wire, connection or component it should not be necssary to remove anything else.

Från <https://dev.azure.com/TechDevLedning/AP-GTP/_sprints/taskboard/Autonomous%20Platform%20Inhouse/AP-GTP/AP%20-%20Inhouse/Sprint%205%20-%20Inhouse?workitem=12725>

Stability/robustness
\- Wires should not get loose if neighbouring pins are inspected
\- Wires should not get loose if the go kart is moved or driven around.
Easiness of adding/changing wires
Modular
a change of concept should not require redesign of the complete node.
Generic

Inspectable:
\- no need to move away bundles wires or pscb modules to see connections
\- Pins should be Accesible with multimeter.

No tape or glue to tie  components (including wires) to the box.
All wires connections made on fixed components or interconnection point on the pcb

Idea: add a universal pcb that can accomodate:
Add screw or connector rails.

Köp Fjäderplint 4-pol 2.54mm vertikal till rätt pris @ electrokit
Köp Skruvplint 2.54mm 6-pol till rätt pris @ electrokit

Design ideas:

Which boards to accomodate?

DC/DC 12V - 3,3 V   -REMOVE (can be supplied by bluepill regulator itself)

DC/DC 12V -5 V

CAN Board
LV converter - CAN Be REMOVED. IF proven necessary it can also be mounted on experiment pcb
Bluepill

Connection board?

Connections need

5V:
In  Feed
Out to CAN board      (internal)
Out to Bluepill board (internal)
Out to ST programmer  (external)
Out to LV converter (internal)

Out to Propulsion   (external)

3,3V (1 in, 6 out max)  CURRENT, can be REMOVED, since Bluepill can provide it's own 3.3V.

Only SPCU

IN Feed
Out to Bluepill
Out to LV converter

Out to Speed sensor 4x

Gnd ( 1 In, 6 out max)
In feed
Out to Bluepill
Out to CAN board
Out to LV converter (2x)
Out to Propulsion

Out to Speed sensor  4x

Control and Sensor I/O (max 4 connections currently

Control Kangaroo through  I2C: S1, S2
Control Propulsion: SCL, SDA

Sensor Speed:  4x digital input

Screw terminals

CAN connector

5V

CAN connector

CAN

CAN

3.3V

Pin headers

35 mm x 20 mm
40 x 20 mm freespace

Pin headers

Pin headers

Screw terminals

CAN board

Spring insert clips

40x28 mm min
45x28 mm freespace

Spring insert clips

Bluepill

Connection 1 board has:

Area for connecting  Voltage 1, Voltage 2, Ground 1, Ground 2
\- 1 screw terminal for each
\- 4 header pins

Area for connecting generic control/sensor I/O
\- with spring clips to outside connectio
\- Pin header for internal connections

Connection 12V board has

```
• 4 screw terminals for  +12V (after fuse)
• 4 screw terminals for gnd.
Pin header for fan.
```

Skip level converter with different Bluepill?

Tested: Bluepill on 5V and CAN MCP on 5V works without LV converters.

Test: bluepill on 3.3V with CAN MPC on 5V works as well (speed sensor node)

Hypothesis: LV converters only needed if BP only gets 3.3V?
Apparently not a with Bluepil on 3.3V and CAN MCP on 5V it also works.

Propose: skip LV converters.

What is the purpose of 12V power out? Not just easier with 12V power splitters?

12V clamps. Use those with 2 screws and nuts on the underside

Idea for fuse holders?

30A Chassis / Panel Mount Blade Fuse Holder - Altronics

TRU COMPONENTS Flatsäkringshållare Flatsäkring Standard 30 A 1 st | Conrad Electronic

Should we have 3.3V power or could the bluepill not just supply that.

The BP down converter can only power 300 mA which is fair enough

Ground supply. Should we seperate 5 and 3.3V grounds?
Should we have a 12v Entrance rail coupling point?
Basically to supply 3.3V, 5V DC board and fan.
