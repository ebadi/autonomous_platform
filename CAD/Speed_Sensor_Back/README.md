Speed sensor assembly on the rear consists of the following components.

# Different components

## Holder Base

Purpose: connect to the chassis

3d printing recommendation

- copy the instance in ideamaker.
- mirror one of the instances (left/right are symmetric but needs mirroring)
- put them both flat on the base.
- create supports

Improvements:
tighter fitting to the chassis.

## Holder connector

Purpose:
provide a way to adjust for small chassis variations in X and Y.
Need to be seperated from Holder Base to allow for clearance for mounting near the wheel

3d printing recommendation

- copy the instance in ideamaker.
- mirror one of the instances (left/right are symmetric but needs mirroring)
- put them both flat on the base.

Improvements:
New component. Added m6 mutter inserts. Allows for X and Y adjustments

## Sensor Holder

Purpose: provide a stable mount for the speed sensor and an interface to the chassis base.
Improvements: allows for mounting of unmodified lm393 pcb without any adjustment.

Sensor ring:
ring that can optically read by the optical sensor

3d printing recommendation

- use rafts

# Complete assembly.

See as well included picture.

![Rear speed holder assembly](./mounted%20rear%20assembly.png)

Remoe middle wheelcap from Ninebot wheel. Use tiny screwdriver to prise between middle wheelcap and complete cap.
Unscrew the 4 screws that hold the bigger cap.
Moun the sensor ring on the wheel with 4 m3 screws.

Mount LM393 sensor on the Sesnor holder with one m3 screw.
Mount sesnor holder and pcb on the connector with 2 screws. Do not tighten yet.
Mount the holder base on the chasis with a m6 screw.
Mount at last the sesnor connector, holder and pcb assembly on the base with 2 screws.

Adjust in X and Y, so the optocoupler on the pcb is "looking through" the hear of the optical rim.

# Wiring

All LM393 pcb's need 3 wires.

Red: Power 5V
Green: D output
Black: Gnd
