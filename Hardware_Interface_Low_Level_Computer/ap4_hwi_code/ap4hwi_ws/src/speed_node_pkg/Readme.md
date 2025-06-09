Requirements on Speed node package.

Needs to subscribe topics that are created by the CAN converter:

GET_0x5dc_SpeedSensorLF_PulseCnt
GET_0x5dc_SpeedSensorRF_PulseCnt
GET_0x5dc_SpeedSensorLR_PulseCnt
GET_0x5dc_SpeedSensorRR_PulseCnt

These topics publish the raw data values from the speed sensors (as pulse counts expressed in "pulses/second").

Requirements:

- The speed node shall have configurable parameters (ie to be retrieved from a yaml file).
- The speed node shall calculate a resulting speed per wheel
- The speed node shall calculate a resulting speed for the vehicle.
- the speed node shall provide diagnostics on the input values.
- Resulting output shall be published in an output topic, named in a sensible way. (ego speed, absolute value, vector, qf, diagnostics, age)
- Compilable as a seperate package, included in the ap4hwi_ws package.

Speed can be expected to be an absolute value, since this node can not not know if  the vehicle is moving forward/rearward only from the raw data from the speed sensors.

A more advanced node could also do more eloborate calculations and present the speed as being a vector.
Adding a direction vector.
Adding diagnosis on wheel spin.
Taking reversing signal and adding that to the direction.
Provide a QF thhat gets "invalid" or "bad" if relevant input was old, missing or out of range.

Ideas:

if l/r front wheel speeds differ, and/or resulting rear axle speeds differ from resulting front axle speed, it can be assumed that the vehicle is cornering.
If the (driven) rear wheels do have different speeds, it can also be a result of wheel spin, either caused by excessive brake or acceleratino torque.

If a sensor provides only zero as value, either the sensor can be broken or the wheel is not rotating.
If the speed value topics are not updated it is likely that the speed sensor ecu is not working.

Speed per wheel can be calculated as follows:

First calculate rotations per second.

R = pulsecount/ holes_per_disk

V = R / wheel_circumference

Needed configuration constants (to be specified in configuration file)
holes per disk rear wheel = 60
holes per disk front wheel = 35

circumference front wheel = 0.69 m
circumference rear wheel = 0.78 m
