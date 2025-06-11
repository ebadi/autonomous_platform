Todo / Generic notes.

Considerations for design of speed sensor signaling in the AP4.

Sent the signal info as raw data to the HWI that will there calculate a speed.

Rationale: the actual measured speed will need to be derived by 2 parameters: speed sensor disk hole pattern and wheel circumference.  Those parameters are different for the front and rear wheel.

Defining the unity for raw data from the speed sensor on CAN:

- pulses/second.

With a design criteria of maximum 10 m/s, 35 of 60 hole speed disk and a circumference of appr 0.3m of a wheel, the max rotation
frequency of wheel can be around 30 rotations per second. With

speed = wheel circumference * rotations per second.

With Speed = 10 m/s
wheel cirumference = 0.69 (front wheel) and 0,78 (rear wheel)
gives
rps front = 10/0,78 =  12,8
rps rear = 10/0,69  = 14.5

with
holes per disk front = 35
holes per disk rear = 60

We get
pules/s front = 35 * 12,8 = 448\
pules/s rear = 60 * 14.5 = 870

The sensor is not able to see direction and will not be. Unless with using an analog input that could observe the edge flank of a signal and using a hole that would have an assymetic pattern. NOw a simpler approach was chosen, with digital I/O's that are only able to see a change from L/H or H/L. Pulses therefore do not get a direction and will only have zero or positive value.

So to code max 870 pulses 10 bits (2^10 = 1024 max) would be enough. For some additional space we can use 12 bits.

That will leave some space for status bits or additional info.
Diagnostics:
max speed overflow: a speed of max speed (0xFFF) should be considered as an overflow value (max speed in reality is equel to or higher than 4095 pulses/s).
Invalid: currently the speed sensor interpreter has no logic to handle invalid input data.

Blue Pill not initialied: LED will not light
CAN not initialized: LED will not light.
CAN be also seen in the terminal

As soon as CAN is initalized and the speed ECU can send CAN messages it will do so.

Send frequency: every 0.1 s.  Is now every second.

DBC file:

Speed sensor ECU:

assembling.

Mounting on the kart:

use

- m4 x10 screws
- m4 nut
  to fasten on the mount plate.
