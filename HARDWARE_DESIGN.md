## Hardware Design Principles

This document aims to collect the design principles for any new hardware added. Hardware may need to be added for different software layers, hence they can be summarized here.

See `HOW_TO_EXTEND.md` for general design principles. These should always be kept in mind.

### Modular Mounting of Components

This document is supposed to collect our thought about how sensors, hardware and components should be mounted on autonomous platform in order to make it as modular and adaptable as possible for future use.

### General idea

Use a metal sheet with pre fabricated metal holes as a base for mounting components. Create 3D printed fixtures to connect Hardware Nodes, Raspberry Pi 4 and other major components.

![Layout](Images/layout.PNG)

Using the prefabricated holes one can then create 3d printed fixtures in a standardized way to mount hardware anywhere along the sheet.

![Mounting idea](Images/mounting_idea.PNG)

![Component mounting](Images/component_mounting.PNG)

Specific solutions for this needs to be CADed later on.

### Mounting the aluminum plates to the frame

One can either use the pre existing mounting holes on the gokart chassis and screw directly into the chassis.

![Pre existing mounting holes on the gokart chassis](Images/pre_existing_holes.PNG)

Or one can create 3d printed fixtures to mount the plates, I.e:

![3d printed fixtures](Images/plate_fixture.PNG)

### Component list

- Aluminum metal sheet with be fabricated holes: Aluminum Lochblech blank 250x500x1.5mm article nr:  148-21-919 OR 4001116378041  https://www.elfa.se/en/aluminium-perforated-plate-round-holes-500x250x1-5mm-alfer-4001116378041/p/14821919 Holes are spaced 15 mm apart, hole diameter 4.5 mm

![Aluminum sheet](Images/Aluminium_sheet.PNG)

- M4 machine screws
- M4 nuts

One can either use pre existing cable routing hardware as in the image below

![Pre existing cable routine](Images/existing_cable_management.PNG)

or can 3d print simple cable holders such as this image

![3d print simple cable holders](Images/cable_management.PNG)

![3d print simple cable holders](Images/cable_management_above.PNG)
