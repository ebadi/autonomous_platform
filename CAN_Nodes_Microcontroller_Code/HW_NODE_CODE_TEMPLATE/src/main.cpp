/*
TEMPLATE PLATFORMIO PROJECT FOR HARDWARE NODE USED IN AUTONOMOUS PLATFORM GENERATION 4
REMOVE THIS COMMENT SECTION
*/

/*
Here is a TLDR description of the HW ECU node

HW_NODE name: TEMPLATE_NODE
Hardware: STM32 Bluepill FC103xx

Purpose of this node:
example:
- Control of steering Motor

Sensor Inputs:
- Encoder readings

Actuator outputs:
- Serial commands for driver module
*/

/*
              Changelog:
_____________________________________________
|   NAME    |   DATE    |   Changes Made    |

*/


/*
================================================
Include Arduino libraries to use in project

HOW TO INCLUDE NEW LIBRARIES: Add library in platformio.ini file under lib_deps and they will be downloaded automatically during compiling
This can be done manually or through platformio's library manager.
*/
#include <Arduino.h> // Must Be Included since the PlattformIO uses Arduino Framework


/* Include config files */
#include "pin_config.h"
/*
=================================================
Include custom Controller Area Network (CAN) Database for Autonomous Platform Generation 4.
Make sure this platformio project is located in the CAN_Nodes_Microcontroller_Code directory in order for the relative path to work
Module Libraries are located in: SHARED_HW_Node_Libraries
adding paths to each one is done in platformio.ini
*/
#include "AP4_CAN.h"



/* Define and initialie global variables */
bool set_SW_filter = false;
AP4_CAN can_interface(CAN_CS_PIN, set_SW_filter);
uint32_t t;
bool recieved_interrupt = false;
/* Include node specific library dependencies */
// From the lib folder within each project


/*
AP4 CAN read interrupt function
// Interuptfunction declaration*/ 
void AP4CANInterrupt();



// ---------------------------- CODE: ------------------------------

/*
Arduino setup function or also called init
*/
void setup() {
  // note that the void loop() does not have scope to read variables declared within the setup()

  //Serial is used for debugging purposes, USB-micro connection cable of the STM32 Bluepill
  Serial.begin(9600);
  // Check if a USB micro cable is connected and serial communication can be connected
  // otherwise, moves on
  bool break_while=true;
  uint32_t t = getCurrentMillis();
  while(!Serial && break_while)
  {
    uint32_t dt=getCurrentMillis()-t;
    if(dt > (u_int32_t)3000)
    {
      break_while=false;
    }
  }

  // WARNING!!! This should only be necessary when there are two nodes in the system. remove later
  Serial.println("waiting 1 minute for raspberry pi to start!!!");
  delay(60000);

  // Set up of CAN buss, will be needed for every ECU:
  can_interface.InitMCP2515();

  // Set up of CAN buss
  can_interface.InitMCP2515();
  int nr_can_bus_init_attempts = 0;
  int max_attempts = 500;
  // Tries 500 times to setup the CAN bus
  while(can_interface.InitMCP2515() == false && nr_can_bus_init_attempts < max_attempts)
  {
    Serial.println("  INIT OF CANBUSS FAILED! Waiting 1 second and retrying");
    nr_can_bus_init_attempts++;
    delay(1000);
  }
  /*
  Configure what CAN signals should be sent (by ID) and in what Periodic intervall (ms) 
  */
  //can_interface.AddPeriodicCanSend(CAN_ID_FRAMENAME, 5000); See CAN_DB.h for frame names
  

  /*
  Configure what CAN signals should be read from the CAN bus (by Id), any other message ID will be ignored!!
  */
  //can_interface.AddCANSWFilter(CAN_ID_FRAMENAME);

  attachInterrupt(digitalPinToInterrupt(CAN_INTERRUPT_PIN), AP4CANInterrupt, LOW);

  Serial.println("Set up of CAN interface complete!");

  

  pinMode(LED_BUILTIN, OUTPUT); //internal LED will light up!
  Serial.println("Init of ECU is Complete!");
  Serial.println("");
}


/*
"WHILE True-loop"
*/
void loop() {
  // the loop function runs over and over again forever
  if(recieved_interrupt)
  {
  // Updates each value in the can_interface object
  can_interface.InterruptReadCAN();
  }

  /*
        GENERIC ECU CODE SHOULD BE ADDED HERE
  Dont forget to define if needed global variables before the setup loop
  Configure GPIO pins accordingly
  Add CAN frame handling (SPCU for inspiriation)
  Include module libraries at the top 
  NO heavy computations, only act as an interface!!
  */
 
}


void AP4CANInterrupt() 
{
  // detects intrupt at the INT pin
  recieved_interrupt = true;
}