/*
todo
*/

#ifndef __AP4_CAN_LIB__
#define __AP4_CAN_LIB__
#pragma once
/*
Include standard libraries
*/
#include <vector>
#include <string>
#include <Arduino.h>
#include <mcp2515.h> //CAN base library


/*
Include CAN_DB.h file

This header file defines every CAN frame sent on AP4

*/

#include "CAN_DB.h"

/*
CAN abstraction library for AP4
*/

class AP4_CAN {
//public variables
public:

can_obj_can_db_h_t can_storage_container;  //stores the data for all possible frames sent on AP4

//private variables
private:

bool is_initialized;
std::vector<int> period_send_ids;
std::vector<uint32_t> period_send_intervals;
std::vector<uint32_t> period_send_intervall_internal_timers;

std::vector<int> can_sw_filter;
std::vector<int> can_hw_filter;
bool use_sw_filter;

MCP2515 canbuss_interface;

//public functions
public:
//constructor(s), 
AP4_CAN(int can_cs_pin, bool input_use_sw_filter);

// interrupt read function - Reads data recieved on canbuss and enters it into the internal buffer
void InterruptReadCAN();

// Add a CAN frame ID to periodically send
bool AddPeriodicCanSend(int frame_id, int intervall);

// Add read filter Ids, can messages are only saved if the read msg is in a locally saved list ccan_read_list
bool AddCANSWFilterId(int id); //Returns true if successfully added to list
//bool AddCANHWFilterId(int id);


// Sends CAN frames with different periodicity
bool PeriodicCanSend();

// Send a CAN frame one single time
bool SendSingleCanFrame(int frame_id);

//bool InitInterrupt(int can_int_pin);
bool InitMCP2515();

// Retrieve any errors from the controller chip
uint8_t GetMCP2515Errors();

private:

uint64_t ui64_from_can_msg(unsigned char msg[8]);
bool ui64_to_can_msg(uint64_t input_uint64, unsigned char* msg[8]);

/*
hungover pseudo-code

call periodicCanSend(); often in main loop, like very often > 100 hz ?

for frame_id in frame_id_list
	timer_id += time since last iteration

	if timer_id > period_sent_interval
		send can frame with id = id
		timer_id = 0


ADD DATA TO FRAMES USING CAN_DB functions ie

int sensordata = 50;
AP4_CAN can_interface;

encode_frame_XxxXXxxx(&can_interface.can_obj, sensordata);
saves shit to the can_obj in at the right location

get shit from can frame
int output_val;
decode_frame_xxXxxXX(&can_interface.can_obj, &output_val)

hungover psuedo code
some sort of shit for shit to work on raspberry pi

#ifdef __ROS_INTERFACE__
#define __ROS_INTERFACE__

?

#endif

*/
};

#endif