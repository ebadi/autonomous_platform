/*
Autonomous Platform Generation 4 interface for SpeedSensors, 
used for steering the segways propulsion

Manual/Datasheet:https://www.faranux.com/product/lm393-motor-speed-measuring-sensor-module-for-arduino-com33/
Interupts: https://www.electronicshub.org/working-with-interrupts-in-stm32f103c8t6/#Hardware_Interrupts

Created by: Fredrik and Erik - Thesis2023

Changelog:

+2023-04-05:    First Draft Finnished
                    fully functional library 
*/


// IMPORTANT NOTE:::::

/* TO PUT IN MAIN SETUP WHEN USING LIBRARY!!!!!!
void OBJ_NAME_fcn_interupt()
{
    OBJ_NAME.update();
}
*/
#include <Arduino.h>
#include "Stream.h"
#ifndef __Speed_Sensor__
#define  __Speed_sensor__

class Speed_Sensor_Interface{
    // PUBLIC VARIABLES:
    public:
    float _speed; // in m/s
    bool _sensor_ok;

    // PRIVATE VARIABLES:
    private:
    unsigned int _internal_pulses_counter; // nr of holes in disc
    unsigned int _nr_encoder_positions; //nr of 
    float _wheel_diameter; //m 
    unsigned int _update_timer_period;
    unsigned int _time_old;
    int _attach_pin;

    Stream *_debug_serial_port;  // serial port to write any debug information on, for debugging purposes.
    String last_error="HEJ"; //Saves Last error code encontered
    // Error Codes:
    String error_1 = "Speed is outside specified ranges"; 

    // PUBLIC FUNCTIONS:
    public: 
    Speed_Sensor_Interface(unsigned int nr_encoder_positions,float wheel_diameter, unsigned int update_timer_period, int attach_pin);
    unsigned int update();
    float speed_calculation();
    int get_pin();
    float get_Speed();
    void init_debug(Stream &inputDebugPort);
    bool get_sensor_ok();

    // PRIVATE FUNCTIONS:
    private:
        //private functions
    void add_error_code(String new_error_code);
    void SendInformationDebugSerial(String inputDebugInformation);


};
#endif
