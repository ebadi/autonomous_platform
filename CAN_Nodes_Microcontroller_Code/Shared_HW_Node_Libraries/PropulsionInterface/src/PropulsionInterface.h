/*
Autonomous Platform Generation 4  interface for Digital to Analog Converter, MCP4725
used for steering the segways propulsion

Manual/Datasheet: https://ww1.microchip.com/downloads/en/devicedoc/22039d.pdf 

Created by: Fredrik and Erik - Thesis2023

Changelog:

+2023-04-05:    First Draft Finnished
                    fully functional library for demo test-rig.
*/



#ifndef __Digital_to_Analog_Converter__
#define  __Digital_to_Analog_Converter__


#include <Propulsion_config.h>
#include <Adafruit_MCP4725.h>
#include "Stream.h"


class DAC_controller_Interface{
    public:
    //public variables
    private:
    //private variables
    Stream *debug_serial_port;  // serial port to write any debug information on, for debugging purposes.
    String last_error="";  //Saves Last error code encontered
    bool Dac_ok;  //indiciates weather or not the error could be recovered from.

    double _v_in; //Input voltage to DAC modules
    double _DC_offset;

    Adafruit_MCP4725 dac_acc;
    Adafruit_MCP4725 dac_break; 

    bool _in_reverse;

    
    //Error Code Definitions:
    String error_1= "IC2 initial set up failed";
    String error_2 ="Voltage parameter in send_voltage() are not within propulsion controls limits";
    String error_3 = "Could not send new Voltage reference. IC2 communication has failed";
    String error_4="External sen_voltage, dac selection was incorrect!";
    String error_5="";
    
    public:
    //public functions
    // Constructor
    DAC_controller_Interface(double v_in,double DC_offset,Stream &inputDebugPort);
    DAC_controller_Interface();

    bool DAC_init(double v_in,double DC_offset,Stream &inputDebugPort);

    bool send_voltage(Adafruit_MCP4725 *dac, float voltage);
    bool send_voltage(String dac, float voltage);

    void slam_on_brake();
    void gentle_brake();
    void press_reverse();
    bool get_error_in_dac();
    bool get_in_reverse();


    private:
    //private functions
    void add_error_code(String new_error_code);
    void SendInformationDebugSerial(String inputDebugInformation);

};

#endif