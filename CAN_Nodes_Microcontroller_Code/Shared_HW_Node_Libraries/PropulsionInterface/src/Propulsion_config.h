#ifndef __DAC_CONFIG__
#define __DAC_CONFIG__


#define DAC_SDA PB7
#define DAC_SCL PB6

#define DAC_Vin 4.994
#define DAC_offset -0.01

#define DAC_SCALE_VAL 4095
#define PIN_SCALE_VAL 1023
#define DAC_MAX_OUT_VOLTAGE 4.35
#define DAC_MIN_OUT_VOLTAGE 0.85

#define CAN_Propulsion_Scale_Down 0.001f //to make mV -> V
#define CAN_Propulsion_Scale_Up 1000.0f // V->mV

#define DAC_ACCELERATOR_ADDR 0x62 //Defined by MCP4725 module
#define DAC_BRAKE_ADDR 0x63

#endif