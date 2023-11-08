/*
HW Node pinout configuration for STM32 bluepill fc10308
*/


#define LED_OK_PIN 13

// Configuration for CAN buss communication, same for all nodes:
#define CAN_INTERRUPT_PIN PB12
#define CAN_CS_PIN PA8 

/*
CONFIG for Propulsion Control PINS:

// https://www.electronicshub.org/how-to-use-i2c-in-stm32f103c8t6/
// I2C interface pins for STM32 Bluepill */
#define DAC_SDA PB7
#define DAC_SCL PB6
// IS DEFINED IN Propulsion_config in library path for constant defintions

/*
CONFIG definitions for STEERING CONTROL
*/
#define InputPortNr 1   
#define InputMotorNr 1  
#define InputMotorFixedSpeed 150 
#define InputWaitForResponseTime 1000
