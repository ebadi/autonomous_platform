#include <Arduino.h>
#include <PropulsionInterface.h>

// https://www.electronicshub.org/how-to-use-i2c-in-stm32f103c8t6/
// https://www.electronicshub.org/how-to-use-spi-in-stm32f103c8t6/

/*

============================ Constructor FUNCTIONS ==========================

*/
DAC_controller_Interface::DAC_controller_Interface(double v_in,double DC_offset,Stream &inputDebugPort){
    _v_in = v_in;
    _DC_offset = DC_offset;
    _in_reverse=false;
    debug_serial_port = &inputDebugPort;
    last_error = "";
}

DAC_controller_Interface::DAC_controller_Interface(){
    _v_in = -1;
    _DC_offset = -1;
    _in_reverse=false;
   last_error = "Propulsion DAC_Controller_interface object initialized with empty constructor and values have not been set";
}
/*

============================ PUBLIC FUNCTIONS ==========================

*/
bool DAC_controller_Interface::DAC_init(double v_in,double DC_offset,Stream &inputDebugPort)
{
    _v_in = v_in;
    _DC_offset = DC_offset;
    Dac_ok=true;
    _in_reverse = false;
    debug_serial_port = &inputDebugPort;
    last_error = "";
    
    // Setup of Serial Communication
    Wire.setSCL(DAC_SCL);
    Wire.setSDA(DAC_SDA);
    Wire.begin(); 

    // 
    if(dac_acc.begin(DAC_ACCELERATOR_ADDR) && dac_break.begin(DAC_BRAKE_ADDR)){
       //Succesful initial set up of IC2
    }
    else{
        add_error_code(error_1);
        Dac_ok = false;
    }

    send_voltage(&dac_break,(float)DAC_MIN_OUT_VOLTAGE);
    send_voltage(&dac_acc,(float)DAC_MIN_OUT_VOLTAGE);

    if(Dac_ok){
        debug_serial_port->println("\nGreat Success! DAC init complete\n");
    }

    return Dac_ok;
}


/*
sff
*/
bool DAC_controller_Interface::send_voltage(Adafruit_MCP4725 *dac,float voltage){
    //Serial.println("Voltage: " + String(voltage) + " , Comparison1: " + String(voltage > (float)4.5)+ "  Comparison 2: " + String(voltage < (float)0.85));
    if (voltage > (float)4.5 || voltage < (float)0.85)
    {
        Dac_ok = false;
        add_error_code(error_2);
        dac->setVoltage((0.85-_DC_offset)*DAC_SCALE_VAL/_v_in,false);

    }
    else
    {
        Dac_ok = true;
    }
    if(Dac_ok==1){ // OTHERWISE will be stuck in loop waiting for IC2 response
        dac->setVoltage((voltage-_DC_offset)*DAC_SCALE_VAL/_v_in,false);
    }
    else{
        Dac_ok = false;
        add_error_code(error_3);  
    }
    SendInformationDebugSerial(last_error);  
    return Dac_ok;
}


bool DAC_controller_Interface::send_voltage(String dac,float voltage){
    if (dac.equals("dac_acc")){
        send_voltage(&dac_acc,voltage);
        
    }
    else if (dac.equals("dac_break"))
    {
        send_voltage(&dac_break,voltage);
    }
    else
    {
        Serial.println("wrong dac name bruh");
        Dac_ok = false;
        add_error_code(error_4);
    }
    return Dac_ok;
}

void DAC_controller_Interface::slam_on_brake(){
    send_voltage(&dac_acc,DAC_MIN_OUT_VOLTAGE);
    delay(10);
    send_voltage(&dac_break,DAC_MAX_OUT_VOLTAGE);
    delay(500);
}


void DAC_controller_Interface::gentle_brake(){
    send_voltage(&dac_acc,DAC_MIN_OUT_VOLTAGE);
    delay(1000);
    send_voltage(&dac_break,DAC_MAX_OUT_VOLTAGE);
    delay(1000); //may include until speed=0;
    send_voltage(&dac_break,DAC_MIN_OUT_VOLTAGE);

}


void DAC_controller_Interface::press_reverse(){
    gentle_brake();
    delay(200);
    send_voltage(&dac_break,DAC_MAX_OUT_VOLTAGE);
    delay(50);
    send_voltage(&dac_break,DAC_MIN_OUT_VOLTAGE);

    delay(200);

    send_voltage(&dac_break,DAC_MAX_OUT_VOLTAGE);
    delay(50);
    send_voltage(&dac_break,DAC_MIN_OUT_VOLTAGE);
}
bool DAC_controller_Interface::get_in_reverse(){
    return _in_reverse;
}

bool DAC_controller_Interface::get_error_in_dac(){
    return Dac_ok;
}

/*

============================ PRIVATE FUNCTIONS ==========================

*/
void DAC_controller_Interface::add_error_code(String new_error_code){
    last_error = String(last_error + "\n\t" + new_error_code);
}

void DAC_controller_Interface::SendInformationDebugSerial(String inputDebugInformation) {
  if (debug_serial_port->availableForWrite() && inputDebugInformation.length()>1) {
    debug_serial_port->println(inputDebugInformation);
    delay(10); //otherwise write over printing message above
    last_error = "";
  }
}