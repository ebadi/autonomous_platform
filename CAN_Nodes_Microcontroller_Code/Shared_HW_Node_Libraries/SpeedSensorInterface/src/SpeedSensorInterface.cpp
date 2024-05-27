#include <SpeedSensorInterface.h>


/*

=================== CONSTURCTOR ======================

*/

Speed_Sensor_Interface::Speed_Sensor_Interface(unsigned int nr_encoder_positions,float wheel_diameter, unsigned int update_timer_period, int attach_pin){
    _internal_pulses_counter = 0;
    _speed = 0;
    _nr_encoder_positions = nr_encoder_positions;
    _wheel_diameter = wheel_diameter;
    _update_timer_period = update_timer_period;
    _attach_pin = attach_pin;
    _sensor_ok = true;
    _time_old = 0;
    last_error = "HEJ DU";
}


unsigned int Speed_Sensor_Interface::update(){
    _internal_pulses_counter = _internal_pulses_counter + 1;
    return _internal_pulses_counter;
}


float Speed_Sensor_Interface::get_Speed(){
    return _speed;
}

int Speed_Sensor_Interface::get_pin()
{
    return _attach_pin;
}

bool Speed_Sensor_Interface::get_sensor_ok(){
    return _sensor_ok;
}

float Speed_Sensor_Interface::speed_calculation(){
    if( getCurrentMillis() - _time_old >= _update_timer_period)
    {
        _speed = (float) ( (float)_internal_pulses_counter /(float) _nr_encoder_positions) * 3.1415 * _wheel_diameter * (1000.0f / ((float)getCurrentMillis() - (float)_time_old));
        _time_old = getCurrentMillis();
        _internal_pulses_counter = 0;

        if (_speed*3.6 > 25 || _speed<float(-0.0001)){
            _sensor_ok = false;
            add_error_code(error_1);
        }
        else{
            _sensor_ok = true;
        }
    }
    SendInformationDebugSerial(last_error);
    return _speed;
}

void Speed_Sensor_Interface::init_debug(Stream &inputDebugPort){
    _debug_serial_port = &inputDebugPort;
}

/*

============================ PRIVATE FUNCTIONS ==========================

*/
void Speed_Sensor_Interface::add_error_code(String new_error_code){
    last_error = String(last_error + "\n\t" + new_error_code);
}

void Speed_Sensor_Interface::SendInformationDebugSerial(String inputDebugInformation) {
  if (_debug_serial_port->availableForWrite() && inputDebugInformation.length()>1) {
    _debug_serial_port->println(inputDebugInformation);
    delay(10); //otherwise write over printing message above
    last_error = "";
  }
}
