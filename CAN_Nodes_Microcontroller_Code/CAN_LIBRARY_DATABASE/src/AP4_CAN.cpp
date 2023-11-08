#include "AP4_CAN.h"
#include "AP4_CAN_config.h"
AP4_CAN::AP4_CAN(int can_cs_pin, bool input_use_sw_filter) : canbuss_interface(can_cs_pin)
{
    is_initialized = false;
    use_sw_filter = input_use_sw_filter;
}

// interrupt read function - Reads data recieved on canbuss and enters it into the internal buffer
void AP4_CAN::InterruptReadCAN()
{
    struct can_frame can_recieve_msg;

    //replace if with while?
    while(canbuss_interface.readMessage(&can_recieve_msg) == MCP2515::ERROR_OK)
    {

        //if can msg id is in list of msgs to read
        //https://www.techiedelight.com/check-vector-contains-given-element-cpp/ 
        if(std::count(can_sw_filter.begin(), can_sw_filter.end(), can_recieve_msg.can_id) || !use_sw_filter)
        {
            //convert from can msg to uint64_t
            uint64_t data_to_pack = ((uint64_t)can_recieve_msg.data[7] << 56) | 
                    ((uint64_t)can_recieve_msg.data[6] << 48) | 
                    ((uint64_t)can_recieve_msg.data[5] << 40) | 
                    ((uint64_t)can_recieve_msg.data[4] << 32) | 
                    ((uint64_t)can_recieve_msg.data[3] << 24) | 
                    ((uint64_t)can_recieve_msg.data[2] << 16) | 
                    ((uint64_t)can_recieve_msg.data[1] << 8) | 
                    ((uint64_t)can_recieve_msg.data[0] << 0);

            //store data from can frame in object
            unpack_message(&can_storage_container, can_recieve_msg.can_id, data_to_pack, can_recieve_msg.can_dlc, -1 );
        }
        
    }
}


// Add a CAN frame ID to periodically send
bool AP4_CAN::AddPeriodicCanSend(int frame_id, int intervall)
{
    period_send_ids.push_back(frame_id);
    period_send_intervals.push_back((uint32_t)intervall);
    uint32_t new_timer = 0;
    period_send_intervall_internal_timers.push_back(new_timer);

    return true;
}

// Add read filter Ids, can messages are only saved if the read msg is in a locally saved list ccan_read_list
bool AP4_CAN::AddCANSWFilterId(int id)
{
    if (std::find(can_sw_filter.begin(), can_sw_filter.end(),id)!=can_sw_filter.end())
    {
        can_sw_filter.push_back(id);

        return true;
    }
    else
        return false;
}

// Sends CAN frames with different periodicity
bool AP4_CAN::PeriodicCanSend()
{
    for(int i = 0; i < period_send_intervall_internal_timers.size(); i++)
    {
        // internal_timer_i += time_since_last_iteration
        uint32_t delta_time = (getCurrentMillis() - period_send_intervall_internal_timers[i]);

        if(delta_time >= period_send_intervals[i])
        {
            //send a CAN frame with this id

            struct can_frame can_send_msg;
            can_send_msg.can_id = period_send_ids[i];

            uint64_t data_out = 0;

            pack_message(&can_storage_container, can_send_msg.can_id, &data_out);

            //uint64_t to can msg https://github.com/howerj/dbcc 
            
            can_send_msg.data[7] = data_out >> 56;
	        can_send_msg.data[6] = data_out >> 48;
	        can_send_msg.data[5] = data_out >> 40;
	        can_send_msg.data[4] = data_out >> 32;
	        can_send_msg.data[3] = data_out >> 24;
	        can_send_msg.data[2] = data_out >> 16;
	        can_send_msg.data[1] = data_out >>  8;
	        can_send_msg.data[0] = data_out >>  0;

            can_send_msg.can_dlc = 8;
            
            canbuss_interface.sendMessage(&can_send_msg);

            //reset timer for this ID
            period_send_intervall_internal_timers[i] = getCurrentMillis();

        }
    }

    return true;
}

bool AP4_CAN::SendSingleCanFrame(int frame_id){
    //send a CAN frame with this id

    struct can_frame can_send_msg;
    can_send_msg.can_id = frame_id;

    uint64_t data_out = 0;

    pack_message(&can_storage_container, can_send_msg.can_id, &data_out);
     //uint64_t to can msg https://github.com/howerj/dbcc 
            
    can_send_msg.data[7] = data_out >> 56;
    can_send_msg.data[6] = data_out >> 48;
    can_send_msg.data[5] = data_out >> 40;
    can_send_msg.data[4] = data_out >> 32;
    can_send_msg.data[3] = data_out >> 24;
    can_send_msg.data[2] = data_out >> 16;
    can_send_msg.data[1] = data_out >>  8;
    can_send_msg.data[0] = data_out >>  0;

    can_send_msg.can_dlc = 8;
            
    canbuss_interface.sendMessage(&can_send_msg);
    return true;
}


/*bool AP4_CAN::InitInterrupt(int can_int_pin)
{
    // attach interrupt guide https://circuitdigest.com/microcontroller-projects/how-to-use-interrupts-in-stm32f103c8

    attachInterrupt(digitalPinToInterrupt(can_int_pin),AP4_CAN::InterruptReadCAN,LOW); //maybe set to FALLING?
    return true;
}*/

bool AP4_CAN::InitMCP2515()
{
    //init SPI on stm32 bluepill

    //canbuss_interface = new MCP2515(can_cs_pin);
    SPI.setMOSI(Can_setMosi);
    SPI.setMISO(Can_setMiso);
    SPI.setSCLK(Can_setCLK);
    SPI.begin();

    if(canbuss_interface.reset()==MCP2515::ERROR::ERROR_OK && canbuss_interface.setBitrate(CAN_1000KBPS, MCP_8MHZ)==MCP2515::ERROR::ERROR_OK && canbuss_interface.setNormalMode()==MCP2515::ERROR::ERROR_OK){
        //Serial.println("Success");
        is_initialized = true;
        return true;
    }
    else{
        //Serial.println("failed");
        is_initialized = false;
        return false;
    } 

    is_initialized = true;
    return true;
}

// Retrieve any errors from the controller chip
uint8_t AP4_CAN::GetMCP2515Errors()
{
    return canbuss_interface.getErrorFlags();
}

uint64_t AP4_CAN::ui64_from_can_msg(unsigned char msg[8])
{
    return ((uint64_t)msg[7] << 56) | 
    ((uint64_t)msg[6] << 48) | 
    ((uint64_t)msg[5] << 40) | 
    ((uint64_t)msg[4] << 32) | 
    ((uint64_t)msg[3] << 24) | 
    ((uint64_t)msg[2] << 16) | 
    ((uint64_t)msg[1] << 8) | 
    ((uint64_t)msg[0] << 0);
}
bool AP4_CAN::ui64_to_can_msg(uint64_t input_uint64, unsigned char* msg[8])
{
    *msg[7] = input_uint64 >> 56;
	*msg[6] = input_uint64 >> 48;
	*msg[5] = input_uint64 >> 40;
	*msg[4] = input_uint64 >> 32;
	*msg[3] = input_uint64 >> 24;
	*msg[2] = input_uint64 >> 16;
	*msg[1] = input_uint64 >>  8;
	*msg[0] = input_uint64 >>  0;

    return true;
}