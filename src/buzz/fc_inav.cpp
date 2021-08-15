#include "fc_inav.h"
#include "include/inc_fc/msp/msp_msg.hpp"
#include "include/inc_fc/msp/Client.hpp"
#include <iostream>
#include <sys/time.h>
// #include <vector>

#ifdef __cplusplus
extern "C"
{
#endif

/*the variables POSE and CMDS are being manipulated either by 
user input functions or otherwise, then CMDS is being used to
set the RC commands and POSE is being used to send positions 
to the FC to pursue position or velocity*/

float CMDS[6] = {1500,1500,900,1500,1000,1900}; //Roll, Pitch, Thrust, Yaw, Aux1, Aux2
float POSE[4] = {10,25,39,40};                      //position: x,y,z and Yaw wrt mocap frame
float DESIRED[4] = {0,0,0,0};                   //flag (0: poshold or 1:pos track or 2: vel track), x_des, y_des, z_des
int fc_mode = 0; //0: just hover, 1: track position, 2: track speed
float fc_voltage = 0;
bool reset_flag = false;
bool mocap_active = false;
int DONE = 0;

static pthread_t FC_THREAD;
static pthread_mutex_t LOCK_CMDS;

/*this function keeps sending command messages and 
collects needed information all the time, or else fc
goes to failsafe mode and freezes*/
void* fc_main_thread(void* args)
{
    std::vector<uint16_t> cmds(6, 1500);
    std::vector<int16_t> mocap_data(5, 0);
    std::vector<int16_t> desired_vec(4, 0);

    // std::vector<uint16_t> cmds(6, 1500);
    msp::client::Client client;    
    client.setLoggingLevel(msp::client::LoggingLevel::SILENT);
    client.setVariant(msp::FirmwareVariant::INAV);
    client.start(SERIAL_DEVICE, BAUDRATE);
    msp::FirmwareVariant fw_variant = msp::FirmwareVariant::INAV;

    // rebooting the drone
    msp::ByteVector  reboot_data = msp::ByteVector(0);
    bool reb = client.sendData(msp::ID::MSP_REBOOT,reboot_data);
    if(reb) std::cout<<"reboot successful\r\n";
    std::this_thread::sleep_for(std::chrono::seconds(10));

    /*setting up messages we need*/
    msp::msg::Debug debug(fw_variant);    //for debug messages in case we need them
    msp::msg::SetMocap mocap(fw_variant); //for sending the mocap data to fc 
    msp::msg::Analog analog(fw_variant);  //for voltage reading
    msp::msg::SetRawRc rc(fw_variant);    //for sending commands to fc
    msp::msg::SetDesVec desired_vector(fw_variant); //for sending desired position or velocity

    cmds[2] = 900;      //low thrust
    cmds[4] = 1000;     //motors not armed initially
    cmds[5] = 1800;     //NAV_POSHOLD mode initially

    /*setting up some time variables that we need*/
    struct timeval begin, current_time, last_ctrl_tmr, last_sens_tmr, last_mocap_tmr;
    gettimeofday(&begin, 0);
    gettimeofday(&last_ctrl_tmr,0);
    gettimeofday(&last_sens_tmr,0);
    gettimeofday(&last_mocap_tmr,0);

    long sec,usec;

    /*this loop goes on forever in an independent thread*/
    while(DONE != 1){
        int i;
        // setting appropriate messages contents
        for(i = 0; i < 6; i++)cmds[i] = CMDS[i];
        for(i = 1; i < 5; i++)mocap_data[i] = POSE[i-1];
        for(i = 0; i < 4; i++)desired_vec[i] = DESIRED[i];

        //check if we have received an order to reset
        if(reset_flag)
        {
            reb = client.sendData(msp::ID::MSP_REBOOT,reboot_data);
            if(reb) std::cout<<"reboot successful and waiting\r\n";
            WAIT(10);
            reset_flag = false;
            printf("finished waiting %d\n",reset_flag);
        }
        
        gettimeofday(&current_time, 0);
        long DT = current_time.tv_sec - begin.tv_sec;
        
        // if(DT > 5 && DT < 10)cmds[4] = 1800;
        // else cmds[4] = 1000;

        /* Sending RC control messages*/
        sec = current_time.tv_sec - last_ctrl_tmr.tv_sec;
        usec = current_time.tv_usec - last_ctrl_tmr.tv_usec; //this time is micro seconds
        double ctrl_time_elapsed = sec + usec*1e-6;
        if(ctrl_time_elapsed > CTRL_TIME_PERIOD){
            // std::cout<<DT<<std::endl;
            gettimeofday(&last_ctrl_tmr,0);
            rc.channels = cmds;
            reb = client.sendData(rc.id(),rc.encode());
            if(reb){}
        }
        
        /* Sending mocap value messages*/
        sec = current_time.tv_sec - last_mocap_tmr.tv_sec;
        usec = current_time.tv_usec - last_mocap_tmr.tv_usec; //this time is micro seconds
        double mocap_time_elapsed = sec + usec*1e-6;
        if(mocap_time_elapsed > MOCAP_TIME_PERIOD && mocap_active){
            gettimeofday(&last_mocap_tmr,0);
            mocap_data[0] = mocap_data[0] + 1;
            mocap.pose = mocap_data;
            reb = client.sendData(mocap.id(),mocap.encode());
            // client.sendMessageNoWait(mocap);
            if(reb){}
        }

        /* Collecting data and debug messages*/
        sec = current_time.tv_sec - last_sens_tmr.tv_sec;
        usec = current_time.tv_usec - last_sens_tmr.tv_usec; //this time is micro seconds
        double sens_time_elapsed = sec + usec*1e-6;
        if(sens_time_elapsed > SENS_TIME_PERIOD){
            gettimeofday(&last_sens_tmr,0);
            std::cout<<DT<<std::endl;
            if(client.sendMessage(analog) == 1){
                fc_voltage = analog.vbat;
                // std::cout << "time: " << DT << "  voltage is  "<<analog.vbat<<std::endl;  
            }
            if(client.sendMessage(debug) == 1) {
                std::cout << "#Debug message:" << std::endl;
                std::cout << debug.debug1 << " , " << 
                             debug.debug2 << " , " << 
                             debug.debug3 << " , " << 
                             debug.debug4 << std::endl;
            }
            else printf("no debug!");
        }
    }
    printf("done with the while loop\n");
    pthread_mutex_destroy(&LOCK_CMDS);
}

void fc_inav_main()
{
    if(pthread_mutex_init(&LOCK_CMDS, NULL) != 0) {
      fprintf(stderr, "Error initializing the command lock mutex: %s\n",
              strerror(errno));
      return ;
   }

    if(pthread_create(&FC_THREAD, NULL, &fc_main_thread, NULL) != 0) {
      fprintf(stderr, "Can't create FC_INAV thread: %s\n", strerror(errno));
   }
}

int fc_set_RC(buzzvm_t vm)
{
    buzzvm_lnum_assert(vm, 6);
    /* Push the vector components */
    buzzvm_lload(vm, 1);
    buzzvm_lload(vm, 2);
    buzzvm_lload(vm, 3);
    buzzvm_lload(vm, 4);
    buzzvm_lload(vm, 5);
    buzzvm_lload(vm, 6);
    buzzvm_type_assert(vm, 6, BUZZTYPE_FLOAT);
    buzzvm_type_assert(vm, 5, BUZZTYPE_FLOAT);
    buzzvm_type_assert(vm, 4, BUZZTYPE_FLOAT);
    buzzvm_type_assert(vm, 3, BUZZTYPE_FLOAT);
    buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
    buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);

    pthread_mutex_lock(&LOCK_CMDS);
    CMDS[0]=buzzvm_stack_at(vm, 6)->f.value; //roll
    CMDS[1]=buzzvm_stack_at(vm, 5)->f.value; //pitch
    CMDS[2]=buzzvm_stack_at(vm, 4)->f.value; //throttle
    CMDS[3]=buzzvm_stack_at(vm, 3)->f.value; //yaw
    CMDS[4]=buzzvm_stack_at(vm, 2)->f.value; //aux1
    CMDS[5]=buzzvm_stack_at(vm, 1)->f.value; //aux2
    pthread_mutex_unlock(&LOCK_CMDS);

    return buzzvm_ret0(vm);
}

int fc_get_voltage(buzzvm_t vm)
{
    buzzvm_pushs(vm, buzzvm_string_register(vm, "fc_voltage", 1));
    buzzvm_pushf(vm,fc_voltage);
    buzzvm_gstore(vm);
    return buzzvm_ret0(vm);
}

int fc_land(buzzvm_t vm)
{
    CMDS[2] = 1000;
    return buzzvm_ret0(vm);
}

int fc_takeoff(buzzvm_t vm)
{
    CMDS[2] = 1400;
    return buzzvm_ret0(vm);
}

int fc_reset(buzzvm_t vm)
{
    reset_flag = true;
    return buzzvm_ret0(vm);
}

int fc_arm(buzzvm_t vm)
{
    CMDS[4] = 1800;
    return buzzvm_ret0(vm);
}

int fc_disarm(buzzvm_t vm)
{
    CMDS[4] = 1000;
    return buzzvm_ret0(vm);
}

int fc_activate_mocap(buzzvm_t vm)
{
    mocap_active = true;
    return buzzvm_ret0(vm);
}

int fc_deactivate_mocap(buzzvm_t vm)
{
    mocap_active = false;
    return buzzvm_ret0(vm);
}


int fc_wait(buzzvm_t vm)
{
    printf("entered the wait function...\n");
    buzzvm_lnum_assert(vm, 1);
    /* Push the vector components */
    buzzvm_lload(vm, 1);
    buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
    float wait_time = buzzvm_stack_at(vm, 1)->f.value;
    struct timeval start_waiting,current_waiting;
    gettimeofday(&start_waiting, 0);
    double dt_wait = 0;
    long sec_,usec_;

    while(dt_wait < wait_time)
    {
        // printf("waiting...\n");
        gettimeofday(&current_waiting, 0);
        sec_ = current_waiting.tv_sec - start_waiting.tv_sec;
        usec_ = current_waiting.tv_usec - start_waiting.tv_usec; //this time is micro seconds
        dt_wait = sec_ + usec_*1e-6;
    }

    return buzzvm_ret0(vm);
}

void WAIT(float a)
{
    struct timeval start_waiting,current_waiting;
    gettimeofday(&start_waiting, 0);
    double dt_wait = 0;
    long sec_,usec_;

    while(dt_wait < a)
    {
        // printf("waiting...\n");
        gettimeofday(&current_waiting, 0);
        sec_ = current_waiting.tv_sec - start_waiting.tv_sec;
        usec_ = current_waiting.tv_usec - start_waiting.tv_usec; //this time is micro seconds
        dt_wait = sec_ + usec_*1e-6;
    }
}

// just a note for future reference: if you forget to put 
//"return buzzvm_ret0(vm)", the function will act weirdly if 
//it is being called (like a funtion calls itself more than 
// once when it was supposed to be called once, stuff like that)


int fc_dummy(buzzvm_t vm)
{
    printf("this is a dummy function!!!\n");
    return buzzvm_ret0(vm);
}
#ifdef __cplusplus
} // extern "C"
#endif