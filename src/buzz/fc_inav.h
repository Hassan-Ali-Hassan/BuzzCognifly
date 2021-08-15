#ifndef FC_INAV
#define FC_INAV


#define CTRL_TIME_PERIOD 0.01
#define MOCAP_TIME_PERIOD 0.2
#define SENS_TIME_PERIOD 0.2
#define SERIAL_DEVICE "/dev/ttyS0"
#define BAUDRATE 115200

#ifdef __cplusplus
extern "C"
{
#endif

#include <buzz/buzzvm.h>
#include "buzzcognifly_closures.h"
#include <sys/time.h>

extern float CMDS[6];
extern float POSE[4];
extern int DONE;

/*main inav function execution*/
void fc_inav_main();
/*changes the values of command vector*/
int fc_set_RC(buzzvm_t vm);
/*gets voltage value*/
int fc_get_voltage(buzzvm_t vm);
/*lands drone*/
int fc_land(buzzvm_t vm);
/*takes drone off*/
int fc_takeoff(buzzvm_t vm);
/*resets fc*/
int fc_reset(buzzvm_t vm);
/*arms motors*/
int fc_arm(buzzvm_t vm);
/*disarms motors*/
int fc_disarm(buzzvm_t vm);
/*activate mocap functionality*/
int fc_activate_mocap(buzzvm_t vm);
/*deactivate mocap functionality*/
int fc_deactivate_mocap(buzzvm_t vm);


/*just a generic waiting function (takes float input)*/
int fc_wait(buzzvm_t vm);
int fc_dummy(buzzvm_t vm);
void WAIT(float);
#ifdef __cplusplus
} // extern "C"
#endif

#endif