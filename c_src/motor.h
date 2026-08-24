#ifndef MOTOR_H

#define MOTOR_H

/* Public C ABI used by the Python ctypes adapter. Counts and velocities use
 * EPOS4 native units; SOEM implementation details stay in motor.c. */

 

#include <stdint.h>

 

#ifdef __cplusplus

extern "C" {

#endif

 

/* Open the named network interface and configure the first EtherCAT slave. */
int motor_init(const char *iface);

/* Request the CiA 402 enabled state. */
int motor_enable(void);

/* Request the CiA 402 shutdown state. */
int motor_disable(void);

/* Set profile velocity in the drive's configured native units. */
int motor_set_velocity(uint32_t speed);

/* Move by signed encoder counts relative to the current position. */
int motor_move_relative(int32_t counts);

/* Read the signed actual position in encoder counts. */
int32_t motor_get_position(void);

/* Close SOEM after all motor activity has stopped. */
void motor_close(void);

/* Start/stop a background worker that issues repeated relative moves. */
int motor_start_continuous(int32_t step_counts, uint32_t interval_ms);
int motor_stop_continuous(void);

 

#ifdef __cplusplus

}

#endif

 

#endif // MOTOR_H