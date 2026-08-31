#ifndef MOTOR_H

#define MOTOR_H

/* Public C ABI used by the Python ctypes adapter. Counts and velocities use
 * EPOS4 native units; SOEM implementation details stay in motor.c. Every
 * per-drive function takes a 1-based slave index matching its EtherCAT chain
 * position (as reported by motor_slave_count()/bin/scan_slaves), not
 * anything configured in EPOS Studio. */



#include <stdint.h>



#ifdef __cplusplus

extern "C" {

#endif



/* Open the named network interface and configure every EtherCAT slave found
 * on the bus (profile position mode, default profile velocity). Returns the
 * number of slaves configured, or a negative value on failure. */
int motor_init(const char *iface);

/* Number of slaves detected by the most recent motor_init() call. */
int motor_slave_count(void);

/* Request the CiA 402 enabled state for the given slave. */
int motor_enable(int slave);

/* Request the CiA 402 shutdown state for the given slave. */
int motor_disable(int slave);

/* Clear a CiA 402 fault (statusword bit 0x0008) so the drive can be
 * enabled again. motor_enable's normal state sequence has no effect while
 * faulted. */
int motor_fault_reset(int slave);

/* Set profile velocity in the drive's configured native units. */
int motor_set_velocity(int slave, uint32_t speed);

/* Set profile acceleration/deceleration (0x6083/0x6084) in the drive's
 * configured native units. Higher values ramp to the profile velocity
 * faster, which matters here because moves are frequent and small. */
int motor_set_acceleration(int slave, uint32_t accel, uint32_t decel);

/* Move by signed encoder counts relative to the current position. */
int motor_move_relative(int slave, int32_t counts);

/* Move directly to an absolute target position in encoder counts. */
int motor_move_absolute(int slave, int32_t target_counts);

/* Read the signed actual position in encoder counts. */
int32_t motor_get_position(int slave);

/* Write the CiA 402 software position limit (0x607D:01 min, 0x607D:02 max)
 * so the drive itself rejects/clips targets outside [min_pos, max_pos],
 * independent of any clamping done by a higher-level caller. Encoder
 * counts, same units as motor_get_position. */
int motor_set_position_limits(int slave, int32_t min_pos, int32_t max_pos);

/* Close SOEM after all motor activity has stopped. */
void motor_close(void);

/* Start/stop a background worker that issues repeated relative moves on the
 * given slave. Each slave has its own independent worker. */
int motor_start_continuous(int slave, int32_t step_counts, uint32_t interval_ms);
int motor_stop_continuous(int slave);



#ifdef __cplusplus

}

#endif



#endif // MOTOR_H
