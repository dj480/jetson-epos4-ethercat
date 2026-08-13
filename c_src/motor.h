#ifndef MOTOR_H

#define MOTOR_H

 

#include <stdint.h>

 

#ifdef __cplusplus

extern "C" {

#endif

 

int motor_init(const char *iface);

int motor_enable(void);

int motor_disable(void);

int motor_set_velocity(uint32_t speed);

int motor_move_relative(int32_t counts);

int32_t motor_get_position(void);

void motor_close(void);

/* Continuous velocity control: start a background thread that issues repeated
	relative moves of `step_counts` every `interval_ms` milliseconds. */
int motor_start_continuous(int32_t step_counts, uint32_t interval_ms);
int motor_stop_continuous(void);

 

#ifdef __cplusplus

}

#endif

 

#endif // MOTOR_H