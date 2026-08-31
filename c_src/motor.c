/*
 * Production C interface used by Python through ctypes.
 *
 * The EPOS4 exposes configuration and motion values as CiA 402 object
 * dictionary entries. This file performs SDO transactions against whichever
 * slave the caller names; the Python layer supplies high-level requests such
 * as "move relative".
 */
#include <stdint.h>

#include <stdio.h>

#include <string.h>

#include <inttypes.h>



#include "soem/soem.h"

#include "motor.h"



/* SOEM process-data map and context shared by the exported motor functions.
 * IOmap is required by SOEM even though these commands use SDO transactions
 * rather than directly reading mapped process data. */
static uint8 IOmap[4096];

static ecx_contextt ctx;

/* Upper bound on how many slaves the per-slave continuous-motion arrays
 * below can track. The bus only ever has a couple of EPOS4 drives on it;
 * this is a generous cap, not a tuned limit. */
#define MOTOR_MAX_SLAVES 8



/* Reject a slave index outside the detected bus before it reaches an SDO
 * call, since SOEM does not itself bounds-check ctx.slavelist accesses. */
static int _valid_slave(int slave)
{
    if (slave < 1 || slave > ctx.slavecount)
    {
        printf("Invalid slave %d (bus has %d slave(s))\n", slave, ctx.slavecount);
        return 0;
    }
    return 1;
}

/* Initialize EtherCAT and configure every detected drive's base settings.
 * The caller must successfully run this before any other motor_* function.
 * The interface name is an operating-system network-device name. */
int motor_init(const char *iface)

{

    const char *interface_name = (iface != NULL) ? iface : "enP8p1s0";

    /* stdout is fully buffered by default when not attached to a terminal
     * (e.g. piped through Python's subprocess/ctypes host process), so
     * every printf below would otherwise sit invisible until the process
     * exits. Line-buffer it so callers see progress as it happens. */
    setvbuf(stdout, NULL, _IOLBF, 0);

    printf("Starting EtherCAT...\n");



    if (!ecx_init(&ctx, interface_name))

    {

        printf("Failed to open interface\n");

        return -1;

    }



    printf("Interface opened\n");



    if (ecx_config_init(&ctx) <= 0)

    {

        printf("No slaves found\n");

        ecx_close(&ctx);

        return -2;

    }



    printf("%d slave(s) found\n", ctx.slavecount);



    ecx_config_map_group(&ctx, IOmap, 0);

    ecx_configdc(&ctx);



    /* Apply the same base configuration to every slave found on the bus. */
    for (int slave = 1; slave <= ctx.slavecount; slave++)
    {
        /* 0x6060 is Modes of Operation. Value 1 selects Profile Position
         * Mode, which accepts a target position and a setpoint toggle. */
        int8_t mode = 1;
        ecx_SDOwrite(&ctx, slave, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTRXM);

        int8_t mode_display = 0;
        int msize = sizeof(mode_display);
        ecx_SDOread(&ctx, slave, 0x6061, 0x00, FALSE, &msize, &mode_display, EC_TIMEOUTRXM);
        printf("Slave %d: Mode Display = %d\n", slave, mode_display);

        /* 0x6081 is Profile Velocity, used by later profile-position moves
         * unless the caller changes it with motor_set_velocity. */
        uint32_t profile_velocity = 5000;
        ecx_SDOwrite(&ctx, slave, 0x6081, 0x00, FALSE, sizeof(profile_velocity), &profile_velocity, EC_TIMEOUTRXM);
        printf("Slave %d: Profile Velocity = %u\n", slave, profile_velocity);
    }



    return ctx.slavecount;

}



int motor_slave_count(void)
{
    return ctx.slavecount;
}



/* Follow the CiA 402 state sequence to enable operation. The controlword
 * (0x6040) requests state changes and the statusword (0x6041) reports them:
 * Shutdown (0x0006), Switch On (0x0007), then Enable Operation (0x000F). */
int motor_enable(int slave)

{

    if (!_valid_slave(slave))
        return -1;

    uint16_t ctrlwrd = 0;

    uint16_t statuswrd = 0;

    int csize = sizeof(ctrlwrd);

    int ssize = sizeof(statuswrd);



    // Read initial status

    ecx_SDOread(&ctx, slave, 0x6041, 0x00, FALSE, &ssize, &statuswrd, EC_TIMEOUTRXM);

    printf("Slave %d: Initial Status = 0x%04X\n", slave, statuswrd);



    /* Shutdown */

    ctrlwrd = 0x0006;

    ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, FALSE, csize, &ctrlwrd, EC_TIMEOUTRXM);

    osal_usleep(500000);



    ecx_SDOread(&ctx, slave, 0x6041, 0x00, FALSE, &ssize, &statuswrd, EC_TIMEOUTRXM);

    printf("Slave %d: After 0x0006 -> Status = 0x%04X\n", slave, statuswrd);



    /* Switch On */

    ctrlwrd = 0x0007;

    ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, FALSE, csize, &ctrlwrd, EC_TIMEOUTRXM);

    osal_usleep(500000);



    ecx_SDOread(&ctx, slave, 0x6041, 0x00, FALSE, &ssize, &statuswrd, EC_TIMEOUTRXM);

    printf("Slave %d: After 0x0007 -> Status = 0x%04X\n", slave, statuswrd);



    /* Enable Operation */

    ctrlwrd = 0x000F;

    ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, FALSE, csize, &ctrlwrd, EC_TIMEOUTRXM);

    osal_usleep(500000);



    ecx_SDOread(&ctx, slave, 0x6041, 0x00, FALSE, &ssize, &statuswrd, EC_TIMEOUTRXM);

    printf("Slave %d: After 0x000F -> Status = 0x%04X\n", slave, statuswrd);



    printf("Slave %d: Drive Ready\n", slave);

    return 1;

}



int motor_fault_reset(int slave)

    /* CiA 402 transition 15 (Fault -> Switch on disabled): a rising edge on
     * controlword bit 7 (0x0080) clears a fault. motor_enable's normal
     * shutdown/switch-on/enable-operation sequence has no effect while
     * faulted (the statusword's fault bit, 0x0008, stays set), so this is
     * a separate call. */
{

    if (!_valid_slave(slave))
        return -1;

    uint16_t statuswrd = 0;
    int ssize = sizeof(statuswrd);
    ecx_SDOread(&ctx, slave, 0x6041, 0x00, FALSE, &ssize, &statuswrd, EC_TIMEOUTRXM);
    printf("Slave %d: Status before fault reset = 0x%04X\n", slave, statuswrd);

    uint16_t ctrlwrd = 0x0080;
    ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, FALSE, sizeof(ctrlwrd), &ctrlwrd, EC_TIMEOUTRXM);
    osal_usleep(200000);

    ctrlwrd = 0x0000;
    ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, FALSE, sizeof(ctrlwrd), &ctrlwrd, EC_TIMEOUTRXM);
    osal_usleep(200000);

    ecx_SDOread(&ctx, slave, 0x6041, 0x00, FALSE, &ssize, &statuswrd, EC_TIMEOUTRXM);
    printf("Slave %d: Status after fault reset = 0x%04X\n", slave, statuswrd);

    return 1;

}



int motor_disable(int slave)

    /* Shutdown removes operation enable while leaving the EtherCAT session
     * available for another command or for an orderly close. */
{

    if (!_valid_slave(slave))
        return -1;

    uint16_t ctrlwrd = 0x0006;

    ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, FALSE, sizeof(ctrlwrd), &ctrlwrd, EC_TIMEOUTRXM);

    printf("Slave %d: Drive disabled\n", slave);

    return 1;

}



int motor_set_velocity(int slave, uint32_t speed)

    /* Update object 0x6081 in the drive's configured native velocity units. */
{

    if (!_valid_slave(slave))
        return -1;

    ecx_SDOwrite(&ctx, slave, 0x6081, 0x00, FALSE, sizeof(speed), &speed, EC_TIMEOUTRXM);

    printf("Slave %d: Profile Velocity = %u\n", slave, speed);

    return 1;

}



int motor_set_acceleration(int slave, uint32_t accel, uint32_t decel)

    /* Update objects 0x6083/0x6084. The drive otherwise keeps whatever
     * acceleration ramp it powered on with, which is unrelated to the
     * profile velocity set via motor_set_velocity. */
{

    if (!_valid_slave(slave))
        return -1;

    ecx_SDOwrite(&ctx, slave, 0x6083, 0x00, FALSE, sizeof(accel), &accel, EC_TIMEOUTRXM);

    ecx_SDOwrite(&ctx, slave, 0x6084, 0x00, FALSE, sizeof(decel), &decel, EC_TIMEOUTRXM);

    printf("Slave %d: Profile Acceleration = %u, Deceleration = %u\n", slave, accel, decel);

    return 1;

}



int motor_set_position_limits(int slave, int32_t min_pos, int32_t max_pos)

    /* Update objects 0x607D:01/0x607D:02. This is enforced by the drive's
     * own firmware against Profile Position targets, so it holds even if a
     * higher-level caller's own clamping has a bug. */
{

    if (!_valid_slave(slave))
        return -1;

    ecx_SDOwrite(&ctx, slave, 0x607D, 0x01, FALSE, sizeof(min_pos), &min_pos, EC_TIMEOUTRXM);

    ecx_SDOwrite(&ctx, slave, 0x607D, 0x02, FALSE, sizeof(max_pos), &max_pos, EC_TIMEOUTRXM);

    printf("Slave %d: Software Position Limit = [%d, %d]\n", slave, min_pos, max_pos);

    return 1;

}



int32_t motor_get_position(int slave)

    /* Object 0x6064 is the signed actual position in encoder counts. */
{

    if (!_valid_slave(slave))
        return 0;

    int32_t actualpos = 0;

    int psize = sizeof(actualpos);

    ecx_SDOread(&ctx, slave, 0x6064, 0x00, FALSE, &psize, &actualpos, EC_TIMEOUTRXM);

    return actualpos;

}



/* Write an absolute target to 0x607A and pulse the 0x0010 new-setpoint bit
 * so the drive latches it. Shared by motor_move_relative (which computes the
 * target from the current position) and motor_move_absolute (which is given
 * the target directly). Caller must have already validated slave. */
static int _issue_absolute_move(int slave, int32_t targetpos)

{

    printf("Slave %d: Target Position  = %d\n", slave, targetpos);



    /* 0x607A is the target position object and uses encoder counts. */

    ecx_SDOwrite(&ctx, slave, 0x607A, 0x00, FALSE, sizeof(targetpos), &targetpos, EC_TIMEOUTRXM);



    /* Toggle bit 4, "new set-point", so the drive accepts the target. */

    uint16_t ctrlwrd = 0x000F;

    ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, FALSE, sizeof(ctrlwrd), &ctrlwrd, EC_TIMEOUTRXM);

    osal_usleep(10000);



    ctrlwrd = 0x001F;

    ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, FALSE, sizeof(ctrlwrd), &ctrlwrd, EC_TIMEOUTRXM);

    osal_usleep(10000);



    ctrlwrd = 0x000F;

    ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, FALSE, sizeof(ctrlwrd), &ctrlwrd, EC_TIMEOUTRXM);

    osal_usleep(10000);



    printf("Slave %d: Move command issued\n", slave);



    /* short sleep to yield, avoid long blocking that causes pulsed motion */
    osal_usleep(10000);



    int32_t newpos = motor_get_position(slave);

    printf("Slave %d: New Position = %d\n", slave, newpos);



    return 1;

}



/* Convert a relative request into an absolute target and trigger the move.
 * EPOS4 profile-position mode accepts an absolute target at 0x607A, so a
 * relative request first reads 0x6064 and writes current + requested counts. */
int motor_move_relative(int slave, int32_t move_counts)

{

    if (!_valid_slave(slave))
        return -1;

    int32_t actualpos = motor_get_position(slave);

    int32_t targetpos = actualpos + move_counts;



    printf("Slave %d: Current Position = %d\n", slave, actualpos);

    return _issue_absolute_move(slave, targetpos);

}



/* Move directly to an absolute target position, skipping the current-position
 * read that motor_move_relative needs in order to compute its target. Used by
 * callers that already track a desired absolute position, such as the
 * arm-following controller. */
int motor_move_absolute(int slave, int32_t target_counts)

{

    if (!_valid_slave(slave))
        return -1;

    return _issue_absolute_move(slave, target_counts);

}



/* Release SOEM after all commands have stopped. Every slave's continuous
 * worker must be stopped first so none can access ctx after it is closed. */
void motor_close(void)

{

    ecx_close(&ctx);

}

/* Continuous velocity thread implementation, one worker per slave. */
static pthread_t _vel_thread[MOTOR_MAX_SLAVES + 1];
static volatile int _vel_running[MOTOR_MAX_SLAVES + 1];
static int32_t _vel_step[MOTOR_MAX_SLAVES + 1];
static uint32_t _vel_interval_ms[MOTOR_MAX_SLAVES + 1];

static void *_vel_loop(void *arg)
{
    int slave = (int)(intptr_t)arg;
    /* The drive is configured for profile position, so continuous motion is
     * approximated with many small relative moves. */
    while (_vel_running[slave]) {
        motor_move_relative(slave, _vel_step[slave]);
        if (_vel_interval_ms[slave])
            osal_usleep(_vel_interval_ms[slave] * 1000);
    }
    return NULL;
}

int motor_start_continuous(int slave, int32_t step_counts, uint32_t interval_ms)
{
    if (!_valid_slave(slave))
        return -1;
    if (slave > MOTOR_MAX_SLAVES) {
        printf("Slave %d exceeds MOTOR_MAX_SLAVES (%d)\n", slave, MOTOR_MAX_SLAVES);
        return -1;
    }
    if (_vel_running[slave])
        return 0; /* Do not create duplicate worker threads for this slave. */
    _vel_step[slave] = step_counts;
    _vel_interval_ms[slave] = interval_ms ? interval_ms : 20;
    _vel_running[slave] = 1;
    if (pthread_create(&_vel_thread[slave], NULL, _vel_loop, (void *)(intptr_t)slave) != 0) {
        _vel_running[slave] = 0;
        return -1;
    }
    return 1;
}

int motor_stop_continuous(int slave)
{
    if (!_valid_slave(slave))
        return -1;
    if (slave > MOTOR_MAX_SLAVES || !_vel_running[slave])
        return 0;
    /* Clear the flag before joining so the worker exits after its current
     * move instead of starting another one. */
    _vel_running[slave] = 0;
    pthread_join(_vel_thread[slave], NULL);
    return 1;
}
