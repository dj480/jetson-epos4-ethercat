/*
 * Production C interface used by Python through ctypes.
 *
 * The EPOS4 exposes configuration and motion values as CiA 402 object
 * dictionary entries. This file performs SDO transactions against slave 1;
 * the Python layer supplies high-level requests such as "move relative".
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

 

/* Initialize EtherCAT and configure the drive's base settings. The caller
 * must successfully run this before any other motor_* function. The
 * interface name is an operating-system network-device name. */
int motor_init(const char *iface)

{

    const char *interface_name = (iface != NULL) ? iface : "enP8p1s0";

 

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

 

    /* 0x6060 is Modes of Operation. Value 1 selects Profile Position Mode,
     * which accepts a target position and a setpoint toggle. */

    int8_t mode = 1;

    ecx_SDOwrite(&ctx, 1, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTRXM);

 

    int8_t mode_display = 0;

    int msize = sizeof(mode_display);

    ecx_SDOread(&ctx, 1, 0x6061, 0x00, FALSE, &msize, &mode_display, EC_TIMEOUTRXM);

    printf("Mode Display = %d\n", mode_display);

 

    /* 0x6081 is Profile Velocity, used by later profile-position moves unless
     * the caller changes it with motor_set_velocity. */

    uint32_t profile_velocity = 5000;

    ecx_SDOwrite(&ctx, 1, 0x6081, 0x00, FALSE, sizeof(profile_velocity), &profile_velocity, EC_TIMEOUTRXM);

    printf("Profile Velocity = %u\n", profile_velocity);

 

    return 1;

}

 

/* Follow the CiA 402 state sequence to enable operation. The controlword
 * (0x6040) requests state changes and the statusword (0x6041) reports them:
 * Shutdown (0x0006), Switch On (0x0007), then Enable Operation (0x000F). */
int motor_enable(void)

{

    uint16_t ctrlwrd = 0;

    uint16_t statuswrd = 0;

    int csize = sizeof(ctrlwrd);

    int ssize = sizeof(statuswrd);

 

    // Read initial status

    ecx_SDOread(&ctx, 1, 0x6041, 0x00, FALSE, &ssize, &statuswrd, EC_TIMEOUTRXM);

    printf("Initial Status = 0x%04X\n", statuswrd);

 

    /* Shutdown */

    ctrlwrd = 0x0006;

    ecx_SDOwrite(&ctx, 1, 0x6040, 0x00, FALSE, csize, &ctrlwrd, EC_TIMEOUTRXM);

    osal_usleep(500000);

 

    ecx_SDOread(&ctx, 1, 0x6041, 0x00, FALSE, &ssize, &statuswrd, EC_TIMEOUTRXM);

    printf("After 0x0006 -> Status = 0x%04X\n", statuswrd);

 

    /* Switch On */

    ctrlwrd = 0x0007;

    ecx_SDOwrite(&ctx, 1, 0x6040, 0x00, FALSE, csize, &ctrlwrd, EC_TIMEOUTRXM);

    osal_usleep(500000);

 

    ecx_SDOread(&ctx, 1, 0x6041, 0x00, FALSE, &ssize, &statuswrd, EC_TIMEOUTRXM);

    printf("After 0x0007 -> Status = 0x%04X\n", statuswrd);

 

    /* Enable Operation */

    ctrlwrd = 0x000F;

    ecx_SDOwrite(&ctx, 1, 0x6040, 0x00, FALSE, csize, &ctrlwrd, EC_TIMEOUTRXM);

    osal_usleep(500000);

 

    ecx_SDOread(&ctx, 1, 0x6041, 0x00, FALSE, &ssize, &statuswrd, EC_TIMEOUTRXM);

    printf("After 0x000F -> Status = 0x%04X\n", statuswrd);

 

    printf("\nDrive Ready\n");

    return 1;

}

 

int motor_disable(void)

    /* Shutdown removes operation enable while leaving the EtherCAT session
     * available for another command or for an orderly close. */
{

    uint16_t ctrlwrd = 0x0006;

    ecx_SDOwrite(&ctx, 1, 0x6040, 0x00, FALSE, sizeof(ctrlwrd), &ctrlwrd, EC_TIMEOUTRXM);

    printf("Drive disabled\n");

    return 1;

}

 

int motor_set_velocity(uint32_t speed)

    /* Update object 0x6081 in the drive's configured native velocity units. */
{

    ecx_SDOwrite(&ctx, 1, 0x6081, 0x00, FALSE, sizeof(speed), &speed, EC_TIMEOUTRXM);

    printf("Profile Velocity = %u\n", speed);

    return 1;

}

 

int32_t motor_get_position(void)

    /* Object 0x6064 is the signed actual position in encoder counts. */
{

    int32_t actualpos = 0;

    int psize = sizeof(actualpos);

    ecx_SDOread(&ctx, 1, 0x6064, 0x00, FALSE, &psize, &actualpos, EC_TIMEOUTRXM);

    return actualpos;

}

 

/* Convert a relative request into an absolute target and trigger the move.
 * EPOS4 profile-position mode accepts an absolute target at 0x607A, so a
 * relative request first reads 0x6064 and writes current + requested counts.
 * The 0x0010 new-setpoint bit is then pulsed to latch that target. */
int motor_move_relative(int32_t move_counts)

{

    int32_t actualpos = motor_get_position();

    int32_t targetpos = actualpos + move_counts;

 

    printf("Current Position = %d\n", actualpos);

    printf("Target Position  = %d\n", targetpos);

 

    /* 0x607A is the target position object and uses encoder counts. */

    ecx_SDOwrite(&ctx, 1, 0x607A, 0x00, FALSE, sizeof(targetpos), &targetpos, EC_TIMEOUTRXM);

 

    /* Toggle bit 4, "new set-point", so the drive accepts the target. */

    uint16_t ctrlwrd = 0x000F;

    ecx_SDOwrite(&ctx, 1, 0x6040, 0x00, FALSE, sizeof(ctrlwrd), &ctrlwrd, EC_TIMEOUTRXM);

    osal_usleep(10000);

 

    ctrlwrd = 0x001F;

    ecx_SDOwrite(&ctx, 1, 0x6040, 0x00, FALSE, sizeof(ctrlwrd), &ctrlwrd, EC_TIMEOUTRXM);

    osal_usleep(10000);

 

    ctrlwrd = 0x000F;

    ecx_SDOwrite(&ctx, 1, 0x6040, 0x00, FALSE, sizeof(ctrlwrd), &ctrlwrd, EC_TIMEOUTRXM);

    osal_usleep(10000);

 

    printf("Move command issued\n");

 

    /* short sleep to yield, avoid long blocking that causes pulsed motion */
    osal_usleep(10000);

 

    int32_t newpos = motor_get_position();

    printf("New Position = %d\n", newpos);

 

    return 1;

}

 

/* Release SOEM after all commands have stopped. The continuous worker must
 * be stopped first so it cannot access ctx after it is closed. */
void motor_close(void)

{

    ecx_close(&ctx);

}

/* Continuous velocity thread implementation. */
static pthread_t _vel_thread;
static volatile int _vel_running = 0;
static int32_t _vel_step = 0;
static uint32_t _vel_interval_ms = 5;

static void *_vel_loop(void *arg)
{
    (void)arg;
    /* The drive is configured for profile position, so continuous motion is
     * approximated with many small relative moves. */
    while (_vel_running) {
        motor_move_relative(_vel_step);
        if (_vel_interval_ms)
            osal_usleep(_vel_interval_ms * 1000);
    }
    return NULL;
}

int motor_start_continuous(int32_t step_counts, uint32_t interval_ms)
{
    if (_vel_running)
        return 0; /* Do not create duplicate worker threads. */
    _vel_step = step_counts;
    _vel_interval_ms = interval_ms ? interval_ms : 20;
    _vel_running = 1;
    if (pthread_create(&_vel_thread, NULL, _vel_loop, NULL) != 0) {
        _vel_running = 0;
        return -1;
    }
    return 1;
}

int motor_stop_continuous(void)
{
    if (!_vel_running)
        return 0;
    /* Clear the flag before joining so the worker exits after its current
     * move instead of starting another one. */
    _vel_running = 0;
    pthread_join(_vel_thread, NULL);
    return 1;
}
