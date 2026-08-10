#include <stdint.h>

#include <stdio.h>

#include <string.h>

#include <inttypes.h>

 

#include "soem/soem.h"

#include "motor.h"

 

static uint8 IOmap[4096];

static ecx_contextt ctx;

 

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

 

    // Set Profile Position Mode (0x6060 = 1)

    int8_t mode = 1;

    ecx_SDOwrite(&ctx, 1, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTRXM);

 

    int8_t mode_display = 0;

    int msize = sizeof(mode_display);

    ecx_SDOread(&ctx, 1, 0x6061, 0x00, FALSE, &msize, &mode_display, EC_TIMEOUTRXM);

    printf("Mode Display = %d\n", mode_display);

 

    // Set default Profile Velocity (0x6081 = 5000)

    uint32_t profile_velocity = 5000;

    ecx_SDOwrite(&ctx, 1, 0x6081, 0x00, FALSE, sizeof(profile_velocity), &profile_velocity, EC_TIMEOUTRXM);

    printf("Profile Velocity = %u\n", profile_velocity);

 

    return 1;

}

 

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

{

    uint16_t ctrlwrd = 0x0006;

    ecx_SDOwrite(&ctx, 1, 0x6040, 0x00, FALSE, sizeof(ctrlwrd), &ctrlwrd, EC_TIMEOUTRXM);

    printf("Drive disabled\n");

    return 1;

}

 

int motor_set_velocity(uint32_t speed)

{

    ecx_SDOwrite(&ctx, 1, 0x6081, 0x00, FALSE, sizeof(speed), &speed, EC_TIMEOUTRXM);

    printf("Profile Velocity = %u\n", speed);

    return 1;

}

 

int32_t motor_get_position(void)

{

    int32_t actualpos = 0;

    int psize = sizeof(actualpos);

    ecx_SDOread(&ctx, 1, 0x6064, 0x00, FALSE, &psize, &actualpos, EC_TIMEOUTRXM);

    return actualpos;

}

 

int motor_move_relative(int32_t move_counts)

{

    int32_t actualpos = motor_get_position();

    int32_t targetpos = actualpos + move_counts;

 

    printf("Current Position = %d\n", actualpos);

    printf("Target Position  = %d\n", targetpos);

 

    // Write target position (0x607A)

    ecx_SDOwrite(&ctx, 1, 0x607A, 0x00, FALSE, sizeof(targetpos), &targetpos, EC_TIMEOUTRXM);

 

    // Setpoint toggle logic with exact delays

    uint16_t ctrlwrd = 0x000F;

    ecx_SDOwrite(&ctx, 1, 0x6040, 0x00, FALSE, sizeof(ctrlwrd), &ctrlwrd, EC_TIMEOUTRXM);

    osal_usleep(100000);

 

    ctrlwrd = 0x001F;

    ecx_SDOwrite(&ctx, 1, 0x6040, 0x00, FALSE, sizeof(ctrlwrd), &ctrlwrd, EC_TIMEOUTRXM);

    osal_usleep(100000);

 

    ctrlwrd = 0x000F;

    ecx_SDOwrite(&ctx, 1, 0x6040, 0x00, FALSE, sizeof(ctrlwrd), &ctrlwrd, EC_TIMEOUTRXM);

    osal_usleep(100000);

 

    printf("Move command issued\n");

 

    osal_usleep(2000000);

 

    int32_t newpos = motor_get_position();

    printf("New Position = %d\n", newpos);

 

    return 1;

}

 

void motor_close(void)

{

    motor_disable();

    ecx_close(&ctx);

}