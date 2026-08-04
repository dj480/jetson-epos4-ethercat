#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "soem/soem.h"

static uint8 IOmap[4096];
static ecx_contextt ctx;

int main(void)
{
    int32_t actualpos = 0;
    int32_t targetpos = 0;

    uint16_t ctrlwrd = 0;
    uint16_t statuswrd = 0;

    int8_t mode = 1;
    int8_t mode_display = 0;

    int psize = sizeof(actualpos);
    int ssize = sizeof(statuswrd);
    int csize = sizeof(ctrlwrd);
    int msize = sizeof(mode_display);

    char cmd;
    int32_t move_counts;

    printf("Starting EtherCAT...\n");

    if (!ecx_init(&ctx, "enP8p1s0"))
    {
        printf("Failed to open interface\n");
        return 1;
    }

    printf("Interface opened\n");

    if (ecx_config_init(&ctx) <= 0)
    {
        printf("No slaves found\n");
        ecx_close(&ctx);
        return 1;
    }

    printf("%d slave(s) found\n", ctx.slavecount);

    ecx_config_map_group(&ctx, IOmap, 0);
    ecx_configdc(&ctx);

    mode = 1;

    ecx_SDOwrite(
        &ctx,
        1,
        0x6060,
        0x00,
        FALSE,
        sizeof(mode),
        &mode,
        EC_TIMEOUTRXM);

    ecx_SDOread(
        &ctx,
        1,
        0x6061,
        0x00,
        FALSE,
        &msize,
        &mode_display,
        EC_TIMEOUTRXM);

    printf("Mode Display = %d\n", mode_display);

    ecx_SDOread(
        &ctx,
        1,
        0x6041,
        0x00,
        FALSE,
        &ssize,
        &statuswrd,
        EC_TIMEOUTRXM);

    printf("Initial Status = 0x%04X\n", statuswrd);

    /* Shutdown */
    ctrlwrd = 0x0006;
    ecx_SDOwrite(&ctx, 1, 0x6040, 0x00, FALSE,
                 csize, &ctrlwrd, EC_TIMEOUTRXM);

    osal_usleep(500000);

    ecx_SDOread(&ctx, 1, 0x6041, 0x00, FALSE,
                &ssize, &statuswrd, EC_TIMEOUTRXM);

    printf("After 0x0006 -> Status = 0x%04X\n", statuswrd);

    /* Switch On */
    ctrlwrd = 0x0007;
    ecx_SDOwrite(&ctx, 1, 0x6040, 0x00, FALSE,
                 csize, &ctrlwrd, EC_TIMEOUTRXM);

    osal_usleep(500000);

    ecx_SDOread(&ctx, 1, 0x6041, 0x00, FALSE,
                &ssize, &statuswrd, EC_TIMEOUTRXM);

    printf("After 0x0007 -> Status = 0x%04X\n", statuswrd);

    /* Enable Operation */
    ctrlwrd = 0x000F;
    ecx_SDOwrite(&ctx, 1, 0x6040, 0x00, FALSE,
                 csize, &ctrlwrd, EC_TIMEOUTRXM);

    osal_usleep(500000);

    ecx_SDOread(&ctx, 1, 0x6041, 0x00, FALSE,
                &ssize, &statuswrd, EC_TIMEOUTRXM);

    printf("After 0x000F -> Status = 0x%04X\n", statuswrd);

    printf("\nDrive Ready\n");

    while (1)
    {
        printf("\nCommands:\n");
        printf("  p            Print position\n");
        printf("  m <counts>   Move relative\n");
        printf("  e            Enable drive\n");
        printf("  d            Disable drive\n");
        printf("  q            Quit\n");
        printf("> ");

        scanf(" %c", &cmd);

        if (cmd == 'p')
        {
            psize = sizeof(actualpos);

            ecx_SDOread(
                &ctx,
                1,
                0x6064,
                0x00,
                FALSE,
                &psize,
                &actualpos,
                EC_TIMEOUTRXM);

            printf("Position = %d\n", actualpos);
        }

        else if (cmd == 'e')
        {
            ctrlwrd = 0x0006;
            ecx_SDOwrite(&ctx, 1, 0x6040, 0x00, FALSE,
                         csize, &ctrlwrd, EC_TIMEOUTRXM);

            osal_usleep(500000);

            ctrlwrd = 0x0007;
            ecx_SDOwrite(&ctx, 1, 0x6040, 0x00, FALSE,
                         csize, &ctrlwrd, EC_TIMEOUTRXM);

            osal_usleep(500000);

            ctrlwrd = 0x000F;
            ecx_SDOwrite(&ctx, 1, 0x6040, 0x00, FALSE,
                         csize, &ctrlwrd, EC_TIMEOUTRXM);

            osal_usleep(500000);

            printf("Drive enabled\n");
        }

        else if (cmd == 'm')
        {
            scanf("%d", &move_counts);

            psize = sizeof(actualpos);

            ecx_SDOread(
                &ctx,
                1,
                0x6064,
                0x00,
                FALSE,
                &psize,
                &actualpos,
                EC_TIMEOUTRXM);

            targetpos = actualpos + move_counts;

            printf("Current Position = %d\n", actualpos);
            printf("Target Position  = %d\n", targetpos);

            ecx_SDOwrite(
                &ctx,
                1,
                0x607A,
                0x00,
                FALSE,
                sizeof(targetpos),
                &targetpos,
                EC_TIMEOUTRXM);

            ctrlwrd = 0x000F;
            ecx_SDOwrite(&ctx, 1, 0x6040, 0x00, FALSE,
                         csize, &ctrlwrd, EC_TIMEOUTRXM);

            osal_usleep(100000);

            ctrlwrd = 0x001F;
            ecx_SDOwrite(&ctx, 1, 0x6040, 0x00, FALSE,
                         csize, &ctrlwrd, EC_TIMEOUTRXM);

            osal_usleep(100000);

            ctrlwrd = 0x000F;
            ecx_SDOwrite(&ctx, 1, 0x6040, 0x00, FALSE,
                         csize, &ctrlwrd, EC_TIMEOUTRXM);

            printf("Move command issued\n");

            osal_usleep(2000000);

            ecx_SDOread(
                &ctx,
                1,
                0x6064,
                0x00,
                FALSE,
                &psize,
                &actualpos,
                EC_TIMEOUTRXM);

            printf("New Position = %d\n", actualpos);
        }

        else if (cmd == 'd')
        {
            ctrlwrd = 0x0006;

            ecx_SDOwrite(
                &ctx,
                1,
                0x6040,
                0x00,
                FALSE,
                csize,
                &ctrlwrd,
                EC_TIMEOUTRXM);

            printf("Drive disabled\n");
        }

        else if (cmd == 'q')
        {
            ctrlwrd = 0x0006;

            ecx_SDOwrite(
                &ctx,
                1,
                0x6040,
                0x00,
                FALSE,
                csize,
                &ctrlwrd,
                EC_TIMEOUTRXM);

            printf("Drive disabled\n");
            break;
        }

        else
        {
            printf("Unknown command\n");
        }
    }

    ecx_close(&ctx);

    return 0;
}
