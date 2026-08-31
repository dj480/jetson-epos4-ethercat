/* Read-only bus diagnostic. Enumerates every slave SOEM finds and prints its
 * position, addresses, and identity so multi-drive topology can be verified
 * before any PDO/SDO configuration touches the drives. Never writes a
 * controlword or any object dictionary entry. */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "soem/soem.h"

static uint8 IOmap[4096];
static ecx_contextt ctx;

int main(int argc, char *argv[])
{
    const char *iface = (argc > 1) ? argv[1] : "enP8p1s0";

    printf("Starting EtherCAT scan on %s...\n", iface);

    if (!ecx_init(&ctx, iface))
    {
        printf("Failed to open interface %s\n", iface);
        return 1;
    }

    printf("Interface opened\n");

    int wkc = ecx_config_init(&ctx);
    printf("ecx_config_init() returned %d, ctx.slavecount = %d\n", wkc, ctx.slavecount);

    if (wkc <= 0)
    {
        printf("No slaves found\n");
        ecx_close(&ctx);
        return 1;
    }

    printf("%d slave(s) found\n\n", ctx.slavecount);

    /* Map process data so Obits/Ibits reflect the default PDO assignment
     * read from each slave's EEPROM, without sending any process data. */
    ecx_config_map_group(&ctx, IOmap, 0);

    for (int i = 1; i <= ctx.slavecount; i++)
    {
        ec_slavet *s = &ctx.slavelist[i];

        printf("Slave %d:\n", i);
        printf("  Name           = %s\n", s->name);
        printf("  Configured addr= 0x%04X\n", s->configadr);
        printf("  Alias addr     = 0x%04X\n", s->aliasadr);
        printf("  Vendor ID      = 0x%08X\n", s->eep_man);
        printf("  Product code   = 0x%08X\n", s->eep_id);
        printf("  Revision       = 0x%08X\n", s->eep_rev);
        printf("  Serial number  = 0x%08X\n", s->eep_ser);
        printf("  State          = 0x%02X\n", s->state);
        printf("  Output bits    = %d\n", s->Obits);
        printf("  Input bits     = %d\n", s->Ibits);
        printf("\n");
    }

    ecx_close(&ctx);

    return 0;
}
