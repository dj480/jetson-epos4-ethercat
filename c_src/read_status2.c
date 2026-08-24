/* SOEM discovery utility: open an interface, enumerate slaves, print basic
 * identity/state information, and close the context without moving hardware. */
#include <stdio.h>

#include "soem/soem.h"


/* SOEM stores master/session state in this context. */
static ecx_contextt ctx;

/* SOEM requires an IO map during configuration even though this utility only
 * prints discovery information. */
char IOmap[4096];


int main(int argc, char *argv[])

{

    /* Confirm that the compiler and SOEM headers are available. */
    if(argc < 2)

    {

        printf("Usage: %s <interface>\n", argv[0]);

        return 1;

    }


    printf("Initializing EtherCAT on %s\n", argv[1]);


    if(!ecx_init(&ctx, argv[1]))

    {

        printf("Failed to initialize EtherCAT\n");

        return 1;

    }


    /* Configuration discovers slaves and populates ctx.slavelist. */
    ecx_config_init(&ctx);


    printf("\nFound %d slave(s)\n", ctx.slavecount);


    /* SOEM slave indexes begin at 1; index 0 represents the master. */
    for(int i = 1; i <= ctx.slavecount; i++)

    {

        ec_slavet *slave = &ctx.slavelist[i];


        printf("\n------------------------\n");

        printf("Slave %d\n", i);

        printf("Name : %s\n", slave->name);

        printf("State: %d\n", slave->state);

    }


    ecx_close(&ctx);


    return 0;

}
