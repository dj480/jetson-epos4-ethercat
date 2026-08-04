#include <stdio.h>

#include "soem/soem.h"


static ecx_contextt ctx;

char IOmap[4096];


int main(int argc, char *argv[])

{

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


    ecx_config_init(&ctx);


    printf("\nFound %d slave(s)\n", ctx.slavecount);


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
