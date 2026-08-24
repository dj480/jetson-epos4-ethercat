/* Link smoke test for SOEM. It intentionally does not open an interface, so
 * it can run without EtherCAT hardware or network permissions. */
#include <stdio.h>
#include "soem/soem.h"

int main()
{
	/* Minimal build and link smoke test for the SOEM dependency. */
	printf("SOEM include test passed!\n");
	return 0;
}
