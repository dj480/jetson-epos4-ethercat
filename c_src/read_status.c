/* Minimal executable used to verify that the native build completed. It does
 * not open EtherCAT or move hardware; it only confirms the program runs. */
#include <stdio.h>

int main()
{
	/* Basic executable smoke test confirming the EPOS4 build completed. */
	printf("EPOS4 communication test\n");
	printf("Program compiled successfully\n");

	return 0;
}
