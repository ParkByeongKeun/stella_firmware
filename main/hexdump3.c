#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>


void hexdump3(char *title, void *pack, size_t size) 
{ 
	int   idx = 0;

	char strTmp[4]    = {"\0"};
	char strAscii[32] = {"\0"};
	char strDump[64]  = {"\0"};
	char *dump        = NULL; 

	dump = (char *)pack;
	
	if ((size > 0) && (pack != NULL)) {
        printf("***** \x1B[35m%s\x1B[0m"
		     " \x1B[32m%d\x1B[0m bytes *****\r\n", (title == NULL) ? "None" : title, size);

		memset(strDump, 0, 64);
		memset(strAscii, 0, 32);

		for(idx = 0; idx < size; idx++) { 
			if    ((0x1F < dump[idx]) && (dump[idx] < 0x7F) ) { strAscii[idx & 0x0F] = dump[idx]; } 
			else                                              { strAscii[idx & 0x0F] = 0x2E;
			}

			snprintf(strTmp, 4, "%02X ", (unsigned char)dump[idx]); 
			strcat(strDump, strTmp); 
			if( (idx != 0) && ((idx & 0x03) == 0x03)) { strcat(strDump, " "); } 
		
			if((idx & 0x0F) == 0x0F) {
				printf("<0x%04X> %s%s\r\n", (idx & 0xFFF0), strDump, strAscii); 
				memset(strDump, 0, 64);
				memset(strAscii, 0, 32);
			}
		} 
		
		if (((size - 1) & 0x0F) != 0x0F) {
			for(idx = strlen(strDump) ; idx < 52; idx++) {
				strDump[idx] = 0x20;
			} 

			printf("<0x%04X> %s%s\r\n", (size & 0xFFF0), strDump, strAscii);
		} 
		
		printf("\r\n"); 
	} 
}

