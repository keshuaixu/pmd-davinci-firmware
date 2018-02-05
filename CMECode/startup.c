#include <stdio.h>

extern char _etext, _data, _edata, _bss, _ebss;

void CopyDataSectionToRAM()
{
	char *src = &_etext;
	char *dst = &_data;
	
	/* ROM has data at end of text; copy it. */
	while (dst < &_edata) {
		*dst++ = *src++;
	}
	
	/* Zero bss */
	for (dst = &_bss; dst< &_ebss; dst++)
		*dst = 0;

	stdout = stderr;
}
