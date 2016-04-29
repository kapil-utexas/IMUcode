#include <stdio.h>
#include <rt_misc.h>
#include "retarget.h"
#include "usart.h"
#pragma import(__use_no_semihosting_swi)

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;
FILE __stderr;

int fputc(int ch, FILE *f) 
{
	  USART_WRITE_ONE((unsigned char) ch,Print_Port);
  return 0;
}

int fgetc (FILE *fp)  {
  return (0);
}


int fclose(FILE* f) {
return 0;
}

int ferror(FILE *f) {
  /* Your implementation of ferror */
  return EOF;
}
int fseek (FILE *fp, long nPos, int nMode)  {
  return (0);
}


int fflush (FILE *pStream)  {
  return (0);
}


void _ttywrch(int ch) 
{
//  serialWrite((unsigned char) ch, PRINTFPORT);
}


void _sys_exit(int return_code) {
label:  goto label;  /* endless loop */
}

