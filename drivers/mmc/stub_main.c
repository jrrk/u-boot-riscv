#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include "minion_lib.h"

int echo = 0;
static char linbuf[99];

void myputchar(char ch)
{
   putchar(ch);
}

void myputs(const char *str)
{
  while (*str)
    {
      myputchar(*str++);
    }
}

void myputn(unsigned n)
{
  if (n > 9) myputn(n / 10);
  myputchar(n%10 + '0');
}

void myputhex(unsigned n, unsigned width)
{
  if (width > 1) myputhex(n >> 4, width-1);
  n &= 15;
  if (n > 9) myputchar(n + 'A' - 10);
  else myputchar(n + '0');
}

int main()
{
  while (fgets(linbuf, sizeof(linbuf), stdin))
    {
      myputchar('\r');
      myputchar('\n');
      minion_dispatch(linbuf);
      myputchar('\r');
      myputchar('\n');
    }
}
