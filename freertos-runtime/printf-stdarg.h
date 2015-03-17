#ifndef _PRINTF_STDARG_H_
#define _PRINTF_STDARG_H_
int puts(const char *s);
int printf(const char *format, ...);
int sprintf(char *out, const char *format, ...);
int snprintf( char *buf, unsigned int count, const char *format, ... );
#endif
