#ifndef _STRING_H_
#define _STRING_H_

#include <stddef.h>

int memcmp(const void *s1, const void *s2, unsigned n);
void *memmove(void *__dest, __const void *__src, size_t count);
void *memset(void *s1, int c, unsigned n);
char *strcpy(char *dest, const char *src);
unsigned strlen(const char *s);
int strcmp(const char *cs, const char *ct);
void *memchr(const void *s, int c, size_t count);
char *strchr(const char *s, int c);
void freertos_printf( const char *ctrl1, ...);
#endif
