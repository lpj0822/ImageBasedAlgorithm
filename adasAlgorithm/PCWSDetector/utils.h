#ifndef __WS_UTILS_H__
#define __WS_UTILS_H__

extern void* my_malloc(int size);

extern void* my_calloc(int count, int size);

extern void* my_realloc(void *buffer, int nNew, int nOld);

extern void my_free(void* ptr);

extern unsigned int my_gettime(void);

extern void my_printf(const char *pFmt, ...);

extern void* my_fopen(const char* filePath, const char* flag);

extern int my_fread(void* dest, int size, int nmemb, void* file);

extern int my_fwrite(const void* ptr, int size, int nmemb, void* file);

extern int my_fclose(void* file);
#endif
