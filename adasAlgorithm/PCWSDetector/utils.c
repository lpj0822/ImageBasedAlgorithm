#define AMBA_PLATFORM 0

#include <stdio.h>
#include <string.h>

#if AMBA_PLATFORM
#include <AmbaDataType.h>
#include <AmbaPrintk.h>
#include <AmbaKAL.h>
#include <AmbaShell.h>
#include <AmbaFS.h>
extern AMBA_KAL_BYTE_POOL_t G_MMPL;
#else
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#endif

void* my_malloc(int size)
{
    void* ptr = NULL;

#if AMBA_PLATFORM
    AmbaKAL_BytePoolAllocate(&G_MMPL, (void**)&ptr, size, 100);
#else
    ptr = malloc(size);
#endif

    return ptr;
}

void* my_calloc(int count, int size)
{
    void* ptr = NULL;

#if AMBA_PLATFORM
    AmbaKAL_BytePoolAllocate(&G_MMPL, (void**)&ptr, count*size, 100);
#else
    ptr = calloc(count,size);
#endif
    return ptr;
}

void* my_realloc(void *buffer, int nNew, int nOld)
{
	unsigned char *ptr=NULL;

	if(nNew>0)
    {
#if AMBA_PLATFORM
        AmbaKAL_BytePoolAllocate(&G_MMPL, (void**)&ptr, nNew, 100);

        if(ptr==NULL)
        {
            AmbaPrint("Mem_Alloc Error.\n");
        }
        memset(ptr,0,nNew);
        if(buffer != NULL)
        {
            if(nNew > nOld)
            {
                memcpy(ptr,buffer,nOld);
            }else{
                memcpy(ptr,buffer,nNew);
            }
            AmbaKAL_BytePoolFree(buffer);
        }

#else
        ptr = (unsigned char *)realloc(buffer, nNew);
#endif

    }
    return ptr;

}

void my_free(void* ptr)
{
#if AMBA_PLATFORM
    AmbaKAL_BytePoolFree(ptr);
#else
    free(ptr);
#endif
    ptr = NULL;
}

unsigned int my_gettime(void)
{
#if AMBA_PLATFORM
    return (unsigned int)AmbaSysTimer_GetTickCount();
#else
	static clock_t time = -1;
	clock_t cur;
	if (time == -1)
	{
		time = clock();
		return 0;
	}
	cur = clock();
	return (unsigned int)(cur - time);
#endif
}


void my_printf(const char *pFmt, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, pFmt);
    vsprintf(buffer,pFmt,args);
    va_end(args);

#if AMBA_PLATFORM
    AmbaPrint(buffer);
#else
    printf(buffer);
#endif
   
}

void* my_fopen(const char* filePath, const char* flag)
{
#if AMBA_PLATFORM
    AMBA_FS_FILE* result = NULL;
    result = AmbaFS_fopen(filePath, flag);
#else
    FILE* result = NULL;
    result = fopen(filePath, flag);
#endif


    if(!result)
    {
#if AMBA_PLATFORM
        AmbaPrint("Can not open file : %s\n", filePath);
#else
        printf("Can not open file : %s\n", filePath);
#endif
    }

    return result;
}

int my_fread(void* dest, int size, int nmemb, void* file)
{
    int readSize = 0;

#if AMBA_PLATFORM
    readSize =AmbaFS_fread(dest, size, nmemb, (AMBA_FS_FILE*)file);
#else
    readSize = fread(dest, size, nmemb, (FILE*)file);
#endif

    return readSize;
}

int my_fwrite(const void* ptr, int size, int nmemb, void* file)
{
    int writeSize = 0;

#if AMBA_PLATFORM
    writeSize = AmbaFS_fwrite(ptr, size, nmemb, (AMBA_FS_FILE*)file);
#else
    writeSize = fwrite(ptr,size,nmemb,(FILE*)file);
#endif

    return writeSize;
}

int my_fclose(void* file)
{
    int close = 0;

#if AMBA_PLATFORM
    close = AmbaFS_fclose((AMBA_FS_FILE*)file);
#else
    close = fclose((FILE*)file);
#endif

    return close;
}

