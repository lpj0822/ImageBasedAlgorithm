#include "string_split.h"
#include <stdlib.h>
#include <string.h>

int strsplit (const char *str, char *parts[], const char *delimiter)
{		
	size_t i = 0;
	char *part = NULL;
	char *copy = NULL;

	if (str == NULL)
		return 0;

    int strLength = (int)strlen(str);
	copy = (char *)calloc(strLength + 1, sizeof(char));
	if (copy == NULL)
		return -1;
	strcpy(copy, str);

	part = strtok(copy, delimiter);
	strcpy(parts[i], part);
	i++;

	while (part)
	{
		part = strtok(NULL, delimiter);
		if (NULL == part)
			break;
		strcpy(parts[i], part);
		i++;
	}

	if (copy)
	{
		free(copy);
		copy = NULL;
	}

    return (int)i;
}

