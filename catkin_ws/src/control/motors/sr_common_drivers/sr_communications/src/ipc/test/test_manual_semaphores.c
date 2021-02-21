/**
 * @file        test_semaphores.c
 * @brief       The test file for semaphores.c.
 *
 * @author      Raul Perula-Martinez <raul.perula@uc3m.es>
 * @date        2015-03
 * @author      Victor Gonzalez <vgonzale@ing.uc3m.es>
 * @date        2010-01
 *
 * @copyright   Copyright (C) 2015 University Carlos III of Madrid.
 *              All rights reserved.
 * @license     LEUC3M v1.0, see LICENSE.txt
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the Licencia Educativa UC3M as published by
 * the University Carlos III of Madrid, either version 1.0, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY. See the Licencia Educativa UC3M
 * version 1.0 or any later version for more details.
 *
 * A copy of the Licencia Educativa UC3M is in the LICENSE file.
 */

#include <stdio.h>
#include <string.h>
#include "sr_communications/ipc/semaphores.h"

//#define DEFAULT_SEMFILE        AD_CORE_OUTPUTDIR "communications/ipc/semfile.tests"
#define DEFAULT_SEMFILE        "/tmp/communications/ipc/semfile.tests"

#define TRUE 1
#define FALSE 0

int main(int argc, char **argv)
{
    key_t semKey;
    int semID;
    char semFile[128];

    strcpy(semFile, DEFAULT_SEMFILE);

    if (createKey(&semKey, semFile) < 0) {
        perror("Error cretaing semaphore Key.\n");
        return (-1);
    }
    printf("Key created.\n");

    if (getSemID(semKey, &semID) < 0) {
        perror("Error in getSemID.\n");
        return (-2);
    }
    printf("getSemID successful\n");

    if (waitSem(semID) < 0) {
        perror("Error in waitSem.\n");
        return -3;
    }
    printf("waitSem successful\n");

    if (freeSem(semID) < 0) {
        perror("Error in freeSem().\n");
        return -4;
    }
    printf("freeSem successful\n");

    if (rmSem(semID) < 0) {
        perror("Error in rmSem().\n");
    }
    printf("rmSem successful\n");

    return 0;
}
