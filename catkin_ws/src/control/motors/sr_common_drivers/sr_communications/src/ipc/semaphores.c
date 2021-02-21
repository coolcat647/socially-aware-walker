/**
 * @file        semaphores.c
 * @brief       Software API for an easy use of the *nix IPC system.
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

#include "sr_communications/ipc/semaphores.h"

//////////////////////////////////////////////////

int createKey(key_t * semKey, char * semFile)
{
    if (access(semFile, F_OK) != 0) {
        if (creat(semFile, 0777) == -1) {
            perror("createKey() - error creating file keys for semaphore\n");
            fprintf(stderr, "kcreateKey %s\n", semFile);
            return -1;
        }
    }

    if ((*semKey = ftok(semFile, 'S')) < 0) {
        perror("createKey() - error ftok ");
        return -2;
    }

    return (int) *semKey;
}

//////////////////////////////////////////////////

int getSemID(key_t semKey, int * semID)
{
    *semID = semget(semKey, 1, IPC_CREAT | 0777);
    if (*semID == -1) {
        perror("error semget");
        return -1;
    }

    if (semctl(*semID, 0, SETVAL, 1) < 0) {
        perror("error semctl");
        return -2;
    }

    return (int) *semID;
}

//////////////////////////////////////////////////

int rmSem(int semID)
{
    struct sembuf arg[] = {0,
                           0,
                           0};

    if (semctl(semID, 0, IPC_RMID, arg) < 0) {
        perror("rmSem() - error semctl");
        return -1;
    }

    return 0;
}

//////////////////////////////////////////////////

int waitSem(int semID)
{
    struct sembuf opIn[] = {0,
                            -1,
                            0}; /*  opIn.sem_num = 0;
     opIn.sem_op = -1;
     opIn.sem_flg = 0;
     */
    if (semop(semID, opIn, 1) < 0) {
        perror("waitSem() - error semop");
        return -1;
    }

    return 0;
}

//////////////////////////////////////////////////

int freeSem(int semID)
{
    struct sembuf opOut[] = {0,
                             1,
                             0};

    if (semop(semID, opOut, 1) < 0) {
        perror("freeSem() - error semop");
        return -1;
    }

    return 0;
}

//////////////////////////////////////////////////
