#ifndef __SEMAPHORES_H_
#define __SEMAPHORES_H_

/**
 * @file        semaphores.h
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

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <unistd.h>
#include <sys/sem.h>
#include <sys/ipc.h>
#include <fcntl.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_SEMFILENAME_SIZE 128

    /**
     * @brief Structure where all semaphore related data is stored
     */
    typedef struct semaphore {
            char semFile[MAX_SEMFILENAME_SIZE];
            int semID;
            key_t semKey;
    } semaphore_t;

    /**
     * @brief Create keys from a file to be used by a semaphore.
     *
     * @param semKey pointer to the created key
     * @param semFile full path of the file. If the file does not exists it is created.
     *
     * @return -1 if an error while creating the file happened.
     * @return -2 if a file-to-key error happened
     * @return semKey in case of success
     */
    int createKey(key_t *, char[]);

    /**
     * @brief Gets a semaphore identification.
     *
     * @param semKey
     * @param semID is where the semaphore identificator is stored
     *
     * @return -1 if there is an error in semget operation
     * @return -2 if there is an error in semctl operation
     * @return semID in case of success
     */
    int getSemID(key_t, int * semID);

    /**
     * @brief Removes a semaphore set
     * @param semID an integer with semaphore identification.
     * @return 0 if everything goes correctly
     * @return -1 if there is an error in semop operation
     */
    int rmSem(int semID);

    /**
     * @brief waits for a semaphore until its resources are free
     * @param semID an integer which identifies the semaphore
     * @return 0 if everything goes correctly
     * @return -1 if there is an error in semop operation
     */
    int waitSem(int semID);

    /**
     * @brief Frees a taken semaphore
     * @param semID is the identifier of the semaphore
     * @return 0 if everything goes correctly
     * @return -1 if there is an error in semop operation
     */
    int freeSem(int semID);

#ifdef __cplusplus
}
#endif

#endif
