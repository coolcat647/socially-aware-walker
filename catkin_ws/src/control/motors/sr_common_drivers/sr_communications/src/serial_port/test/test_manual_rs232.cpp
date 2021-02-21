/**
 * @file        test_rs232.c
 * @brief       The test file for RS232.c.
 *
 *              You will need a serial port to test this library. The program tries to
 *              communicate through the serial port with a unit of the Faulhaber's
 *              MCDC3006S driver. For that reason, all the configurations are made in
 *              order to allow the communication with it.
 *
 *              If you want to communicate with other device (i.e. using minicom, etc.)
 *              you will have to configurate it in a proper way or modify the code of
 *              this file.
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
#include "sr_communications/serial_port/rs232.h"

#define DEF_SERIAL_PORT_FILE    "/dev/brazo_derecho"
#define DEF_SEMFILE             "semfile.sem"

// If you want to use an specific command to test the communication, this is the default one :)
#define DEFAULT_COMMAND         "POS\n\r"

int main(int argc, char **argv)
{
    Rs232 rs;

    char spFile[SP_MAX_FILENAME_SIZE];
    char command[SP_MSG_SIZE], response[SP_MSG_SIZE];

    semaphore_t sem;        // Semaphore data type

    int baud;               // Baudrate

    sprintf(sem.semFile, DEF_SEMFILE);
    sprintf(spFile, DEF_SERIAL_PORT_FILE);

    printf("Semaphore file: %s\n", sem.semFile);
    printf("Serial port file: %s\n", spFile);

    printf("Initializing the communication with the serial port...\n");
    if (rs.initCommunication(baud, spFile, sem.semFile) < ERR_NOERR) {
        perror("Error in init communication\n");
        exit(-1);
    }

    sprintf(command, DEFAULT_COMMAND);
    printf("Sending command through the serial port...\n");
    if (rs.writeToRS232(command, strlen(command)) < ERR_NOERR) {
        perror("Error writing in the serial port\n");
        exit(-1);
    }

    printf("Comand sent. Now reading the response....\n");
    if (rs.readFromRS232(response) < ERR_NOERR) {
        perror("Error reading from the serial port\n");
        exit(-1);
    }

    printf("Response received.\n");
    printf("The command was: %s\n", command);
    printf("And the received response was: %s\n", response);
    printf("\n");

    printf("Trying again, but this time using askToRS232 function.\n");
    if (rs.askToRS232(command, strlen(command), response) < ERR_NOERR) {
        perror("Error asking to the serial port\n");
        exit(-1);
    }

    printf("Question sent and response received.\n");
    printf("The command was: %s\n", command);
    printf("And the received response was: %s\n", response);
    printf("\n");

    return 0;
}
