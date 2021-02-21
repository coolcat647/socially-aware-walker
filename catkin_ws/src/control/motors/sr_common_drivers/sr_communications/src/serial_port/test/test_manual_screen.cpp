/**
 * @file        test_screen.c
 * @brief       The test file for serial.c
 *
 *              You will need a serial port to test this library. The program tries to
 *              communicate through the serial port with a unit of the 4Dsystems screen.
 *              For that reason, all the configurations are made in
 *              order to allow the communication with it.
 *
 * @author      Raul Perula-Martinez <raul.perula@uc3m.es>
 * @date        2015-03
 * @author      David Garcia <dggodoy@ing.uc3m.es>
 * @date        2012-02
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

#define DEF_SERIAL_PORT_FILE    "/dev/ojo"
#define DEF_SEMFILE             "semfile.sem"

// If you want to use an specific command to test the communication, this is the default one :)
#define DEFAULT_COMMAND         "U"

int main(int argc, char **argv)
{
    Rs232 rs;

    char spFile[SP_MAX_FILENAME_SIZE];
    char command[SP_MSG_SIZE], response[SP_MSG_SIZE];

    int i;
    int result;

    semaphore_t sem;        // Semaphore data type

    int baud = 9600;        // Baudrate

    sprintf(sem.semFile, DEF_SEMFILE);
    sprintf(spFile, DEF_SERIAL_PORT_FILE);

    printf("Semaphore file: %s\n", sem.semFile);
    printf("Serial port file: %s\n", spFile);

    printf("Initializing the communication with the serial port...\n");
    if (rs.initCommunication(baud, spFile, sem.semFile) < ERR_NOERR) {
        perror("Error in initSerial\n");
        exit(-1);
    }

    sleep(1);

    sprintf(command, DEFAULT_COMMAND);
    printf("Sending command through the serial port...\n");
    if (rs.writeToRS232(command, 1/*sizeofchar(command)*/) < ERR_NOERR) {
        perror("Error writing in the serial port\n");
        exit(-1);
    }

    printf("Waiting the response...\n");
    bzero(response, SP_MSG_SIZE);
    result = rs.readSerial(response, 1);
    if (result < ERR_NOERR) {
        printf("The error was: %d\n", result);
        perror("Error reading from the serial port\n");
        exit(-1);
    }

    //sumamos 0x30 para que sean caracteres imprimibles
    for (i = 0; i < result; i++) {
        response[i] += 0x30;
    }
    printf("Response received.\n");
    printf("The command was: %s\n", command);
    printf("And the received response was: %s\n", response);
    printf("\n");

    sleep(5);

    bzero(command, SP_MSG_SIZE);
    command[0] = 0x56;
    command[1] = 0x01;

    printf("Sending command through the serial port...\n");
    if (rs.writeToRS232(command, 2/*sizeofchar(command)*/) < ERR_NOERR) {
        perror("Error writing in the serial port\n");
        exit(-1);
    }

    printf("Waiting the response...\n");
    bzero(response, SP_MSG_SIZE);
    result = rs.readSerial(response, 5);
    if (result < ERR_NOERR) {
        printf("The error was: %d\n", result);
        perror("Error reading from the serial port\n");
        //exit (-1);
    }

    //sumamos 0x30 para que sean caracteres imprimibles
    for (i = 0; i < result; i++) {
        response[i] += 0x30;
    }
    printf("Response received.\n");
    printf("The command was: %s\n", command);
    printf("And the received response was: %s\n", response);
    printf("\n");

    sleep(5);

    bzero(response, SP_MSG_SIZE);
    bzero(command, SP_MSG_SIZE);
    command[0] = 0x56;
    command[1] = 0x01;

    printf("(Asking) Sending command through the serial port and waiting the response...\n");
    if (rs.askToSerial(command, 2, response, 5) < ERR_NOERR) {
        perror("Error asking in the serial port\n");
        exit(-1);
    }

    //sumamos 0x30 para que sean caracteres imprimibles
    for (i = 0; i < result; i++) {
        response[i] += 0x30;
    }
    printf("Response received.\n");
    printf("The command was: %s\n", command);
    printf("And the received response was: %s\n", response);
    printf("\n");

    sleep(10);

    return 0;
}
