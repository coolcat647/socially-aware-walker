#ifndef __RS232_H__
#define __RS232_H__

/**
 * @file        rs232.h
 * @brief       Software API to deal with the RS232 port. This API was designed to interact with faulhaber's MCDC3006S
 *              driver using the serial port interface.
 *
 * @author      Raul Perula-Martinez <raul.perula@uc3m.es>
 * @date        2015-03
 * @author      David Garcia <dggodoy@ing.uc3m.es>
 * @date        2012-02
 * @author      Victor Gonzalez <vgonzale@ing.uc3m.es>
 * @date        2009-01
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

#include <sys/ioctl.h>
#include <sys/time.h>
#include <ros/ros.h>
#ifdef APPLE
#include ETC_DIR "/drivers/termios/termios.h"
#else
#include <termios.h>
#endif

#include "../ipc/semaphores.h"
#include "../common_errors.h"

/// Serial Port Config
#define DEFAULT_BAUDRATE 19200
/// Serial Port reading timeout (in milliseconds)
#define SP_READ_TIMEOUT 2000
/// Serial Port reading timeout (in milliseconds)
#define SPLT_READ_TIMEOUT 10000
/// Serial Port Default Message Size
#define SP_MSG_SIZE 64
/// Serial Port Max Filename descriptor size in chars (Usually is /dev/something)
#define SP_MAX_FILENAME_SIZE 64

class Rs232 {
    public:
        /**
         * @brief Function that initializes the communication with the serial port.
         *
         *   Before that creates a semaphore to forbid simultaneous access to the serial port
         *
         * @param baudrate serial port communication speed
         * @param serialDevice serial device file name
         * @param semFile semaphore file to make the semKey
         *
         * @return -1 if could not configure serial port
         * @return -2 if could not open serial device
         * @return -3 if could not set zero motorSensor
         * @return -4 if there is any problem with semaphores
         */
        int initCommunication(int baudrate, char *serialDevice, char *semFile);

        /**
         * @brief Ends the communication closing the file descriptor and the removes the semaphore
         */
        int endCommunication();

        /* Read and Write from/to the Serial Port functions */

        /**
         * @brief writes (sends) a command to the serial port
         *
         * @param command The string where the command is stored
         * @param commandSize is the number of chars of the command string
         *
         * @return ERR_NOERR if everything is correct
         * @return ERR_SEM if there is an error waiting the semaphore
         * @return ERR_WRI if there is an error writing to the serial port file descriptor
         */
        int writeToRS232(char *command, int commandSize);

        /**
         * @brief reads frames finished with "CRLF" from the serial port. After reading it, the incoming message is stored in a string
         *
         * @param buff string where the output from the serial port is stored
         *
         * @return ERR_NOERR if there aren't any problems
         * @return ERR_SEM in case of a semaphore related error
         * @return ERR_READ if there is an error when reading from the serial port file descriptor
         * @return ERR_TIMEOUT in case of a reading timeout
         */
        int readFromRS232(char * buff);

        /**
         * @brief sends a question to the serial port and gets its response finished with "CRLF". The response is stored in a string
         *
         * This function sends a command or a question to the serial port and immediately reads the response produced by the device which is connected to the other
         * side of the serial port.
         * It is better to use askToRS232() function rather the combination of writeToRS232() and readFromRS232() functions. The reason of that is because using the other
         * two functions the semaphore is reserved and freed in each function. When you expect an immediate answer from the driver it is safer to
         * maintain the semaphore open until you have read this response.
         *
         * @param command is the string used to send the question/command to the serial port
         * @param commandSize is the size in bytes (chars) of the question/command sent to the serial port
         * @param response string where the output incoming from the serial port is stored
         *
         * @return ERR_NOERR if there aren't any problems
         * @return ERR_SEM in case of a semaphore related error
         * @return ERR_WRI if there is an error when writing in the serial port file descriptor
         * @return ERR_READ if there is an error when reading from the serial port
         * @return ERR_TIMEOUT if the time available for this operation (defined in SP_READ_TIMEOUT) expires
         */
        int askToRS232(char *command, int commandsize, char *response);

        // Functions added by David García to use with protocol not ended with "CRLF"

        /**
         * @brief reads form the serial port the number of characters specified by responseSize. After reading it, the incomming message is stored in a string
         *
         * @param response string where the output from the serial port is stored
         * @param responseSize bytes to be read
         *
         * @return ERR_NOERR if there aren't any problems
         * @return ERR_SEM in case of a semaphore related error
         * @return ERR_READ if there is an error when reading from the serial port file descriptor
         * @return ERR_TIMEOUT in case of a reading timeout
         * @return i number of characters read
         */
        int readSerial(char *response, int responseSize);

        /**
         * @brief sends a question to the serial port and gets its response. The response is stored in a string
         *
         * This function sends a command or a question to the serial port and immediately reads the response produced by the device wich is connected to the other
         * side of the serial port.
         * It is better to use askToRS232() function rather the combination of writeToRS232() and readFromRS232() functions. The reason of that is because using the other
         * two functions the semaphore is reserved and freed in each function. When you expect an immediate answer from the driver it is safer to
         * maintain the semaphore open until you have read this response.
         *
         * @param command is the string used to send the question/command to the serial port
         * @param commandSize is the size in bytes (chars) of the question/command sent to the serial port
         * @param response string where the output incoming from the serial port is stored
         * @param responseSize is the size in bytes (chars) of the expected response
         *
         * @return ERR_NOERR if there aren't any problems
         * @return ERR_SEM in case of a semaphore related error
         * @return ERR_WRI if there is an error when writing in the serial port file descriptor
         * @return ERR_READ if there is an error when reading from the serial port
         * @return ERR_TIMEOUT if the time available for this operation (defined in SP_READ_TIMEOUT) expires
         * @return i number of characters read
         */
        int askToSerial(char *command, int commandSize, char *response, int responseSize);

        /**
         * @brief Returns a time difference in milliseconds.
         * @param before is the reference time value to compare
         * @param after is the time wich we want to compare
         * @return the diference between before and after in milliseconds
         */
        long int timeDifferenceMsec(struct timeval *before, struct timeval *after);

    private:
        // RSd file descriptor of the serial port
        // semID semaphore identifier
        int _RSd, _semID;
};

#endif
