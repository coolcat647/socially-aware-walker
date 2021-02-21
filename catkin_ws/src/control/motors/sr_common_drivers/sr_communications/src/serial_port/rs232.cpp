/**
 * @file        rs232.cpp
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

#include "sr_communications/serial_port/rs232.h"

//////////////////////////////////////////////////

int Rs232::initCommunication(int baudrate, char *serialDevice, char *semFile)
{
    /// @TODO change this to use a rs232_t and a semaphore_t

    key_t semKey;
    struct termios newtio;

    int error; /// Used to catch errors from functions

    error = createKey(&semKey, semFile);
    if (error < 0) {
        ROS_ERROR("Error cretaing semaphore Key");
        return (-4);
    }
    error = getSemID(semKey, &_semID);
    if (error < 0) {
        ROS_ERROR("Error in getSemID. semKey = %x semFile = %s\n", semKey, semFile);
        return (-4);  // Semaphore related error
    }

    error = waitSem(_semID);
    if (error < 0) {
        ROS_ERROR("RS232.c - initCommunication: Error in waitSem\n\r");
        freeSem(_semID);
        rmSem(_semID);
        return (-4);  // Semaphore related error
    }

    // Do not use O_NOCTTY, not supported on kernels 2.6.31+
    //*RSd = open( serialDevice, O_RDWR | O_NOCTTY );
    _RSd = open(serialDevice, O_RDWR | O_NONBLOCK | O_NDELAY);
    if (_RSd < 0) {
        ROS_ERROR("Error opening serial device: %s", serialDevice);
        freeSem(_semID);
        rmSem(_semID);
        return (-2);
    }

    bzero((void*) &newtio, sizeof(newtio));
    switch(baudrate) {
        case 4800:
            newtio.c_cflag = B4800 | CS8 | CLOCAL | CREAD;
            break;
        case 9600:
            newtio.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
            break;
        case 19200:
            newtio.c_cflag = B19200 | CS8 | CLOCAL | CREAD;
            break;
        case 38400:
            newtio.c_cflag = B38400 | CS8 | CLOCAL | CREAD;
            break;
        case 57600:
            newtio.c_cflag = B57600 | CS8 | CLOCAL | CREAD;
            break;
        case 115200:
            newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
            break;
        default:
            newtio.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    }
    newtio.c_iflag = IGNPAR | ICRNL;
    newtio.c_oflag = 0;
    //newtio.c_lflag |= (ICANON | ECHO | ECHOE);		//modo canonico
    newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);	//modo raw
    newtio.c_cc[VINTR] = 0;
    newtio.c_cc[VQUIT] = 0;
    newtio.c_cc[VERASE] = 0;
    newtio.c_cc[VKILL] = 0;
    newtio.c_cc[VEOF] = 4;
    newtio.c_cc[VSWTC] = 0;
    newtio.c_cc[VSTART] = 0;
    newtio.c_cc[VSTOP] = 0;
    newtio.c_cc[VSUSP] = 0;
    newtio.c_cc[VEOL] = 0;
    newtio.c_cc[VREPRINT] = 0;
    newtio.c_cc[VDISCARD] = 0;
    newtio.c_cc[VWERASE] = 0;
    newtio.c_cc[VLNEXT] = 0;
    newtio.c_cc[VEOL2] = 0;
    newtio.c_cc[VTIME] = 0; /* inter-character timer unused */
    newtio.c_cc[VMIN] = 0;   //1  /* blocking read until 1 chars received */
    tcflush(_RSd, TCIFLUSH);
    tcflush(_RSd, TCOFLUSH);

    if (tcsetattr(_RSd, TCSANOW, &newtio)) {
        ROS_ERROR("Error configurating serial port dev = %s\n", serialDevice);
        freeSem(_semID);
        rmSem(_semID);
        return -1;
    }

    if (tcflush(_RSd, TCIOFLUSH) < 0) {
        ROS_ERROR("Error flushing serial port dev = %s\n", serialDevice);
        freeSem(_semID);
        rmSem(_semID);
        return -1;
    }

    freeSem(_semID);

    return ERR_NOERR;
}

//////////////////////////////////////////////////

int Rs232::endCommunication()
{
    waitSem(_semID);
    close(_RSd);
    freeSem(_semID);

    return rmSem(_semID);
}

//////////////////////////////////////////////////

int Rs232::writeToRS232(char* command, int commandSize)
{
    if (waitSem(_semID) < ERR_NOERR) {
        ROS_ERROR("Error when waiting the semaphore\n\r");
        freeSem(_semID);
        return ERR_SEM;
    }

    if (write(_RSd, command, commandSize) <= ERR_NOERR) {
        ROS_ERROR("Error when writing in the serial port file descriptor\n\r");
        freeSem(_semID);
        return ERR_WRI;
    }

    freeSem(_semID);
    return ERR_NOERR;
}

//////////////////////////////////////////////////

int Rs232::readFromRS232(char * buff)
{
    int i = 0;
    //int timeOut = SP_READ_TIMEOUT;
    int readResult;
    int status = 1;
    int bytes = 0;          /// Number of bytes available at the serial port

    struct timeval before, now;

    if (waitSem(_semID) < ERR_NOERR) {
        ROS_ERROR("Error when waiting the semaphore\n\r");
        freeSem(_semID);
        return ERR_SEM;
    }

    gettimeofday(&before, 0);
    do {                                // Begin reading from the serial port

        ioctl(_RSd, FIONREAD, &bytes);   // Check if there is some information
        if (bytes > 0) {                // There is information buffer ready to be readed
            readResult = read(_RSd, &buff[i], 1);
            if (readResult < 0) {
                ROS_ERROR("Reading Error");
                freeSem(_semID);
                return (ERR_READ);
            }
            else {
                i += readResult;        // Add to i the number of readed bytes
            }

            if (i > 1) {
                if ((buff[i - 2] == '\n') && (buff[i - 1] == '\n')) {
                    status = 0;         // We have readed data from the serial port
                }
            }
        }

        gettimeofday(&now, 0);
        if (SP_READ_TIMEOUT < timeDifferenceMsec(&before, &now)) {
            freeSem(_semID);
            return (ERR_TIMEOUT);
        }

        usleep(1);
    }
    while(status);

    freeSem(_semID);

    return (status);
}

//////////////////////////////////////////////////

int Rs232::askToRS232(char *command, int commandSize, char *response)
{
    int i = 0;
    //int timeOut = SP_READ_TIMEOUT;
    int readResult;
    int status = 1;
    int bytes = 0;          /// Number of bytes available at the serial port

    struct timeval before, now;

    if (waitSem(_semID) < ERR_NOERR) {
        ROS_ERROR("Error when waiting the semaphore\n\r");
        freeSem(_semID);
        return ERR_SEM;
    }

    // Sending the command to the serial port
    if (write(_RSd, command, commandSize) <= ERR_NOERR) {
        ROS_ERROR("Error when asking (writing) to the serial port file descriptor\n\r");
        freeSem(_semID);
        return ERR_WRI;
    }

    // Reading the response from the serial port
    gettimeofday(&before, 0);
    do {                             // Begin reading from the serial port

        ioctl(_RSd, FIONREAD, &bytes); // Check if there is some information
        if (bytes > 0) {        // There is information buffer ready to be readed
            readResult = read(_RSd, &response[i], 1);
            if (readResult < 0) {
                ROS_ERROR("Reading Error");
                freeSem(_semID);
                return (ERR_READ);
            }
            else {
                i += readResult;                // Add to i the number of readed bytes
            }

            if (i > 1) {
                if ((response[i - 2] == '\n') && (response[i - 1] == '\n')) {
                    status = ERR_NOERR;         // We have readed data from the serial port
                }
            }
        }

        gettimeofday(&now, 0);
        if (SP_READ_TIMEOUT < timeDifferenceMsec(&before, &now)) {
            freeSem(_semID);
            ROS_ERROR("Timeout Error. Error when asking to the serial port file descriptor\n\r");
            ROS_ERROR("status : %d\n", status);
            return (ERR_TIMEOUT);
        }
        usleep(100);
    }
    while(status);

    freeSem(_semID);

    return (status);
}

//////////////////////////////////////////////////

/// Functions to use with protocols not ended with special characters

int Rs232::readSerial(char *response, int responseSize)
{
    int i = 0;
    int readResult;
    int status = 1;
    int bytes = 0;          /// Number of bytes available at the serial port

    struct timeval before, now;

    gettimeofday(&before, 0);
    do {                                // Begin reading from the serial port

        ioctl(_RSd, FIONREAD, &bytes);   // Check if there is some information
        if (bytes > 0) {                // There is information buffer ready to be readed
            readResult = read(_RSd, &response[i], responseSize);

            if (readResult < 0) {
                ROS_ERROR("Reading Error: %d.", readResult);
                freeSem(_semID);
                return (ERR_READ);
            }
            else if (readResult == 0) {
                //ha llegado al final del fichero, pero no entraría porque comprobamos que haya datos con ioctl
                ROS_ERROR("Reading Error: EOF reached");
                freeSem(_semID);
                return (ERR_READ);
            }
            else {
                i += readResult;        // Add to i the number of readed bytes
            }

            //we have readed the desired data
            if (i == responseSize) {
                status = 0;         // We have readed data from the serial port
            }
        }

        gettimeofday(&now, 0);
        if (SPLT_READ_TIMEOUT < timeDifferenceMsec(&before, &now)) {
            freeSem(_semID);
            ROS_ERROR("Timeout ERROR while reading");
            return (ERR_TIMEOUT);
        }

        // do not delete this usleep
        usleep(500);

    }
    while(status);

    //devolvemos el número de caracteres leídos
    return (i);
}

//////////////////////////////////////////////////

int Rs232::askToSerial(char *command, int commandSize, char *response, int responseSize)
{
    int i = 0;
    int readResult;
    int status = 1;
    int bytes = 0;          /// Number of bytes available at the serial port

    struct timeval before, now;

    // Sending the command to the serial port
    if (write(_RSd, command, commandSize) <= ERR_NOERR) {
        ROS_ERROR("Error when asking (writing) to the serial port file descriptor\n\r");
        freeSem(_semID);
        return ERR_WRI;
    }

    // Reading the response from the serial port
    gettimeofday(&before, 0);
    do {                                // Begin reading from the serial port

        ioctl(_RSd, FIONREAD, &bytes);   // Check if there is some information
        if (bytes > 0) {                // There is information buffer ready to be readed
            readResult = read(_RSd, &response[i], responseSize);

            if (readResult < 0) {
                ROS_ERROR("Reading Error: %d.", readResult);
                freeSem(_semID);
                return (ERR_READ);
            }
            else if (readResult == 0) {
                //ha llegado al final del fichero, pero no entraría porque comprobamos que haya datos con ioctl
                ROS_ERROR("Reading Error: EOF reached");
                freeSem(_semID);
                return (ERR_READ);
            }
            else {
                i += readResult;        // Add to i the number of readed bytes
            }

            //we have readed the desired data
            if (i == responseSize) {
                status = 0;         // We have readed data from the serial port
            }
        }

        gettimeofday(&now, 0);
        if (SPLT_READ_TIMEOUT < timeDifferenceMsec(&before, &now)) {
            freeSem(_semID);
            ROS_ERROR("Timeout ERROR while reading (asking)");
            return (ERR_TIMEOUT);
        }

        // do not delete this usleep
        usleep(500);

    }
    while(status);

    return (i);
}

//////////////////////////////////////////////////

long int Rs232::timeDifferenceMsec(struct timeval *before, struct timeval *after)
{
    /** @TODO Error control here */
    return ((after->tv_sec - before->tv_sec) * 1000 + (after->tv_usec - before->tv_usec) / 1000);
}

//////////////////////////////////////////////////
