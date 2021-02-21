#ifndef __COMMON_ERRORS_H__
#define __COMMON_ERRORS_H__

/**
 * @file        common_error_definitions.h
 * @brief       Common errors for low-level drivers and devices.
 *
 * @author      Raul Perula-Martinez <raul.perula@uc3m.es>
 * @date        2015-03
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

/**
 * \def ERR_NOERR
 * \brief No error.
 */
#define ERR_NOERR 0

// Errors: Driver related errors.

/**
 * \def ERR_CURLIM
 * \brief Current limit reached.
 */
#define ERR_CURLIM -1

/**
 * \def ERR_TIMEOUT
 * \brief Timeout reached
 */
#undef  ERR_TIMEOUT // to avoid other collisions.
#define ERR_TIMEOUT -2

/**
 * \def ERR_NOHOME
 * \brief Error establishing the homing position (HO command to the driver).
 */
#define ERR_NOHOME -3

/**
 * \def ERR_OUTOFRANGE
 * \brief Error out of range. The entered parameter is not between the allowed limits.
 */
#define ERR_OUTOFRANGE -4

/**
 * \def ERR_POSLIMIT
 * \brief Error in the position limits of the driver.
 */
#define ERR_POSLIMIT -5

/**
 * \def ERR_CONF
 * \brief Error trying to configure the device (used in the mcdc3006s driver).
 */
#define ERR_CONF -6

// Errors: Serial Port related errors.

/**
 * \def ERR_READ
 * \brief Error reading from the serial port.
 */
#define ERR_READ -7

/**
 * \def ERR_WRI
 * \brief Error writing to the serial port.
 */
#define ERR_WRI -8

/**
 * \def ERR_COM
 * \brief Error communicating with the serial port.
 */
#define ERR_COM -9

/**
 * \def ERR_SEM
 * \brief Semaphore related error.
 */
#define ERR_SEM -10

/**
 * \def ERR_PARAM_NF
 * \brief Param Not found error.
 */
#define ERR_PARAM_NF    -11

/**
 * \def ERR_FILE_ACCESS
 * \brief File access error.
 */
#define ERR_FILE_ACCESS -12

/**
 * \def ERR_NA
 * \brief error Not Available.
 */
#define ERR_NA -13

/**
 * \def ERR_UNK
 * \brief unknown error.
 */
#define ERR_UNK -14

/**
 * \def ERR_NOTLINKED
 * \brief Error because the actuator is not linked to a driver.
 */
#define ERR_NOTLINKED -15

/**
 * \def ERR_OUTOFBOUNDS
 * \brief error because some parameter is not between the allowed limits.
 */
#define ERR_OUTOFBOUNDS -16

/**
 * \def ERR_CREAT
 * \brief error creating an object.
 */
#define ERR_CREAT -17

/**
 * \def ERR_GENERIC
 * \brief generic error.
 */
#define ERR_GENERIC -18

/**
 * \def ERR_NAK
 * \brief nack received.
 */
#define ERR_NAK -18

/**
 * \def ERR_UNCONFIG
 * \brief device not configured.
 */
#define ERR_UNCONFIG -19

#endif
