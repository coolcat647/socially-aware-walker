#ifndef __MCD3006S_DATA_H__
#define __MCD3006S_DATA_H__

/**
 * @file        mcdc3006s_data.h
 * @brief       Data for motors.
 *
 * @author      Raul Perula-Martinez <raul.perula@uc3m.es>
 * @date        2015-03
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
 * \todo This two defines must be moved to a primitive level
 */
//#define MCDC3006S_PULSES_PER_REV 2048      // Warning
//#define MCDC3006S_REDUCTION_FACTOR 123000  // motorValue = FACTOR * dofValue
#define CALIBRATION_TIMEOUT 10              // Calibration timeout in seconds
#define CALIBRATION_VELOCITY 500            // Calibration velocity in rpm
#define CALIBRATION_CURRENT_LIMIT 1500      // Calibration limit current in mA

#define ACTIVATE 1
#define DEACTIVATE 0

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

// Driver Masks used to extract information from GST and GFS (Get Status and Get Fault Status) commands
#define ENABLED_MASK            8           /**< Mask used to check if the driver is enabled or not. Masks the bit 3 of
                                                  the GST command response */
#define CURRENT_LIMITING_MASK   16          /**< Mask used to check if the current limiting is active. Masks the bit 4
                                                  of the OST command response */
#define OVERVOLTAGE_MASK        64          /**< Mask used to check if the overvoltage bit is active. Masks the bit 6
                                                  of the OST command response */
#define OVERTEMPERATURE_MASK    128         /**< Mask used to check if the overtemperature is active. Masks the bit 7
                                                  of the OST command response*/
#define DRIVER_INPUT_4_MASK     2048        /**< Mask used to check if the input 4 of the driver is active. I have this
                                                  input connected to the limit sensor. Masks the bit 11 of the OST
                                                  command response */
/**
 * @brief Structure where the configuration of the driver is stored
 */
typedef struct driverConf {
        /**< Driver maximum position (in pulses) */
        long int maxPos;
        /**< Driver minimum position (in pulses) */
        long int minPos;
        /**< Driver maximum velocity (in r.p.m) */
        long int maxVel;
        /**< Driver maximum acceleration (in revolutions / sec^2) */
        long int maxAcc;
        /**< Driver maximum deceleration (in revolutions / sec^2) */
        long int maxDec;
        /**< Driver Continuous Current Limit (LLC) in mA. */
        long int cCLimit;
} driverConf_t;

/**
 * @brief Structure that contains the information of the sensors of the driver (Position, Velocity and Current)
 */
typedef struct driverMotorSensor {
        /**< current position in pulses */
        long int p;
        /**< current velocity in rpm */
        long int v;
        /**< instant current in mA */
        long int i;
} driverSensor_t;

/**
 * @brief Structure where the status of the driver is stored
 */
typedef struct driverStatus {
        /**< Stores if the driver is disabled or not TRUE = enabled; FALSE = disabled */
        int disabled;
        /**< Stores if there is an overTemperature error or not TRUE = error overtemperature; FALSE = Temp is OK */
        int overTemperature;
        /**< Stores if limit current is active or not TRUE = current limit is active;
         FALSE = actualCurrent < currentLimt */
        int curLimiting;
        /**< Stores the status of overVoltage: TRUE = error overvoltage;
         FALSE = no error, voltage is lower than the minimum */
        int overVoltage;
        /**< Stores if the limit sensor is reached or not. I've connected this sensor to the
         input 4 of the driver. */
        int sensorReached;
} driverStatus_t;

#endif
