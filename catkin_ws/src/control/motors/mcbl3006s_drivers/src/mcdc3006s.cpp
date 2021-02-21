/**
 * @file        mcdc3006s.c
 * @brief       Software implementation of the Aplication Program Interface for the drive model faulhaberMCDC3006S.
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

#include "mcdc3006s_drivers/mcdc3006s.h"

//////////////////////////////////////////////////

Mcdc3006s::Mcdc3006s()
{
}

//////////////////////////////////////////////////

Mcdc3006s::~Mcdc3006s()
{
    _comm.endCommunication();
}

//////////////////////////////////////////////////

int Mcdc3006s::init(int baudrate, char *dev, char *sem)
{
    int error = 0;
    if ((error = _comm.initCommunication(baudrate, &dev[0], &sem[0])) < 0) {
        ROS_ERROR("[MCDC3006S] initCommunication with devices failed");
        ROS_WARN("[MCDC3006S] baudrate=%d, serialDevice=%s, semFile=%s", baudrate, dev, sem);

        return error;
    }

    return ERR_NOERR;
}

//////////////////////////////////////////////////

int Mcdc3006s::enable_driver()
{
    char command[] = "EN\n\r\0";

    if (_comm.writeToRS232(command, strlen(command)) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] EnableDriver() --> Error");

        return ERR_WRI;
    }

    return ERR_NOERR;
}

//////////////////////////////////////////////////

int Mcdc3006s::disable_driver()
{
    char command[] = "DI\n\r\0";

    if (_comm.writeToRS232(command, strlen(command)) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] DisableDriver() --> Error");

        return ERR_WRI;
    }

    return ERR_NOERR;
}

//////////////////////////////////////////////////

int Mcdc3006s::get_config(driverConf_t *dc)
{
    (*dc).maxPos = get_max_pos();
    if (dc->maxPos == ERR_COM) {
        ROS_ERROR("[MCDC3006S] getDriverConf() --> Error reading the configuration from the driver");
        return ERR_COM;
    }

    (*dc).minPos = get_min_pos();
    if (dc->minPos == ERR_COM) {
        ROS_ERROR("[MCDC3006S] getDriverConf() --> Error reading the configuration from the driver");
        return ERR_COM;
    }

    (*dc).maxVel = get_max_vel();
    if (dc->maxVel == ERR_COM) {
        ROS_ERROR("[MCDC3006S] getDriverConf() --> Error reading the configuration from the driver");
        return ERR_COM;
    }

    (*dc).maxAcc = get_max_acc();
    if (dc->maxAcc == ERR_COM) {
        ROS_ERROR("[MCDC3006S] getDriverConf() --> Error reading the configuration from the driver");
        return ERR_COM;
    }

    (*dc).maxDec = get_max_dec();
    if (dc->maxDec == ERR_COM) {
        ROS_ERROR("[MCDC3006S] getDriverConf() --> Error reading the configuration from the driver");
        return ERR_COM;
    }

    (*dc).cCLimit = get_cur_lim();
    if (dc->maxAcc == ERR_COM) {
        ROS_ERROR("[MCDC3006S] getDriverConf() --> Error reading the configuration from the driver");

        return ERR_COM;
    }

    return ERR_NOERR;
}

//////////////////////////////////////////////////

long int Mcdc3006s::get_max_pos()
{
    char command[SP_MSG_SIZE], response[SP_MSG_SIZE];

    sprintf(command, "GPL\r");

    if (_comm.askToRS232(command, strlen(command), response) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] getDriverMaxPos() --> Error communicating with driver");

        return ERR_COM;
    }

    // Converting the response of the driver to a long int.
    return atol(response);
}

//////////////////////////////////////////////////

long int Mcdc3006s::get_min_pos()
{
    static char command[SP_MSG_SIZE];
    char response[SP_MSG_SIZE];
    long int minPos;

    sprintf(command, "GNL\r");

    if (_comm.askToRS232(command, strlen(command), response) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] getDriverMinPos() --> Error when communicating with the driver");

        return ERR_COM;
    }

    minPos = atol(response);

    return minPos;
}

//////////////////////////////////////////////////

long int Mcdc3006s::get_max_vel()
{
    char command[SP_MSG_SIZE]; // "GSP\n\r\0";
    char response[SP_MSG_SIZE];

    sprintf(command, "GSP\n\r\0");

    if (_comm.askToRS232(command, strlen(command), response) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] getDriverMaxVel() --> Error communicating with driver");

        return ERR_COM;
    }

    // Converting the response of the driver to a long int.
    return (atol(response));
}

//////////////////////////////////////////////////

long int Mcdc3006s::get_max_acc()
{
    //  long int maxAcc = 0.0;
    char command[SP_MSG_SIZE];
    char response[SP_MSG_SIZE];

    strcpy(command, "GAC\n\r\0");
    if (_comm.askToRS232(command, strlen(command), response) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] getDriverMaxAcc() --> Error communicating with driver");

        return ERR_COM;
    }

    // Converting the response of the driver to a long int.
    return (atol(response));
}

//////////////////////////////////////////////////

long int Mcdc3006s::get_max_dec()
{
    char command[SP_MSG_SIZE];
    char response[SP_MSG_SIZE];

    strcpy(command, "GDEC\n\r\0");

    if (_comm.askToRS232(command, strlen(command), response) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] getDriverMaxDec() --> Error communicating with driver");

        return ERR_COM;
    }

    // Converting the response of the driver to a long int.
    return (atol(response));
}

//////////////////////////////////////////////////

int Mcdc3006s::get_cur_lim()
{
    static char command[SP_MSG_SIZE];
    char response[SP_MSG_SIZE];

    sprintf(command, "GCC\n\r\0");

    if (_comm.askToRS232(command, strlen(command), response) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] getDriverCurLim() --> Error communicating with the driver");

        return ERR_COM;
    }

    return (atoi(response));
}

//////////////////////////////////////////////////

int Mcdc3006s::get_peak_cur_lim()
{
    static char command[SP_MSG_SIZE];
    char response[SP_MSG_SIZE];

    sprintf(command, "GPC\n\r\0");

    if (_comm.askToRS232(command, strlen(command), response) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] getDriverPCurLim() --> Error communicating with the driver");

        return ERR_COM;
    }

    return (atoi(response));
}

//////////////////////////////////////////////////

int Mcdc3006s::get_status(driverStatus_t * drvStatus)
{
    char command[SP_MSG_SIZE];
    char response[SP_MSG_SIZE];

    sprintf(command, "GST\n\r\0");          //
    if (_comm.askToRS232(command, strlen(command), response) < ERR_NOERR) {
        ROS_ERROR(
            "[MCDC3006S] getDriverStatus() --> Error when communicating with the driver. Could not establish driver status (GST).");

        return (ERR_COM);
    }
    // Now response has the driver "status" information
    drvStatus->disabled = atoi(response) & ENABLED_MASK;

    sprintf(command, "OST\n\r\0");          //
    if (_comm.askToRS232(command, strlen(command), response) < ERR_NOERR) {
        ROS_ERROR(
            "[MCDC3006S] getDriverStatus() --> Error when communicating with the driver. Could not establish driver Operative Status (OST).");

        return (ERR_COM);
    } // Now response has the driver "fault status" information

    drvStatus->curLimiting = atoi(response) & CURRENT_LIMITING_MASK;
    drvStatus->overVoltage = atoi(response) & OVERVOLTAGE_MASK;
    drvStatus->overTemperature = atoi(response) & OVERTEMPERATURE_MASK;
    drvStatus->sensorReached = atoi(response) & DRIVER_INPUT_4_MASK;

    drvStatus->curLimiting = (drvStatus->curLimiting > 0) ?
        TRUE : FALSE;
    drvStatus->disabled = (drvStatus->disabled > 0) ?
        TRUE : FALSE;
    drvStatus->overTemperature = (drvStatus->overTemperature > 0) ?
        TRUE : FALSE;
    drvStatus->overVoltage = (drvStatus->overVoltage > 0) ?
        FALSE : FALSE;
    drvStatus->sensorReached = (drvStatus->sensorReached > 0) ?
        FALSE : FALSE;

    return ERR_NOERR;
}

//////////////////////////////////////////////////

int Mcdc3006s::get_sensor(driverSensor_t *sensor)
{
    char command[SP_MSG_SIZE];
    char response[SP_MSG_SIZE];

    int status = ERR_NOERR;

    bzero((void *) response, sizeof(response));

    // Get current Position
    sprintf(command, "POS\n\r\0");
    if (_comm.askToRS232(command, strlen(command), response) < ERR_NOERR) {
        ROS_ERROR(
            "[MCDC3006S] getDriverOdometry() --> Error when communicating with the driver. Could not establish current position");
        return (ERR_COM);
    }
    (*sensor).p = atol(response);

    // Get current velocity --> Current target velocity in rpm
    sprintf(command, "GV\n\r\0");
    if (_comm.askToRS232(command, strlen(command), response) < ERR_NOERR) {
        ROS_ERROR(
            "[MCDC3006S] getDriverOdometry() --> Error when communicating with the driver. Could not establish current velocity");
        return (ERR_COM);
    }
    (*sensor).v = atol(response);

    // Get current instant current in mA
    sprintf(command, "GRC\n\r\0");
    if (_comm.askToRS232(command, strlen(command), response) < ERR_NOERR) {
        ROS_ERROR(
            "[MCDC3006S] getDriverOdometry() --> Error when communicating with the driver. Could not establish current instant current");
        return (ERR_COM);
    }
    (*sensor).i = atol(response);

    return (ERR_NOERR);
}

//////////////////////////////////////////////////

int Mcdc3006s::get_instant_pos(long int *position)
{
    char command[SP_MSG_SIZE];
    char response[SP_MSG_SIZE];
    int status = ERR_NOERR;

    bzero((void *) response, sizeof(response));
    sprintf(command, "POS\n\r\0");          // Get current Position
    if (_comm.askToRS232(command, strlen(command), response) < ERR_NOERR) {
        ROS_ERROR(
            "[MCDC3006S] getDriverInstantPos() --> Error when communicating with the driver. Could not establish current position");

        return (ERR_COM);
    }
    *position = atol(response);

    return ERR_NOERR;
}

//////////////////////////////////////////////////

int Mcdc3006s::get_instant_vel(long int *velocity)
{
    char command[SP_MSG_SIZE];
    char response[SP_MSG_SIZE];
    int status = ERR_NOERR;

    bzero((void *) response, sizeof(response));
    sprintf(command, "GV\n\r\0");           // Get Velocity
    if (_comm.askToRS232(command, strlen(command), response) < ERR_NOERR) {
        ROS_ERROR(
            "[MCDC3006S] getDriverInstantVel() --> Error when communicating with the driver. Could not establish current instant current");

        return (ERR_COM);
    }
    *velocity = atol(response);

    return (ERR_NOERR);
}

//////////////////////////////////////////////////

int Mcdc3006s::get_instant_current(int *current)
{
    char command[SP_MSG_SIZE];
    char response[SP_MSG_SIZE];
    int status = ERR_NOERR;

    bzero((void *) response, sizeof(response));
    sprintf(command, "GRC\n\r\0");          // Get current instant current in mA
    if (_comm.askToRS232(command, strlen(command), response) < ERR_NOERR) {
        ROS_ERROR(
            "[MCDC3006S] getDriverInstantCurrent() --> Error when communicating with the driver. Could not establish current instant current");

        return (ERR_COM);
    }
    *current = atol(response);

    return (ERR_NOERR);
}

//////////////////////////////////////////////////

int Mcdc3006s::set_config(driverConf_t dc)
{
    if (set_min_pos(dc.minPos) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] setDriverConf() --> Error setting the minimum position");

        return ERR_CONF;
    }
    if (set_max_pos(dc.maxPos) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] setDriverConf() --> Error setting the minimum position");

        return ERR_CONF;
    }
    if (set_max_vel(dc.maxVel) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] setDriverConf() --> Error setting the minimum position");

        return ERR_CONF;
    }
    if (set_max_acc(dc.maxAcc) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] setDriverConf() --> Error setting the minimum position");

        return ERR_CONF;
    }
    if (set_max_dec(dc.maxDec) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] setDriverConf() --> Error setting the minimum position");

        return ERR_CONF;
    }

    return ERR_NOERR;
}

//////////////////////////////////////////////////
int Mcdc3006s::set_ac_dec(void)
{
    char command[SP_MSG_SIZE];
    sprintf(command, "AC%d\n\r\0", 10);
    if (_comm.writeToRS232(command, strlen(command)) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] setDriverMaxPos() --> Error writing to the driver");

        return ERR_WRI;
    }
    usleep(5000);
    sprintf(command, "DEC%d\n\r\0", 100);
    if (_comm.writeToRS232(command, strlen(command)) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] setDriverMaxPos() --> Error writing to the driver");

        return ERR_WRI;
    }

    return ERR_NOERR;
}

int Mcdc3006s::set_max_pos(long int maxPos)
{
    char command[SP_MSG_SIZE];

    if (maxPos <= 0) {
        fprintf(stderr, "setDriveMaxPos --> ERROR. Out of range: "
                "maxPos must be higher than 0 (You entered %ld).\n\r",
                maxPos);

        return ERR_OUTOFRANGE;
    }
    sprintf(command, "LL%ld\n\r\0", maxPos);

    if (_comm.writeToRS232(command, strlen(command)) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] setDriverMaxPos() --> Error writing to the driver");

        return ERR_WRI;
    }

    if (activate_limits(ACTIVATE) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] setDriverMaxPos() --> Error activating the driver");

        return ERR_POSLIMIT;
    }

    return ERR_NOERR;
}

//////////////////////////////////////////////////

int Mcdc3006s::set_min_pos(long int minPos)
{
    char command[SP_MSG_SIZE];

    if (minPos >= 0) {
        fprintf(stderr, "setDriveMinPos --> ERROR. minPos must be lower than 0 (You entered %ld).\n\r", minPos);

        return ERR_OUTOFRANGE;
    }

    sprintf(command, "LL%ld\n\r\0", minPos);

    if (_comm.writeToRS232(command, strlen(command)) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] setDriverMinPos() --> Error writing to the driver");

        return ERR_WRI;
    }

    if (activate_limits(ACTIVATE) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] setDriverMaxPos() --> Error activating the driver");

        return ERR_POSLIMIT;
    }

    return ERR_NOERR;
}

//////////////////////////////////////////////////

int Mcdc3006s::set_max_vel(long int maxVel)
{
    char command[SP_MSG_SIZE];

    sprintf(command, "SP%ld\n\r\0", maxVel);

    if (_comm.writeToRS232(command, strlen(command)) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] setDriverMaxVel() --> Error writing to the driver");

        return ERR_WRI;
    }

    return ERR_NOERR;
}

//////////////////////////////////////////////////

int Mcdc3006s::set_max_acc(long int maxAcc)
{
    char command[SP_MSG_SIZE];

    sprintf(command, "AC%ld\n\r\0", maxAcc);

    if (_comm.writeToRS232(command, strlen(command)) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] setDriverMaxAcc() --> Error writing to the driver");

        return ERR_WRI;
    }

    return ERR_NOERR;
}

//////////////////////////////////////////////////

int Mcdc3006s::set_max_dec(long int maxDec)
{
    char command[SP_MSG_SIZE];

    sprintf(command, "DEC%ld\n\r\0", maxDec);

    if (_comm.writeToRS232(command, strlen(command)) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] setDriverMaxDec() --> Error writing to the driver");

        return ERR_WRI;
    }

    return ERR_NOERR;
}

//////////////////////////////////////////////////

int Mcdc3006s::set_cur_lim(int cl)
{
    char command[SP_MSG_SIZE];

    sprintf(command, "LCC%d\n\r\0", cl);

    if (_comm.writeToRS232(command, strlen(command)) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] setDriverCurLim() --> Error writing to the driver");

        return ERR_WRI;
    }

    return ERR_NOERR;
}

//////////////////////////////////////////////////

int Mcdc3006s::set_peak_cur_lim(int pcl)
{
    char command[SP_MSG_SIZE];

    sprintf(command, "LPC%d\n\r\0", pcl);

    if (_comm.writeToRS232(command, strlen(command)) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] setDriverPCurLim() --> Error writing to the driver");

        return ERR_WRI;
    }

    return ERR_NOERR;
}

//////////////////////////////////////////////////

int Mcdc3006s::set_baudrate(int baud)
{
    int c = -1;
    int n;
    char command[SP_MSG_SIZE];
    for (n = 1; n < 10; n++) {
        if (baud == 600 * n) {
            c = 1;
            break;
        }
    }
    if (c) {
        sprintf(command, "BAUD%d\n\r", baud);
    }
    else {
        fprintf(stderr, "driverMCDC3006S - setDriverBaud ERROR %d not supported\n\r", baud);

        return ERR_OUTOFRANGE;
    }
    if (_comm.writeToRS232(command, strlen(command)) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] setDriverCurLim() --> Error writing to the driver\n\r");

        return ERR_WRI;
    }

    return ERR_NOERR;
}

//////////////////////////////////////////////////

int Mcdc3006s::move_abs_pos(long int pos)
{
    char command[SP_MSG_SIZE]; // = "LA";

    sprintf(command, "LA%ld\n\r\0", pos);
    if (_comm.writeToRS232(command, strlen(command)) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] moveDriverAbsPos() --> Error writing to the driver\n\r");

        return ERR_WRI;
    }

    sprintf(command, "M\n\r\0");
    if (_comm.writeToRS232(command, strlen(command)) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] moveDriverAbsPos() --> Error writing to the driver\n\r");

        return ERR_WRI;
    }

    return ERR_NOERR;
}

//////////////////////////////////////////////////

int Mcdc3006s::move_rel_pos(long int pos)
{
    char command[SP_MSG_SIZE]; // = "LR";

    sprintf(command, "LR%ld\n\r\0", pos);
    if (_comm.writeToRS232(command, strlen(command)) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] moveDriverRelPos() --> Error writing to the driver\n\r");

        return ERR_WRI;
    }

    sprintf(command, "M\n\r\0");
    if (_comm.writeToRS232(command, strlen(command)) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] moveDriverRelPos() --> Error writing to the driver\n\r");

        return ERR_WRI;
    }

    return ERR_NOERR;
}

//////////////////////////////////////////////////

int Mcdc3006s::move_vel(long int vel)
{
    char command[SP_MSG_SIZE];

    sprintf(command, "V%ld\n\r\0", vel);
    if (_comm.writeToRS232(command, strlen(command)) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] moveDriverVel() --> Error writing to the driver\n\r");

        return ERR_WRI;
    }

    return ERR_NOERR;
}

//////////////////////////////////////////////////

int Mcdc3006s::save_to_flash()
{
    char command[SP_MSG_SIZE], response[SP_MSG_SIZE];

    sprintf(command, "EEPSAV\n\r\0");
    if (_comm.askToRS232(command, strlen(command), response) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] saveToFlash() --> Error. cannot communicate with the driver");

        return ERR_COM;
    }
    fprintf(stderr, "driverMCDC30065S - saveToFlash : %s\n", response);

    return ERR_NOERR;
}

//////////////////////////////////////////////////

int Mcdc3006s::activate_limits(int action)
{
    char command[SP_MSG_SIZE];

    if (action == ACTIVATE || action == DEACTIVATE) {
        sprintf(command, "APL%d\n\r", action);

        if (_comm.writeToRS232(command, strlen(command)) < ERR_NOERR) {
            ROS_ERROR("[MCDC3006S] activateLimits() --> Error. cannot write to the driver");

            return ERR_WRI;
        }

        return ERR_NOERR;
    }
    ROS_ERROR("[MCDC3006S] activateLimits() --> Error Out of range");

    return ERR_OUTOFRANGE;
}

//////////////////////////////////////////////////

int Mcdc3006s::set_home_position(long int home)
{
    char command[SP_MSG_SIZE];
    sprintf(command, "HO%ld\n\r", home);

    if (_comm.writeToRS232(command, strlen(command))) {
        ROS_ERROR("[MCDC3006S] setDriverHomePosition() --> Error\n\r");

        return ERR_WRI;
    }

    return ERR_NOERR;
}

//////////////////////////////////////////////////

int Mcdc3006s::calibrate(int limit)
{
    // This is the input 4 of the driver. In this input is connected the sensor.
    char calibrationCommand[SP_MSG_SIZE];
    char calibrationResponse[SP_MSG_SIZE];

    int status = 1;
    struct timeval before, now;

    // Configuring the driver parameters
    // LCC: Load Continous Current LPC: Load peak current
    sprintf(calibrationCommand, "LCC%d\n\r", CALIBRATION_CURRENT_LIMIT);
    if (_comm.writeToRS232(calibrationCommand, strlen(calibrationCommand)) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] calibrateDriver() --> Error\n\r");

        return ERR_NOHOME;
    }

    sprintf(calibrationCommand, "HL8\n\r");
    if (_comm.writeToRS232(calibrationCommand, strlen(calibrationCommand)) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] calibrateDriver() --> Error\n\r");

        return ERR_NOHOME;
    }

    sprintf(calibrationCommand, "V%d\n\r", CALIBRATION_VELOCITY);
    if (_comm.writeToRS232(calibrationCommand, strlen(calibrationCommand)) < ERR_NOERR) {
        ROS_ERROR("[MCDC3006S] calibrateDriver() --> Error\n\r");

        return ERR_NOHOME;
    }
    gettimeofday(&before, 0);

    do {
        char str_tmp[SP_MSG_SIZE];
        char calibrationResponse[SP_MSG_SIZE];

        sprintf(str_tmp, "OST\n\r\0");
        _comm.askToRS232(str_tmp, strlen(str_tmp), calibrationResponse); /// @ToDo Error control here

        gettimeofday(&now, 0);
        if (atoi(calibrationResponse) & DRIVER_INPUT_4_MASK) {
            status = ERR_NOERR; // Sensor Reached OK.
        }
        else if (atoi(calibrationResponse) & CURRENT_LIMITING_MASK) {
            ROS_ERROR(
                "[MCDC3006S] calibrateDriver() --> Error Calibrating the driver. Current Limit Reached Could not establish home position");
            status = ERR_CURLIM; // Error calibrating the driver (limit current reached)

        }
        else if (now.tv_sec - before.tv_sec > CALIBRATION_TIMEOUT) {
            ROS_ERROR(
                "[MCDC3006S] calibrateDriver() --> Error Calibrating the driver. Timeout. Could not establish home position");
            status = ERR_TIMEOUT; // Timeout reached before arriving to the sensor
        }
    }
    while(status == 1);

    // Assuring that the driver stops at this point
    char str_tmp1[SP_MSG_SIZE];
    sprintf(str_tmp1, "V0\n\r\0");
    _comm.writeToRS232(str_tmp1, strlen(str_tmp1));

    // Moving the driver to 0.
    if (status == ERR_NOERR) {
        ROS_DEBUG("[MCDC3006S] calibrateDriver() --> Going to home position");

        // move driver to the requested position (in pulses)
        move_rel_pos(limit);

        sleep(3);

        //set home position
        if (set_home_position(0) == 0) {
            move_rel_pos(0);
        }
        else {
            ROS_ERROR(
                "[MCDC3006S] calibrateDriver() --> Error Calibrating the driver. Could not establish home position");
            status = ERR_NOHOME;
        }
    }

    return (status);
}

//////////////////////////////////////////////////
