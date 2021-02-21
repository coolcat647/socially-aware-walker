#ifndef __MOTOR_DRIVER_INTERFACE_H__
#define __MOTOR_DRIVER_INTERFACE_H__

/**
 * @file        motor_driver_interface.h
 * @brief       Interface for the motor drivers.
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

#include "mcdc3006s_data.h"

class MotorDriverInterface {
    public:
        /**
         * @brief Constructor.
         */
        MotorDriverInterface()
        {
        }

        /**
         * @brief Abstract destructor.
         */
        virtual ~MotorDriverInterface()
        {
        }

        /**
         * @brief high level functions
         */
        virtual int init(int baudrate, char *dev, char *sem) = 0;

        /**
         * @brief Enables the driver
         * @return ERR_NOERR if everything works correctly
         * @return ERR_WRI in case of error when writing in the driver descriptor file
         */
        virtual int enable_driver() = 0;

        /**
         * @brief Disables the driver
         * @return ERR_NOERR if everything works correctly
         * @return ERR_WRI in case of error when writing in the driver descriptor file
         */
        virtual int disable_driver() = 0;

        /**
         * @brief Function that gets the current driver configuration as is stored in the
         *        flash memory of the driver and writes it in a driverConf_t type format
         *
         * @param dc is the driverConf_t type where the driver configuration is stored
         *
         * @return ERR_NOERR if there are no problems
         * @return ERR_COM if it is not possible to read (any parameter) information from
         *         the driver. When this happens at least one of the parameters of the driverConf_t
         *         structure could be incomplete.
         */
        virtual int get_config(driverConf_t *dc) = 0;

        /**
         * @brief returns the maximum position allowed by the driver. To set this value see
         * @see SetDriverMaxPos
         *
         * @return ERR_NOERR if there aren't any problems
         * @return ERR_COM if could not communicate with the driver (either write or read)
         * \todo change the way I save the parameter. It has to be saved by parameter by reference.
         * Return only must return Error control
         *
         */
        virtual long int get_max_pos() = 0;

        /**
         * @brief This function reads from the driver min position from its flash memory
         *
         * @return returns the value of the min position in pulses
         * @return ERR_COM if could not communicate with the driver (either write or read)
         * <b>Warning</b>, it is possible to recevie a min pos which the same as ERR_COM. If this happens the function will wrongly understand that an error has occured
         * * \todo change the way I save the parameter. It has to be saved by parameter by reference. Return only must return Error control
         */
        virtual long int get_min_pos() = 0;

        /**
         * @brief Returns the maximum speed allowed by the driver which is set with setDriverMaxVel()
         *        function
         *
         * @return maxVel configurated in the drive in r.p.m.
         * @return ERR_COM if could not communicate with the driver (either write or read)
         * \todo change the way I save the parameter. It has to be saved by parameter by reference.
         *  Return only must return Error control
         */
        virtual long int get_max_vel() = 0;

        /**
         * @brief returns the maximum acceleration allowed by the driver. To set this value see @see SetDriverMaxAcc
         *
         * @return maxAcc Returns the maximum acceleration revolutions/sec²
         * @return ERR_COM if could not communicate with the driver (either write or read)
         * \todo change the way I save the parameter. It has to be saved by parameter by reference.
         * Return only must return Error control
         */
        virtual long int get_max_acc() = 0;

        /**
         * @brief returns the maximum decceleration allowed by the driver. To set this value see
         * @see SetDriverMaxDec
         *
         * @return maxDec Returns the maximum decceleration revolutions/sec²
         * @return ERR_COM if could not communicate with the driver (either write or read)
         * \todo change the way I save the parameter. It has to be saved by parameter by reference.
         * Return only must return Error control
         *
         */
        virtual long int get_max_dec() = 0;

        /**
         * @brief get the continous current limit (CCL) in mA.
         *
         * @return returns the value of the continous current limit (CCL) in mA.
         * @return ERR_COM if could not communicate with the driver (either write or read)
         *
         * WARNING: Untested Function @TODO
         * \todo change the way I save the parameter. It has to be saved by parameter by reference.
         * Return only must return Error control
         */
        virtual int get_cur_lim() = 0;

        /**
         * @brief get the Peak Current Limit (PCL) in mA.
         *
         * @return returns the value of the peak current limit (PCL) in mA.
         * @return ERR_COM if could not communicate with the driver (either write or read)
         *
         * WARNING: Untested Function @TODO
         * \todo change the way I save the parameter. It has to be saved by parameter by reference.
         * Return only must return Error control
         */
        virtual int get_peak_cur_lim() = 0;

        /**
         * @brief Asks to the driver its current status
         * @param drvStatus is a structure where the current driver status is stored. Each of the parameters of the drvStatus are set to 1 if true or 0 if false
         * @return ERR_NOERR if the status could be returned
         * @return ERR_COM if it is not possible to check the driver status
         */
        virtual int get_status(driverStatus_t * drvStatus) = 0;

        /**
         * @brief asks for sensor data of the driver
         * @param sensor is the sensor structure where we want to store the sensor data from the driver.
         * @return ERR_NOERR if no error (operation is succesful)
         * @return ERR_COM if there is an error communicating whit the driver. In this case some (of all) of the sensor parameters could not be up to date.
         */
        virtual int get_sensor(driverSensor_t *sensor) = 0;

        /**
         * @brief asks for the instant position from sensor data of the driver.
         * @param position is a pointer pointing to where the position sensor data from the driver will be stored. The units from the driver are in [pulses]
         * @return ERR_NOERR if no error (operation is succesful)
         * @return ERR_COM if there is an error communicating whit the driver.
         */
        virtual int get_instant_pos(long int *positon) = 0;

        /**
         * @brief asks for the instant velocity from sensor data of the driver.
         * @param velocity is where the velocity sensor data from the driver will be stored. The units from the driver are [rpm]
         * @return ERR_NOERR if no error (operation is succesful)
         * @return ERR_COM if there is an error communicating whit the driver.
         */
        virtual int get_instant_vel(long int *velocity) = 0;

        /**
         * @brief asks for the instant velocity from sensor data of the driver.
         * @param current is where the instant current data from the driver will be stored.
         *        The units from the driver are [rpm]
         * @return ERR_NOERR if no error (operation is succesful)
         * @return ERR_COM if there is an error communicating whit the driver.
         */
        virtual int get_instant_current(int *current) = 0;

        /**
         * @brief Function that configures the driver from a driverConf_t type
         * @param dc is where dc configuration is read in order to write it into the driver
         * @return ERR_NOERR if success
         * @return ERR_CONF if any error occurs
         */
        virtual int set_config(driverConf_t dc) = 0;

        /**
         * @brief Sets the maximum absolute position in pulses
         * @param maxPos in pulses
         * @return ERR_NOERR if everything has gone correctly
         * @return ERR_OUTOFRANGE if maxPos is below or equal 0 (maxPos must be higher than 0).
         * @return ERR_WRI if could not write to the driver
         * @return ERR_POSLIMIT if it is not possible to activate the limits
         */
        virtual int set_max_pos(long int maxPos) = 0;

        /**
         * @brief Sets the minimum absolute position in pulses
         * @param minPos is the minimum absolute position in pulses
         * @return ERR_NOERR if everything has gone correctly
         * @return ERR_OUTOFRANGE if minPos is higher or equal than 0 (minPos must be lower than 0).
         * @return ERR_WRI if could not write to the driver
         * @return ERR_POSLIMIT if it is not possible to activate the limits
         */
        virtual int set_min_pos(long int minPos) = 0;

        /**
         * @brief Sets the maximum speed allowed by the driver in rpm
         * @param maxVel in r.p.m.
         * @return ERR_NOERR if everything goes correct
         * @return ERR_WRI if could not write to the driver
         */
        virtual int set_max_vel(long int maxVel) = 0;

        /**
         * @brief sets the maximum acceleration allowed by the driver in revolutions/s^2
         * @param maxAcc maximum acceleration in revolutions/s^2
         * @return ERR_NOERR if everything goes correct
         * @return ERR_WRI if could not write to the driver
         */
        virtual int set_max_acc(long int maxAcc) = 0;

        /**
         * @brief sets the maximum deceleration allowed by the driver in revolutions/s^2
         * @param maxDec maximum deceleration in r/sec²
         * @return ERR_NOERR if everything goes correct
         * @return ERR_WRI if could not write to the driver
         *
         */
        virtual int set_max_dec(long int maxDec) = 0;

        /**
         * @brief loads continous current limit
         * @param cl current limit in mA Range = [0 - 12000mA]
         * @return ERR_NOERR if everything goes correct
         * @return ERR_WRI if could not write to the driver
         */
        virtual int set_cur_lim(int cl) = 0;

        /**
         * @brief loads peak current limit
         * @param pcl current limit in mA Range = [0 - 12000mA]
         * @return ERR_NOERR if everything goes correct
         * @return ERR_WRI if could not write to the driver
         */
        virtual int set_peak_cur_lim(int pcl) = 0;

        /**
         * @brief tells to the driver at which baudrate we want to operate
         * @param baud is the baudrate of the communication.
         * @return ERR_NOERR if the configuration is succesful
         * @return ERR_OUTOFRANGE if the parameter baud is not in the expected values
         * @return ERR_WRI if it is not possible to write to the driver the configuration
         */
        virtual int set_baudrate(int baud) = 0;

        /**
         * @brief Sends to the driver the order to move to an absolute position in pulses
         * @param p absolute position in pulses
         * @return ERR_NOERR if no error sending the command to the driver
         * @return ERR_WRI if it isn't possible to communicate with the driver
         */
        virtual int move_abs_pos(long int pos) = 0;

        /**
         * @brief Sends to the driver the order to move to a relative position in pulses
         * @param p relative position in pulses
         * @return ERR_NOERR if no error sending the command to the driver
         * @return ERR_WRI if it isn't possible to communicate with the driver
         */
        virtual int move_rel_pos(long int pos) = 0;

        /**
         * @brief Sends to the driver the order to move at a determined speed (in rpm)
         * @param v velocity in rpm (revolutions per minute)
         * @return ERR_NOERR if no error sending the command to the driver
         * @return ERR_WRI if it isn't possible to communicate with the driver
         */
        virtual int move_vel(long int vel) = 0;

        /**
         * @brief Save current configuartion params to FLASH memory of the drive, so the nest time
         *        the drive is turned on it loads thas configuration.
         * WARNING!!! It should not be used more than 10,000 times because the FLASH memory could
         *            get damaged!!!
         *
         * @return ERR_COM if an error communicating with driver occurs
         * @return ERR_NOERR if the save is done correctly
         *
         * The EEPSAV command always responds with the character string “EEPROM writing done”
         * after successful saving of the current settings in the data Flash memory,
         * or with “Flash defect”, if the save has failed.
         */
        virtual int save_to_flash() = 0;

        /**
         * @brief Activates/Deactivates the driver Limits. If the limits aren't activated the driver
         *        will NOT respect the Maximum or Minimum positions.
         * @param action it only has two allowed values: ACTIVATE to activate de driver or DEACTIVATE
         *        to disable de driver
         *
         * @return ERR_NOERR if everything works correctly
         * @return ERR_WRI in case of error when writing in the driver descriptor file
         * @return ERR_OUTOFRANGE if action is not in allowed values set
         */
        virtual int activate_limits(int action) = 0;

        /**
         * @brief Sets the current positon as the given value. NOTE: Use only during the calibration phase.
         *        This function sends to the driver the Home command. The home commands tells to the driver which
         *        is the current position. For example If we send home = 0
         *        we are telling to the driver that the current position is 0 position (and it should used as a
         *        reference).
         * @param home is the given position in pulses. This parameter specifies which is the current position.
         * @return ERR_NOERR if everything works correctly
         * @return ERR_WRI in case of error when writing in the driver descriptor file
         */
        virtual int set_home_position(long int home) = 0;

        /**
         * @brief starts the calibration sequence of the drive
         * @param limit sensor position in pulses measured from the desired 0 position
         * @param currentLimit max current the motor is not allowed to exceed (in mA) This limit is
         *        directly related to torque of the motor.
         * @param calibrationSpeed The speed of the motor during the calibration
         * @param timeOut maximum time (in milli seconds) to perform the calibration. If this time is
         *        exceeded the function will return an error.
         * @return ERR_NOERR If the calibration is done successfully
         * @return ERR_CURLIM if the current limit is reached during the calibration. I.E. an obstacle
         *         is blocking the motor.
         * @return ERR_TIMEOUT if the calibration process lasts more than the timeOut value
         * @return ERR_NOHOME if it is not possible to establish the home position
         */
        virtual int calibrate(int limit) = 0;
};

#endif
