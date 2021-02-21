/**
 * @file        mcdc3006s_mock.h
 * @brief       Mock for mcdc3006s driver.
 *
 * @author      Raul Perula-Martinez <raul.perula@uc3m.es>
 * @date        2015-04
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

#include <gmock/gmock.h>
#include "motor_driver_interface.h"

class MockMcdc3006s : public MotorDriverInterface {
    public:
        MOCK_METHOD0(enable_driver, int());
        MOCK_METHOD0(disable_driver, int());
        MOCK_METHOD0(get_max_pos, long int());
        MOCK_METHOD0(get_min_pos, long int());
        MOCK_METHOD0(get_max_vel, long int());
        MOCK_METHOD0(get_max_acc, long int());
        MOCK_METHOD0(get_max_dec, long int());
        MOCK_METHOD0(get_cur_lim, int());
        MOCK_METHOD0(get_peak_cur_lim, int());
        MOCK_METHOD0(save_to_flash, int());

        MOCK_METHOD1(get_config, int(driverConf_t *dc));
        MOCK_METHOD1(get_status, int(driverStatus_t * drvStatus));
        MOCK_METHOD1(get_sensor, int(driverSensor_t *sensor));
        MOCK_METHOD1(get_instant_pos, int(long int *positon));
        MOCK_METHOD1(get_instant_vel, int(long int *velocity));
        MOCK_METHOD1(get_instant_current, int(int *current));
        MOCK_METHOD1(set_config, int(driverConf_t dc));
        MOCK_METHOD1(set_max_pos, int(long int maxPos));
        MOCK_METHOD1(set_min_pos, int(long int minPos));
        MOCK_METHOD1(set_max_vel, int(long int maxVel));
        MOCK_METHOD1(set_max_acc, int(long int maxAcc));
        MOCK_METHOD1(set_max_dec, int(long int maxDec));
        MOCK_METHOD1(set_cur_lim, int(int cl));
        MOCK_METHOD1(set_peak_cur_lim, int(int pcl));
        MOCK_METHOD1(set_baudrate, int(int baud));
        MOCK_METHOD1(move_abs_pos, int(long int pos));
        MOCK_METHOD1(move_rel_pos, int(long int pos));
        MOCK_METHOD1(move_vel, int(long int vel));
        MOCK_METHOD1(activate_limits, int(int action));
        MOCK_METHOD1(set_home_position, int(long int home));
        MOCK_METHOD1(calibrate, int(int limit));

        MOCK_METHOD3(init, int(int baudrate, char *dev, char *sem));
};
