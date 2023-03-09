/**
 * @file GearMotor.cpp
 * @author Forairaaaaa
 * @brief 
 * @version 0.1
 * @date 2023-03-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "GearMotor.h"

namespace GearMotor
{

    void GearMotor::init(const int& mcpwmA_gpio_num, const int& mcpwmB_gpio_num, const int& mcpwm_group_id,
                        const int& encoderA_gpio_num, const int& encoderB_gpio_num)
    {
        /* Init modules */
        Motor::init(mcpwmA_gpio_num, mcpwmB_gpio_num, mcpwm_group_id);
        Encoder::init(encoderA_gpio_num, encoderB_gpio_num);
    }

}
