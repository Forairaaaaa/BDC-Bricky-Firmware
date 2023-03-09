/**
 * @file Motor.cpp
 * @author Forairaaaaa
 * @brief 
 * @version 0.1
 * @date 2023-03-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "Motor.h"

#define TagMotor "Motor"

namespace GearMotor
{
    
    void Motor::init(void)
    {
        if (_cfg.mcpwmA_gpio_num <= GPIO_NUM_NC || _cfg.mcpwmB_gpio_num <= GPIO_NUM_NC) {
            ESP_LOGE(TagMotor, "Invalid pin config");
            return;
        }

        /* Get duty max ticks */
        _mcpwm_duty_tick_max = _cfg.mcpwm_resolution_hz / _cfg.mcpwm_freq_hz;

        /* Init motor driver */
        bdc_motor_config_t motor_config = {
            .pwma_gpio_num = (uint32_t)_cfg.mcpwmA_gpio_num,
            .pwmb_gpio_num = (uint32_t)_cfg.mcpwmB_gpio_num,
            .pwm_freq_hz = _cfg.mcpwm_freq_hz,
        };
        bdc_motor_mcpwm_config_t motor_mcpwm_config = {
            .group_id = _cfg.mcpwm_group_id,
            .resolution_hz = _cfg.mcpwm_resolution_hz,
        };
        bdc_motor_new_mcpwm_device(&motor_config, &motor_mcpwm_config, &_motor_handler);

        /* Reset motor */
        enable();
        brake();
    }

    esp_err_t Motor::setSpeed(int32_t speed)
    {
        /* Set direction */
        if (speed > 0) {
            bdc_motor_forward(_motor_handler);
        }
        else if (speed < 0) {
            bdc_motor_reverse(_motor_handler);
        }
        else {
            return bdc_motor_brake(_motor_handler);
        }

        speed = abs(speed);

        /* If bigger */
        if (speed > _mcpwm_duty_tick_max)
            speed = _mcpwm_duty_tick_max;
            
        return bdc_motor_set_speed(_motor_handler, speed);
    }

}
