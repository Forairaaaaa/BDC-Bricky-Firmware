/**
 * @file Motor.h
 * @author Forairaaaaa
 * @brief 
 * @version 0.1
 * @date 2023-03-04
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include <bdc_motor.h>
#include <driver/gpio.h>


namespace MotorCtrl
{
    struct MotorConfig_t {
        /* Motor ID */
        uint8_t id = 0;

        /* Pin configs */
        gpio_num_t mcpwmA_gpio_num = GPIO_NUM_NC;
        gpio_num_t mcpwmB_gpio_num = GPIO_NUM_NC;

        /* MCPWM configs */
        int mcpwm_group_id = 0;
        uint32_t mcpwm_freq_hz = 25000;       // 25KHz PWM
        uint32_t mcpwm_resolution_hz = 10000000;  // 10MHz, 1 tick = 0.1us
    };
    

    class Motor {
        private:
            uint32_t _mcpwm_duty_tick_max;
        public:
            Motor();
            ~Motor();
    };









}
