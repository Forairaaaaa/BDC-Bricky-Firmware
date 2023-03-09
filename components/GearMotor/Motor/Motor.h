/**
 * @file Motor.h
 * @author Forairaaaaa
 * @brief 
 * @version 0.1
 * @date 2023-03-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include <bdc_motor.h>
#include <driver/gpio.h>
#include <esp_log.h>

namespace GearMotor
{
    
    struct MotorConfig_t {
        /* Pin configs */
        int mcpwmA_gpio_num = GPIO_NUM_NC;
        int mcpwmB_gpio_num = GPIO_NUM_NC;

        /* MCPWM configs */
        int mcpwm_group_id = 0;
        uint32_t mcpwm_freq_hz = 25000;             // 25KHz PWM
        uint32_t mcpwm_resolution_hz = 10000000;    // 10MHz, 1 tick = 0.1us
    };
    
    class Motor {
        private:
            MotorConfig_t _cfg;
            uint32_t _mcpwm_duty_tick_max;
            bdc_motor_handle_t _motor_handler;

        public:
            inline Motor() { _motor_handler = NULL; }
            inline ~Motor() { bdc_motor_del(_motor_handler); }

            inline MotorConfig_t config(void) { return _cfg; }
            inline void config(const MotorConfig_t& cfg) { _cfg = cfg; }

            inline uint32_t getMaxSpeedTicks() { return _mcpwm_duty_tick_max; }

            inline void setPins(const int& mcpwmA_gpio_num, const int& mcpwmB_gpio_num)
            {
                _cfg.mcpwmA_gpio_num = mcpwmA_gpio_num;
                _cfg.mcpwmB_gpio_num = mcpwmB_gpio_num;
            }
            
            inline void setMcpwmGroupId(const int& mcpwm_group_id) { _cfg.mcpwm_group_id = mcpwm_group_id; }
            
            /**
             * @brief Init motor with configs
             * 
             * @param mcpwmA_gpio_num 
             * @param mcpwmB_gpio_num 
             * @param mcpwm_group_id 
             */
            inline void init(const int& mcpwmA_gpio_num, const int& mcpwmB_gpio_num, const int& mcpwm_group_id)
            {
                setPins(mcpwmA_gpio_num, mcpwmB_gpio_num);
                setMcpwmGroupId(mcpwm_group_id);
                init();
            }

            void init(void);

            /* Motor control wrap */
            inline esp_err_t enable() { return bdc_motor_enable(_motor_handler); }
            inline esp_err_t disable() { return bdc_motor_disable(_motor_handler); }
            inline esp_err_t coast() { return bdc_motor_coast(_motor_handler); }
            inline esp_err_t brake() { return bdc_motor_brake(_motor_handler); }

            /**
             * @brief Set speed 0~getMaxSpeedTicks(), negative for reverse
             * 
             * @param speed 
             * @return esp_err_t 
             */
            esp_err_t setSpeed(int32_t speed);

    };

}
