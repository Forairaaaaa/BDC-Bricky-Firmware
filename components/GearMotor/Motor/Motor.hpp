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

    const char* tagMotor = "Motor";
    
    class Motor {
        private:
            MotorConfig_t _cfg;
            uint32_t _mcpwm_duty_tick_max;
            bdc_motor_handle_t _motor_handler;

        public:
            Motor() { _motor_handler = NULL; }
            ~Motor() { bdc_motor_del(_motor_handler); }

            MotorConfig_t config(void) { return _cfg; }
            void config(const MotorConfig_t& cfg) { _cfg = cfg; }

            void setPins(const int& mcpwmA_gpio_num, const int& mcpwmB_gpio_num)
            {
                _cfg.mcpwmA_gpio_num = mcpwmA_gpio_num;
                _cfg.mcpwmB_gpio_num = mcpwmB_gpio_num;
            }
            void setMcpwmGroupId(const int& mcpwm_group_id) { _cfg.mcpwm_group_id = mcpwm_group_id; }
            
            /* Init with configs */
            void init(const int& mcpwmA_gpio_num, const int& mcpwmB_gpio_num, const int& mcpwm_group_id)
            {
                setPins(mcpwmA_gpio_num, mcpwmB_gpio_num);
                setMcpwmGroupId(mcpwm_group_id);
                init();
            }
            void init(void)
            {
                if (_cfg.mcpwmA_gpio_num <= GPIO_NUM_NC || _cfg.mcpwmB_gpio_num <= GPIO_NUM_NC) {
                    ESP_LOGE(tagMotor, "Invalid pin config");
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

            /* Motor control wrap */
            inline esp_err_t enable() { return bdc_motor_enable(_motor_handler); }
            inline esp_err_t disable() { return bdc_motor_disable(_motor_handler); }
            inline esp_err_t setSpeed(int32_t speed)
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
            inline uint32_t getMaxSpeedTicks() { return _mcpwm_duty_tick_max; }
            inline esp_err_t coast() { return bdc_motor_coast(_motor_handler); }
            inline esp_err_t brake() { return bdc_motor_brake(_motor_handler); }

    };

}
