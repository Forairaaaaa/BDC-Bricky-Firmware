/**
 * @file MotorCtrl.h
 * @author Forairaaaaa
 * @brief 
 * @version 0.1
 * @date 2023-03-04
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "Motor/Motor.hpp"
#include "Encoder/Encoder.hpp"


namespace MotorCtrl
{
    struct MotorCtrlConfig_t {
        /* Motor ID */
        uint8_t id = 0;

        uint32_t pulse_count_per_round = 680;   // Pcnt counts with a round of motor
        double wheel_radius = 0.0325;           // Radius of wheel in meter

        uint8_t task_priority = 5;
    };

    struct PIDConfig_t {
        double current_speed = 0;               // Current speed in RPM
        double expect_speed = 0;                // Expect speed in RPM
        double P = 0;
        double I = 0;
        double D = 0;
    };

    const char* tagMotorCtrl = "MotorCtrl";

    void task_motor_ctrl(void* param);
    
    class MotorCtrl : public Motor, public Encoder {
        private:
            MotorCtrlConfig_t _cfg;
            PIDConfig_t _pid_cfg;

            /* Task related */
            TaskHandle_t _task_handler;
            SemaphoreHandle_t _semaphore_mutex;

            /* Convertion */
            inline double _ticksToRPS(const uint32_t& ticks, const double& dt) { return (double)ticks / ((double)_cfg.pulse_count_per_round * dt); }
            inline double _ticksToRPM(const uint32_t& ticks, const double& dt) { return (double)ticks / ((double)_cfg.pulse_count_per_round * dt) * 60; }


        public:
            MotorCtrl()
            {
                _task_handler = NULL;
                _semaphore_mutex = xSemaphoreCreateMutex();
            }
            ~MotorCtrl() {}

            MotorCtrlConfig_t config(void) { return _cfg; }
            void config(const MotorCtrlConfig_t& cfg) { _cfg = cfg; }

            void init(const int& mcpwmA_gpio_num, const int& mcpwmB_gpio_num, const int& mcpwm_group_id,
                        const int& encoderA_gpio_num, const int& encoderB_gpio_num)
            {
                /* Init modules */
                Motor::init(mcpwmA_gpio_num, mcpwmB_gpio_num, mcpwm_group_id);
                Encoder::init(encoderA_gpio_num, encoderB_gpio_num);
            }
            
            /**
             * @brief Init motor with PID control
             * 
             * @param mcpwmA_gpio_num 
             * @param mcpwmB_gpio_num 
             * @param mcpwm_group_id 
             * @param encoderA_gpio_num 
             * @param encoderB_gpio_num 
             */
            inline void initPID(const int& mcpwmA_gpio_num, const int& mcpwmB_gpio_num, const int& mcpwm_group_id,
                        const int& encoderA_gpio_num, const int& encoderB_gpio_num)
            {
                init(mcpwmA_gpio_num, mcpwmB_gpio_num, mcpwm_group_id, encoderA_gpio_num, encoderB_gpio_num);
                initPID();
            }
            void initPID()
            {
                /* Init task */
                char task_name_buffer[10];
                snprintf(task_name_buffer, 10, "Mctrl%d", _cfg.id);
                xTaskCreate(task_motor_ctrl, task_name_buffer, 4096, this, _cfg.task_priority, &_task_handler);
            }

            PIDConfig_t getPIDConfig()
            {
                xSemaphoreTake(_semaphore_mutex, portMAX_DELAY);
                PIDConfig_t ret = _pid_cfg;
                xSemaphoreGive(_semaphore_mutex);
                return ret;
            }
            void setPIDConfig(PIDConfig_t PIDConfig)
            {
                xSemaphoreTake(_semaphore_mutex, portMAX_DELAY);
                _pid_cfg = PIDConfig;
                xSemaphoreGive(_semaphore_mutex);
            }
            void setPIDConfig(double P, double I, double D)
            {
                xSemaphoreTake(_semaphore_mutex, portMAX_DELAY);
                _pid_cfg.P = P;
                _pid_cfg.I = I;
                _pid_cfg.D = D;
                xSemaphoreGive(_semaphore_mutex);
            }
            void setSpeed(double rpm)
            {
                xSemaphoreTake(_semaphore_mutex, portMAX_DELAY);
                _pid_cfg.expect_speed = rpm;
                xSemaphoreGive(_semaphore_mutex);
            }
            double getCurrentSpeed()
            {
                xSemaphoreTake(_semaphore_mutex, portMAX_DELAY);
                double ret = _pid_cfg.current_speed;
                xSemaphoreGive(_semaphore_mutex);
                return ret;
            }
            
            
    };

    /* Task motor control */
    void task_motor_ctrl(void* param)
    {
        MotorCtrl* motor = (MotorCtrl*)param;
        MotorCtrlConfig_t motor_cfg = motor->config();
        
        PIDConfig_t pid;

        while (1)
        {
            /* Motor config update */



            // motor->setSpeed(300);
            // vTaskDelay(pdMS_TO_TICKS(3000));
            // motor->setSpeed(-300);
            // vTaskDelay(pdMS_TO_TICKS(3000));

            pid = motor->getPIDConfig();
            
            printf("%f %f %f\n", pid.P, pid.I, pid.D);
            vTaskDelay(pdMS_TO_TICKS(10));
            
        }

        vTaskDelete(NULL);
    }

}
