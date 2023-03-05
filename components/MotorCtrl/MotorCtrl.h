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
#include "Motor/Motor.hpp"
#include "Encoder/Encoder.hpp"


namespace MotorCtrl
{
    struct MotorCtrlConfig_t {
        /* Motor ID */
        uint8_t id = 0;

    };

    const char* tagMotorCtrl = "MotorCtrl";
    
    class MotorCtrl : public Motor, public Encoder {
        private:
            MotorCtrlConfig_t _cfg;

        public:
            MotorCtrl() {}
            ~MotorCtrl() {}

            MotorCtrlConfig_t config(void) { return _cfg; }
            void config(const MotorCtrlConfig_t& cfg) { _cfg = cfg; }

            void init(const int& mcpwmA_gpio_num, const int& mcpwmB_gpio_num, const int& mcpwm_group_id,
                        const int& encoderA_gpio_num, const int& encoderB_gpio_num)
            {
                Motor::init(mcpwmA_gpio_num, mcpwmB_gpio_num, mcpwm_group_id);
                Encoder::init(encoderA_gpio_num, encoderB_gpio_num);
            }


    };

}
