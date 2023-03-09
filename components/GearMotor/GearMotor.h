/**
 * @file GearMotor.h
 * @author Forairaaaaa
 * @brief 
 * @version 0.1
 * @date 2023-03-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include "Motor/Motor.h"
#include "Encoder/Encoder.h"

namespace GearMotor
{
    struct GearMotorConfig_t {
        /* Motor ID */
        uint8_t id = 0;

        /* Motor configs for speed calculation */
        uint32_t pulse_count_per_round = 680;       // Pcnt counts with a round of motor
        double wheel_radius = 0.0325;               // Radius of wheel in meter
    };
    
    class GearMotor : private Motor, private Encoder {
        private:
            GearMotorConfig_t _cfg;

            /* Convertion */
            inline double _ticksToRPS(const int32_t& ticks, const double& dt) { return (double)ticks / ((double)_cfg.pulse_count_per_round * dt); }
            inline double _ticksToRPM(const int32_t& ticks, const double& dt) { return (double)ticks / ((double)_cfg.pulse_count_per_round * dt) * 60; }
            inline int32_t _RPSToTicks(const double& rpm, const double& dt) { return rpm * ((double)_cfg.pulse_count_per_round * dt); }
            inline int32_t _RPMToTicks(const double& rpm, const double& dt) { return (rpm / 60) * ((double)_cfg.pulse_count_per_round * dt); }

        public:
            inline GearMotor() {}
            inline ~GearMotor() {}

            inline GearMotorConfig_t config(void) { return _cfg; }
            inline void config(const GearMotorConfig_t& cfg) { _cfg = cfg; }

            inline void ID(const uint8_t& id) { _cfg.id = id; }
            inline uint8_t ID(void) { return _cfg.id; }

            /**
             * @brief Init with configs 
             * 
             * @param mcpwmA_gpio_num 
             * @param mcpwmB_gpio_num 
             * @param mcpwm_group_id 
             * @param encoderA_gpio_num 
             * @param encoderB_gpio_num 
             * @param id 
             */
            inline void init(const int& mcpwmA_gpio_num, const int& mcpwmB_gpio_num, const int& mcpwm_group_id,
                        const int& encoderA_gpio_num, const int& encoderB_gpio_num, const uint8_t& id)
            {
                ID(id);
                init(mcpwmA_gpio_num, mcpwmB_gpio_num, mcpwm_group_id, encoderA_gpio_num, encoderB_gpio_num);
            }
            void init(const int& mcpwmA_gpio_num, const int& mcpwmB_gpio_num, const int& mcpwm_group_id,
                        const int& encoderA_gpio_num, const int& encoderB_gpio_num);

            /**
             * @brief Get motor current speed
             * 
             * @param dt Interval between two calls
             * @return double 
             */
            inline double getMotorSpeed(const double& dt = 0.01) { return _ticksToRPM(Encoder::readCountClear(), dt); }
    };

}
