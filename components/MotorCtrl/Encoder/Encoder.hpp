/**
 * @file Encoder.hpp
 * @author Forairaaaaa
 * @brief 
 * @version 0.1
 * @date 2023-03-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include <driver/pulse_cnt.h>


namespace MotorCtrl
{
    struct EncoderConfig_t {
        /* Pin configs */
        int encoderA_gpio_num = GPIO_NUM_NC;
        int encoderB_gpio_num = GPIO_NUM_NC;

        /* Pcnt unit configs */
        int pcnt_high_limit = 1000;
        int pcnt_low_limit = -1000;
    }; 

    const char* tagEncdoer = "Encoder";

    class Encoder {
        private:
            EncoderConfig_t _cfg;
            pcnt_unit_handle_t _pcnt_unit_handler;

        public:
            Encoder() { _pcnt_unit_handler = NULL; }
            ~Encoder() { pcnt_del_unit(_pcnt_unit_handler); }

            EncoderConfig_t config(void) { return _cfg; }
            void config(const EncoderConfig_t& cfg) { _cfg = cfg; }

            void setPins(const int& encoderA_gpio_num, const int& encoderB_gpio_num)
            {
                _cfg.encoderA_gpio_num = encoderA_gpio_num;
                _cfg.encoderB_gpio_num = encoderB_gpio_num;
            }

            void init(const int& encoderA_gpio_num, const int& encoderB_gpio_num)
            {
                setPins(encoderA_gpio_num, encoderB_gpio_num);
                init();
            }
            void init(void)
            {
                if (_cfg.encoderA_gpio_num <= GPIO_NUM_NC || _cfg.encoderB_gpio_num <= GPIO_NUM_NC) {
                    ESP_LOGE(tagEncdoer, "Invalid pin config");
                    return;
                }

                /* Pcnt setups */
                pcnt_unit_config_t pcnt_unit_config;
                pcnt_unit_config.high_limit = _cfg.pcnt_high_limit;
                pcnt_unit_config.low_limit = _cfg.pcnt_low_limit;
                pcnt_unit_config.flags.accum_count = true;          // enable counter accumulation
                ESP_ERROR_CHECK(pcnt_new_unit(&pcnt_unit_config, &_pcnt_unit_handler));

                /* Filter setups */
                pcnt_glitch_filter_config_t pcnt_filter_config;
                pcnt_filter_config.max_glitch_ns = 1000;
                ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(_pcnt_unit_handler, &pcnt_filter_config));

                /* Pin setups */
                pcnt_chan_config_t pcnt_chan_a_config = {
                    .edge_gpio_num = _cfg.encoderA_gpio_num,
                    .level_gpio_num = _cfg.encoderB_gpio_num,
                };
                pcnt_channel_handle_t pcnt_chan_a_handler = NULL;
                ESP_ERROR_CHECK(pcnt_new_channel(_pcnt_unit_handler, &pcnt_chan_a_config, &pcnt_chan_a_handler));
                pcnt_chan_config_t pcnt_chan_b_config = {
                    .edge_gpio_num = _cfg.encoderB_gpio_num,
                    .level_gpio_num = _cfg.encoderA_gpio_num,
                };
                pcnt_channel_handle_t pcnt_chan_b_handler = NULL;
                ESP_ERROR_CHECK(pcnt_new_channel(_pcnt_unit_handler, &pcnt_chan_b_config, &pcnt_chan_b_handler));

                ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a_handler, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
                ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a_handler, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
                ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b_handler, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
                ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b_handler, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
                ESP_ERROR_CHECK(pcnt_unit_add_watch_point(_pcnt_unit_handler, _cfg.pcnt_high_limit));
                ESP_ERROR_CHECK(pcnt_unit_add_watch_point(_pcnt_unit_handler, _cfg.pcnt_low_limit));
                ESP_ERROR_CHECK(pcnt_unit_enable(_pcnt_unit_handler));
                ESP_ERROR_CHECK(pcnt_unit_clear_count(_pcnt_unit_handler));
                ESP_ERROR_CHECK(pcnt_unit_start(_pcnt_unit_handler));
            }
            
            /* Wrap */
            inline esp_err_t enable() { return pcnt_unit_enable(_pcnt_unit_handler); }
            inline esp_err_t disable() { return pcnt_unit_disable(_pcnt_unit_handler); }
            inline esp_err_t start() { return pcnt_unit_start(_pcnt_unit_handler); }
            inline esp_err_t stop() { return pcnt_unit_stop(_pcnt_unit_handler); }
            inline esp_err_t clear() { return pcnt_unit_clear_count(_pcnt_unit_handler); }
            int readCount() { int ret = 0; pcnt_unit_get_count(_pcnt_unit_handler, &ret); return ret; }
            int raedCountClear() { int ret = 0; pcnt_unit_get_count(_pcnt_unit_handler, &ret); clear(); return ret; }
    };

}
