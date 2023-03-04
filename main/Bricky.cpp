/**
 * @file Bricky.c
 * @author Forairaaaaa
 * @brief 
 * @version 0.1
 * @date 2023-02-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <bdc_motor.h>
#include <MotorCtrl.h>


/**
 * @brief Pin map
 * 
 */
#define MOTOR_M1_IN1    14
#define MOTOR_M1_IN2    13
#define MOTOR_M1_A      41
#define MOTOR_M1_B      42
#define MOTOR_M1_CS     4

#define MOTOR_M2_IN1    46
#define MOTOR_M2_IN2    15
#define MOTOR_M2_A      39
#define MOTOR_M2_B      40
#define MOTOR_M2_CS     3

#define MOTOR_M3_IN1    10
#define MOTOR_M3_IN2    9
#define MOTOR_M3_A      12
#define MOTOR_M3_B      11
#define MOTOR_M3_CS     1

#define MOTOR_M4_IN1    5
#define MOTOR_M4_IN2    6
#define MOTOR_M4_A      8
#define MOTOR_M4_B      7
#define MOTOR_M4_CS     2



#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks




#include "driver/uart.h"
#include "string.h"

void serial_test()
{
    const uart_port_t uart_num = UART_NUM_0;
    uart_config_t uart_config = {
        .baud_rate = 256000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 43, 44, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Setup UART in rs485 half duplex mode
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));


    char* data = (char*) malloc(1024);
    int shit = 0;

    while (1)
    {



        //Read data from UART
        // int len = uart_read_bytes(uart_num, data, 1024, 1000);

        // if (len > 0) {
        //     for (int i = 0; i < len; i++) {
        //         uart_write_bytes(uart_num, &data[i], 1);
        //     }
        // }

        snprintf(data, 1024, "123456\n");
        uart_write_bytes(uart_num, data, 8);

    }

}


extern "C" void app_main(void)
{
    serial_test();


    bdc_motor_config_t motor1_config = {
        .pwma_gpio_num = MOTOR_M1_IN1,
        .pwmb_gpio_num = MOTOR_M1_IN2,
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
    };
    bdc_motor_mcpwm_config_t motor1_mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor1_handler = NULL;
    bdc_motor_new_mcpwm_device(&motor1_config, &motor1_mcpwm_config, &motor1_handler);
    bdc_motor_enable(motor1_handler);


    bdc_motor_config_t motor2_config = {
        .pwma_gpio_num = MOTOR_M2_IN1,
        .pwmb_gpio_num = MOTOR_M2_IN2,
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
    };
    bdc_motor_mcpwm_config_t motor2_mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor2_handler = NULL;
    bdc_motor_new_mcpwm_device(&motor2_config, &motor2_mcpwm_config, &motor2_handler);
    bdc_motor_enable(motor2_handler);


    bdc_motor_config_t motor3_config = {
        .pwma_gpio_num = MOTOR_M3_IN1,
        .pwmb_gpio_num = MOTOR_M3_IN2,
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
    };
    bdc_motor_mcpwm_config_t motor3_mcpwm_config = {
        .group_id = 1,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor3_handler = NULL;
    bdc_motor_new_mcpwm_device(&motor3_config, &motor3_mcpwm_config, &motor3_handler);
    bdc_motor_enable(motor3_handler);


    bdc_motor_config_t motor4_config = {
        .pwma_gpio_num = MOTOR_M4_IN1,
        .pwmb_gpio_num = MOTOR_M4_IN2,
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
    };
    bdc_motor_mcpwm_config_t motor4_mcpwm_config = {
        .group_id = 1,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor4_handler = NULL;
    bdc_motor_new_mcpwm_device(&motor4_config, &motor4_mcpwm_config, &motor4_handler);
    bdc_motor_enable(motor4_handler);






    // bdc_motor_forward(motor1_handler);
    // bdc_motor_forward(motor2_handler);
    // bdc_motor_forward(motor3_handler);
    // bdc_motor_forward(motor4_handler);


    

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    while (1)
    {
        
        bdc_motor_forward(motor1_handler);
        bdc_motor_reverse(motor2_handler);
        bdc_motor_reverse(motor3_handler);
        bdc_motor_forward(motor4_handler);

        printf("set max speed forward\n");
        bdc_motor_set_speed(motor1_handler, 400);
        bdc_motor_set_speed(motor2_handler, 400);
        bdc_motor_set_speed(motor3_handler, 400);
        bdc_motor_set_speed(motor4_handler, 400);
        vTaskDelay(3000 / portTICK_PERIOD_MS);


        printf("set half speed forward\n");
        bdc_motor_set_speed(motor1_handler, 300);
        bdc_motor_set_speed(motor2_handler, 300);
        bdc_motor_set_speed(motor3_handler, 300);
        bdc_motor_set_speed(motor4_handler, 300);
        vTaskDelay(3000 / portTICK_PERIOD_MS);


        bdc_motor_brake(motor1_handler);
        vTaskDelay(100 / portTICK_PERIOD_MS);







        bdc_motor_reverse(motor1_handler);
        bdc_motor_forward(motor2_handler);
        bdc_motor_forward(motor3_handler);
        bdc_motor_reverse(motor4_handler);

        printf("set max speed backward\n");
        bdc_motor_set_speed(motor1_handler, 400);
        bdc_motor_set_speed(motor2_handler, 400);
        bdc_motor_set_speed(motor3_handler, 400);
        bdc_motor_set_speed(motor4_handler, 400);
        vTaskDelay(3000 / portTICK_PERIOD_MS);

        printf("set half speed backward\n");
        bdc_motor_set_speed(motor1_handler, 300);
        bdc_motor_set_speed(motor2_handler, 300);
        bdc_motor_set_speed(motor3_handler, 300);
        bdc_motor_set_speed(motor4_handler, 300);
        vTaskDelay(3000 / portTICK_PERIOD_MS);

        bdc_motor_brake(motor1_handler);
        vTaskDelay(100 / portTICK_PERIOD_MS);




        




    }




}
