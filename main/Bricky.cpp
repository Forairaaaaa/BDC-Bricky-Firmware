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
#include <MotorCtrl.hpp>


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



#define delay(ms) vTaskDelay(pdMS_TO_TICKS(ms))






MotorCtrl::MotorCtrl MC1;
MotorCtrl::MotorCtrl MC2;
MotorCtrl::MotorCtrl MC3;
MotorCtrl::MotorCtrl MC4;


static double shit = 0;

extern "C" void app_main(void)
{
    printf("asdasdadasdasd :) \n");


    MC1.initPID(MOTOR_M1_IN1, MOTOR_M1_IN2, 0, MOTOR_M1_A, MOTOR_M1_B);
    MC2.initPID(MOTOR_M2_IN1, MOTOR_M2_IN2, 0, MOTOR_M2_A, MOTOR_M2_B);
    MC3.initPID(MOTOR_M3_IN1, MOTOR_M3_IN2, 1, MOTOR_M3_A, MOTOR_M3_B);
    MC4.initPID(MOTOR_M4_IN1, MOTOR_M4_IN2, 1, MOTOR_M4_A, MOTOR_M4_B);


    

    while (1)
    {
        // printf("%d %ld\n", MC1.raedCountClear(), MC1.getMaxSpeedTicks());
        // printf("S:%f,%f,%f,%f\n", MC1.getSpeedRPM(0.01), MC2.getSpeedRPM(0.01), MC3.getSpeedRPM(0.01), MC4.getSpeedRPM(0.01));
        // delay(10);

        shit += 0.1;

        MC1.setPIDConfig(shit, shit, shit);
        MC2.setPIDConfig(shit, shit, shit);
        MC3.setPIDConfig(shit, shit, shit);
        MC4.setPIDConfig(shit, shit, shit);
        delay(1000);
    }

}
