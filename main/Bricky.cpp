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



#define delay(ms) vTaskDelay(pdMS_TO_TICKS(ms))




MotorCtrl::Motor M1;
MotorCtrl::Motor M2;
MotorCtrl::Motor M3;
MotorCtrl::Motor M4;

MotorCtrl::Encoder E1;
MotorCtrl::Encoder E2;
MotorCtrl::Encoder E3;
MotorCtrl::Encoder E4;




extern "C" void app_main(void)
{
    printf("asdasdadasdasd :) \n");


    M1.init(MOTOR_M1_IN1, MOTOR_M1_IN2, 0);
    M2.init(MOTOR_M2_IN1, MOTOR_M2_IN2, 0);
    M3.init(MOTOR_M3_IN1, MOTOR_M3_IN2, 1);
    M4.init(MOTOR_M4_IN1, MOTOR_M4_IN2, 1);

    M1.setSpeed(300);
    M2.setSpeed(300);
    M3.setSpeed(300);
    M4.setSpeed(300);



    // while (1)
    // {
    //     M1.setSpeed(400);
    //     M2.setSpeed(400);
    //     M3.setSpeed(400);
    //     M4.setSpeed(400);
    //     delay(2000);
    //     M1.setSpeed(300);
    //     M2.setSpeed(300);
    //     M3.setSpeed(300);
    //     M4.setSpeed(300);
    //     delay(2000);
    //     M1.brake();
    //     M2.brake();
    //     M3.brake();
    //     M4.brake();
    //     delay(1000);
    //     M1.setSpeed(-300);
    //     M2.setSpeed(-300);
    //     M3.setSpeed(-300);
    //     M4.setSpeed(-300);
    //     delay(2000);
        // M1.setSpeed(-400);
        // M2.setSpeed(-400);
        // M3.setSpeed(-400);
        // M4.setSpeed(-400);
    //     delay(2000);


    // }


    E1.init(MOTOR_M1_A, MOTOR_M1_B);
    E2.init(MOTOR_M2_A, MOTOR_M2_B);
    E3.init(MOTOR_M3_A, MOTOR_M3_B);
    E4.init(MOTOR_M4_A, MOTOR_M4_B);

    while (1)
    {
        printf("E:%d, %d, %d, %d\n", E1.raedCountClear(), E2.raedCountClear(), E3.raedCountClear(), E4.raedCountClear());
        delay(10);
    }

}
