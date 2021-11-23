/**
 *@file swirlE.c
 *@example
 *
 *
 *Demonstrates usage of ultrasonic sensors, all 3 servos, autonomous driving
 *
 *@author     Jacob Piirsalu
 *@date       11/11/2021
 */
#include <stdio.h>
#include <getopt.h>
#include <stdlib.h>    // for atoi
#include <signal.h>
#include <rc/time.h>
#include <rc/adc.h>
#include <rc/dsm.h>
#include <rc/servo.h>
#include <stdio.h>
#include <robotcontrol.h>    // includes ALL Robot Control subsystems
#include <rc/time.h>
#include <rc/gpio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdbool.h>
#include "swirlelib.h" //all robot functions written here

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

int main() {
    double servo_pos = 0;
    double sweep_limit = 1.5;
    int ch = 0;    // channel to test, 0 means all channels
    double direction = 1;    // switches between 1 &-1 in sweep mode
    int frequency_hz = 50;    // default 50hz frequency to send pulses
    robot_gpio_init();
    // initialize PRU
    if (rc_servo_init()) return -1;
    // turn on power - Turning On 6V Servo Power Rail
    rc_servo_power_rail_en(1);
    double leftC_sense = 0.0;
    double rightC_sense = 0.0;
    double corr_factor = 0.0;
    double max_speed = 0.08*1.0; //base speed of swirlE
    // - works decently at 0.08*1.25 and window size 3
    //safe speed is 0.08

    double gain = 10;
    double r_wheel_gain = 1.35; //1.35 when full battery 1.5 when under 50%
    int avg_val_ctr = 0;

    double l_red_arr[WINDOW];
    double l_green_arr[WINDOW];
    double l_blue_arr[WINDOW];

    double r_red_arr[WINDOW];
    double r_green_arr[WINDOW];
    double r_blue_arr[WINDOW];

    double l_r_sum = 0, l_g_sum = 0, l_b_sum = 0, r_r_sum = 0, r_g_sum = 0, r_b_sum = 0;
    double l_r_avg = 0, l_g_avg = 0, l_b_avg = 0, r_r_avg = 0, r_g_avg = 0, r_b_avg = 0;


    double corr_arr[WINDOW];
    double corr_factor_avg = 0.0;
    double sum = 0;

    printf("RGB test\n");
    //robot_forward(1,frequency_hz);

    double pulseR = 0.0;
    double pulseL = 0.0;

    //setting states for operation
    bool state[4] = {1, 0, 0, 0}; //initial state
    //state is arranged as: forward/back, capturing, dropping, returning
    //robot_move_cup_up(frequency_hz); //cup starts down
    //rc_usleep(SLEEP_TIME);
    int loopctr = 0;
    FILE *fp;

    fp = fopen("./output.csv", "w+");

    //MAIN CODE:
    while(1) {
        loopctr++;
        if (!state[1] && !state[2]) { //line following state X00X
            rc_usleep(1000);
            double l_red_val = colour_sensor_red(CS_OUT1);
            rc_usleep(1000);
            double r_red_val = colour_sensor_red(CS_OUT2);
            rc_usleep(1000);
            l_r_avg = rolling_avg(l_red_arr, &l_red_val, &l_r_sum);
            rc_usleep(1000);
            r_r_avg = rolling_avg(r_red_arr, &r_red_val, &r_r_sum);

            rc_usleep(1000);
            double l_green_val = colour_sensor_green(CS_OUT1);
            rc_usleep(1000);
            double r_green_val = colour_sensor_green(CS_OUT2);
            rc_usleep(1000);
            l_g_avg = rolling_avg(l_green_arr, &l_green_val, &l_g_sum);
            rc_usleep(1000);
            r_g_avg = rolling_avg(r_green_arr, &r_green_val, &r_g_sum);
            rc_usleep(1000);

            double l_blue_val = colour_sensor_blue(CS_OUT1);
            rc_usleep(1000);
            double r_blue_val = colour_sensor_blue(CS_OUT2);
            rc_usleep(1000);
            l_b_avg = rolling_avg(l_blue_arr, &l_blue_val, &l_b_sum);
            rc_usleep(1000);
            r_b_avg = rolling_avg(r_blue_arr, &r_blue_val, &r_b_sum);
            rc_usleep(1000);

            leftC_sense = l_r_avg + l_g_avg + l_b_avg; //left

            rightC_sense = r_r_avg + r_g_avg + r_b_avg; //right

            corr_factor = gain * (rightC_sense - leftC_sense) / (leftC_sense + rightC_sense);
            corr_factor_avg = rolling_avg(corr_arr, &corr_factor, &sum);
            servo_pos += direction * sweep_limit / frequency_hz;

            if (servo_pos > sweep_limit) {
                servo_pos = sweep_limit;
            }
            fprintf(fp, "%d,%f,%f,%f,%f,%f,%f\n",loopctr,l_r_avg,l_g_avg,l_b_avg,r_r_avg,r_g_avg,r_b_avg);
            printf("%d,%f,%f,%f,%f,%f,%f\n",loopctr,l_r_avg,l_g_avg,l_b_avg,r_r_avg,r_g_avg,r_b_avg);

        }
    }

    // turn off power rail and cleanup
    rc_servo_power_rail_en(0);
    rc_servo_cleanup();
    rc_dsm_cleanup();
    return 0;
}
