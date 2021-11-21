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
    double max_speed = 0.08*1.1; //base speed of swirlE
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

    printf("starting line following\n");

    while (running) {


        leftC_sense = colour_sensor_red(CS_OUT1) + colour_sensor_green(CS_OUT1) + colour_sensor_blue(CS_OUT1); //left
        rightC_sense = colour_sensor_red(CS_OUT2) + colour_sensor_green(CS_OUT2) + colour_sensor_blue(CS_OUT2); //right

        corr_factor = gain * (rightC_sense - leftC_sense) / (leftC_sense + rightC_sense);

        corr_factor_avg = rolling_avg(corr_arr, &corr_factor, &sum);
//        sum = sum - corr_arr[avg_val_ctr];
//        corr_arr[avg_val_ctr] = corr_factor;
//        sum = sum + corr_factor;
//        avg_val_ctr = (avg_val_ctr + 1) % WINDOW; //window size
//        corr_factor_avg = sum / WINDOW;

        printf("%f\n", corr_factor_avg);


        double pulseL = 0.0;
        double pulseR = 0.0;
        servo_pos += direction * sweep_limit / frequency_hz;

        if (servo_pos > sweep_limit) {
            servo_pos = sweep_limit;
        }
        if (corr_factor_avg - 1.10/4 > 0) {

            //ch = 7; right servo, -1 pulse
            pulseR = r_wheel_gain * (servo_pos * max_speed * 6);
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = 0 * servo_pos * max_speed;
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        } else if (corr_factor_avg + 1.10/4 < 0) {
            //ch = 7; right servo
            pulseR = r_wheel_gain * 0 * (servo_pos * max_speed);
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = (servo_pos * max_speed * 6);
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        } else if (corr_factor_avg - 1.0/4 > 0) {
            //ch = 7; right servo
            pulseR = r_wheel_gain * (servo_pos * max_speed * 6);
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = servo_pos * max_speed * 0.25;
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        } else if (corr_factor_avg + 1.0/4 < 0) {
            //ch = 7; right servo
            pulseR = r_wheel_gain * (servo_pos * max_speed)*0.25;
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = servo_pos * max_speed * 6;
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        } else if (corr_factor_avg - 0.9/4 > 0) {
            //ch = 7; right servo
            pulseR = r_wheel_gain * (servo_pos * max_speed * 6);
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = servo_pos * max_speed * 0.5;
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        } else if (corr_factor_avg + 0.9/4 < 0) {
            //ch = 7; right servo
            pulseR = r_wheel_gain * (servo_pos * max_speed)*0.5;
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = servo_pos * max_speed * 6;
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        } else if (corr_factor_avg - 0.8/4 > 0) {
            //ch = 7; right servo
            pulseR = r_wheel_gain * (servo_pos * max_speed * 5);
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = servo_pos * max_speed * 0.75;
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        } else if (corr_factor_avg + 0.8/4 < 0) {
            //ch = 7; right servo
            pulseR = r_wheel_gain * (servo_pos * max_speed) * 0.75;
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = servo_pos * max_speed * 5;
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        } else if (corr_factor_avg - 0.7/4 > 0) {
            //ch = 7; right servo
            pulseR = r_wheel_gain * (servo_pos * max_speed * 4);
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = servo_pos * max_speed;
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        } else if (corr_factor_avg + 0.7/4 < 0) {
            //ch = 7; right servo
            pulseR = r_wheel_gain * (servo_pos * max_speed);
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = servo_pos * max_speed * 4;
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        }
//        else if (corr_factor_avg - 0.60/4 > 0) {
//            //ch = 7; right servo
//            pulseR = r_wheel_gain * (servo_pos * max_speed * 3);
//            pulseR = pulseR < 1.5 ? pulseR : 1.5;
//            rc_servo_send_pulse_normalized(7, -pulseR);
//
//            //ch = 8; left servo
//            pulseL = servo_pos * max_speed;
//            pulseL = pulseL < 1.5 ? pulseL : 1.5;
//            rc_servo_send_pulse_normalized(8, pulseL);
//        } else if (corr_factor_avg + 0.60/4 < 0) {
//            //ch = 7; right servo
//            pulseR = r_wheel_gain * (servo_pos * max_speed);
//            pulseR = pulseR < 1.5 ? pulseR : 1.5;
//            rc_servo_send_pulse_normalized(7, -pulseR);
//
//            //ch = 8; left servo
//            pulseL = (servo_pos * max_speed * 3);
//            pulseL = pulseL < 1.5 ? pulseL : 1.5;
//            rc_servo_send_pulse_normalized(8, pulseL);
//        }
//        else if (corr_factor_avg - 0.5/4 > 0) {
//            //ch = 7; right servo
//            pulseR = r_wheel_gain * (servo_pos * max_speed * 2);
//            pulseR = pulseR < 1.5 ? pulseR : 1.5;
//            rc_servo_send_pulse_normalized(7, -pulseR);
//
//            //ch = 8; left servo
//            pulseL = servo_pos * max_speed;
//            pulseL = pulseL < 1.5 ? pulseL : 1.5;
//            rc_servo_send_pulse_normalized(8, pulseL);
//        } else if (corr_factor_avg + 0.5/4 < 0) {
//            //ch = 7; right servo
//            pulseR = r_wheel_gain * (servo_pos * max_speed);
//            pulseR = pulseR < 1.5 ? pulseR : 1.5;
//            rc_servo_send_pulse_normalized(7, -pulseR);
//
//            //ch = 8; left servo
//            pulseL = (servo_pos * max_speed * 2);
//            pulseL = pulseL < 1.5 ? pulseL : 1.5;
//            rc_servo_send_pulse_normalized(8, pulseL);
//        }
        else {
            //ch = 7; right servo
            pulseR = r_wheel_gain * (servo_pos * max_speed);
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = (servo_pos * max_speed);
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        }
        // sleep roughly enough to maintain frequency_hz
        rc_usleep(1000000 / frequency_hz);
    }
    // turn off power rail and cleanup
    rc_servo_power_rail_en(0);
    rc_servo_cleanup();
    rc_dsm_cleanup();
    return 0;
}
