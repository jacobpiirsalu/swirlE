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
//TO-DO: determine min servo speed
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
    running = 1;
    double leftC_sense = 0.0;
    double rightC_sense = 0.0;
    double corr_factor = 0.0;
    double max_speed = 0.08; //base speed of swirlE
    double gain = 10;
    double r_wheel_gain = 1.35;
    int avg_val_ctr = 0;
    double corr_arr[7];
    double corr_factor_avg = 0.0;
    double sum = 0;

    printf("starting main while loop\n");
    uint64_t *timeRise1Ptr = malloc(sizeof(uint64_t));
    while(1) {
        printf("%d\n",rc_gpio_poll(3, OUT, 10000, timeRise1Ptr);
    }

    while (0) {
        leftC_sense = colour_sensor_red(CS_OUT1) + colour_sensor_green(CS_OUT1) + colour_sensor_blue(CS_OUT1); //left
        rightC_sense = colour_sensor_red(CS_OUT2) + colour_sensor_green(CS_OUT2) + colour_sensor_blue(CS_OUT2); //right

        corr_factor = gain * (rightC_sense - leftC_sense) / (leftC_sense + rightC_sense);
        servo_pos += direction * sweep_limit / frequency_hz;

        if (servo_pos > sweep_limit) {
            servo_pos = sweep_limit;
        }

        sum = sum - corr_arr[avg_val_ctr];
        corr_arr[avg_val_ctr] = corr_factor;
        sum = sum + corr_factor;
        avg_val_ctr = (avg_val_ctr + 1) % 7; //window size
        corr_factor_avg = sum / 7;

        printf("%f\n", corr_factor_avg);

        if (corr_factor_avg - 1.15 > 0) {
            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1 * r_wheel_gain * (servo_pos * max_speed * 6));

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, 0 * servo_pos * max_speed);
        } else if (corr_factor_avg + 1.15 < 0) {
            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1 * r_wheel_gain * 0 * (servo_pos * max_speed));

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, (servo_pos * max_speed * 6));
        } else if (corr_factor_avg - 1.0 > 0) {
            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1 * r_wheel_gain * (servo_pos * max_speed * 6));

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, servo_pos * max_speed);
        } else if (corr_factor_avg + 1.0 < 0) {
            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1 * r_wheel_gain * (servo_pos * max_speed));

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, (servo_pos * max_speed * 6));
        } else if (corr_factor_avg - 0.8 > 0) {
            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1 * r_wheel_gain * (servo_pos * max_speed * 5));

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, servo_pos * max_speed);
        } else if (corr_factor_avg + 0.8 < 0) {
            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1 * r_wheel_gain * (servo_pos * max_speed));

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, (servo_pos * max_speed * 5));
        } else if (corr_factor_avg - 0.7 > 0) {
            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1 * r_wheel_gain * (servo_pos * max_speed * 4));

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, servo_pos * max_speed);
        } else if (corr_factor_avg + 0.7 < 0) {
            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1 * r_wheel_gain * (servo_pos * max_speed));

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, (servo_pos * max_speed * 4));
        } else if (corr_factor_avg - 0.6 > 0) {
            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1 * r_wheel_gain * (servo_pos * max_speed * 3));

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, servo_pos * max_speed);
        } else if (corr_factor_avg + 0.6 < 0) {
            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1 * r_wheel_gain * (servo_pos * max_speed));

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, (servo_pos * max_speed * 3));
        } else if (corr_factor_avg - 0.4 > 0) {
            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1 * r_wheel_gain * (servo_pos * max_speed * 2));

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, servo_pos * max_speed);
        } else if (corr_factor_avg + 0.4 < 0) {
            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1 * r_wheel_gain * (servo_pos * max_speed));

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, (servo_pos * max_speed * 2));
        } else {
            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1 * r_wheel_gain * (servo_pos * max_speed));

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, (servo_pos * max_speed));
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
