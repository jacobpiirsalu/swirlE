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
    uint64_t dsm_nanos = 0;
    int width_us = 0;
    int ch = 0;    // channel to test, 0 means all channels
    double direction = 1;    // switches between 1 &-1 in sweep mode
    int frequency_hz = 50;    // default 50hz frequency to send pulses
    robot_gpio_init();
    // initialize PRU
    if (rc_servo_init()) return -1;
    // turn on power - Turning On 6V Servo Power Rail
    rc_servo_power_rail_en(1);
    running = 1;
    printf("line-following-test\n");
    double leftC_sense = 0.0;
    double rightC_sense = 0.0;
    double left_corr_factor = 0.0;
    double right_corr_factor = 0.0;
    double left_servo_kp = 0.0;
    double right_servo_kp = 0.0;
    double corr_factor = 0.0;
    double max_speed = 10.0;
    double gain = 10;
    double r_wheel_gain = 1.35;
    int avg_val_ctr = 0;
    double corr_arr[7];
    //double delta_arr[5];
    int bluectr = 0;
    int loopctr = 0;
    double corr_factor_avg = 0.0;
    double sum = 0;
    while (1) {
        //colour_sensor(CS_OUT1);//left sensor
        //colour_sensor(CS_OUT2);//right sensor
        leftC_sense = colour_sensor_red(CS_OUT1) + colour_sensor_green(CS_OUT1) + colour_sensor_blue(CS_OUT1);
        rightC_sense = colour_sensor_red(CS_OUT2) + colour_sensor_green(CS_OUT2) + colour_sensor_blue(CS_OUT2);

        corr_factor = gain * (rightC_sense - leftC_sense) / (leftC_sense + rightC_sense);
        servo_pos += direction * sweep_limit / frequency_hz;

        if (servo_pos > sweep_limit) {
            servo_pos = sweep_limit;
        }

        //left_corr_factor = gain*abs(leftC_sense - rightC_sense)/(leftC_sense + rightC_sense);
//        corr_arr[avg_val_ctr] = corr_factor;
//        avg_val_ctr++;
//        //delta_arr[avg_val_ctr] = rightC_sense - leftC_sense;
//        if(avg_val_ctr>=4)
//        {
//            avg_val_ctr = 0;
//            double sum = 0;
//            for(int i = 0; i<4; i++)
//            {
//                sum += corr_arr[i];
//                corr_factor_avg = sum/4.0;
//            }
//        }

        sum = sum - corr_arr[avg_val_ctr];
        corr_arr[avg_val_ctr] = corr_factor;
        sum =  sum + corr_factor;
        avg_val_ctr = (avg_val_ctr+1) % 7; //window size
        corr_factor_avg = sum / 7;



        printf("%f\n",corr_factor_avg);
//        if(is_green_detected(CS_OUT1,CS_OUT2))
//        {
//            printf("saw green\n");
//        }
        loopctr++;
        if(is_blue_detected(CS_OUT1,CS_OUT2))
        {
            printf("saw blue\n");
            bluectr++;
        }
        if(bluectr > 5){
            break;
        }
        if(loopctr > 50){
            loopctr=0;
            bluectr =0;
        }
        if (false) {}
        else if (corr_factor_avg - 1.15 > 0) {

            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1*r_wheel_gain * (servo_pos * 0.09*6));//0.076-.001
            //printf("R: %f\n", pulse);
            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, 0*servo_pos * 0.09);
        } else if (corr_factor_avg + 1.15 < 0) {

            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1*r_wheel_gain * 0*(servo_pos * 0.09));

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, (servo_pos * 0.09*6));
            //printf("L: %f\n", pulse);

        }
        else if (corr_factor_avg - 1.0 > 0) {

            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1*r_wheel_gain * (servo_pos * 0.09*6));
            //printf("R: %f\n", pulse);
            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, servo_pos * 0.09);
        } else if (corr_factor_avg + 1.0 < 0) {

            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1*r_wheel_gain * (servo_pos * 0.09));

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, (servo_pos * 0.09*6));
            //printf("L: %f\n", pulse);

        }
        else if (corr_factor_avg - 0.8 > 0) {

            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1*r_wheel_gain * (servo_pos * 0.09*5));
            //printf("R: %f\n", pulse); d
            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, servo_pos * 0.09);
        } else if (corr_factor_avg + 0.8 < 0) {

            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1*r_wheel_gain * (servo_pos * 0.09));

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, (servo_pos * 0.09*5));
            //printf("L: %f\n", pulse);

//        } else if (corr_factor_avg - 0.7 > 0) {
//
//            //ch = 7; right servo
//            rc_servo_send_pulse_normalized(7, -1*r_wheel_gain * (servo_pos * 0.09*1.5));
//            //printf("R: %f\n", pulse);
//            //ch = 8; left servo
//            rc_servo_send_pulse_normalized(8, servo_pos * 0.09);
//        } else if (corr_factor_avg + 0.7 < 0) {
//
//            //ch = 7; right servo
//            rc_servo_send_pulse_normalized(7, -1*r_wheel_gain * (servo_pos * 0.09));
//
//            //ch = 8; left servo
//            rc_servo_send_pulse_normalized(8, (servo_pos * 0.09*1.5));
//            //printf("L: %f\n", pulse);
//
        }
        else if (corr_factor_avg - 0.7 > 0) {

            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1*r_wheel_gain * (servo_pos * 0.09*4));
            //printf("R: %f\n", pulse);
            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, servo_pos * 0.09);
        } else if (corr_factor_avg + 0.7 < 0) {

            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1 * r_wheel_gain * (servo_pos * 0.09));

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, (servo_pos * 0.09 * 4));
            //printf("L: %f\n", pulse);
        }
        else if (corr_factor_avg - 0.6 > 0) {

            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1*r_wheel_gain * (servo_pos * 0.09*3));
            //printf("R: %f\n", pulse);
            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, servo_pos * 0.09);
        } else if (corr_factor_avg + 0.6 < 0) {

            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1 * r_wheel_gain * (servo_pos * 0.09));

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, (servo_pos * 0.09 * 3));
            //printf("L: %f\n", pulse);
        }
        else if (corr_factor_avg - 0.4 > 0) {

            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1*r_wheel_gain * (servo_pos * 0.09*2));
            //printf("R: %f\n", pulse);
            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, servo_pos * 0.09);
        } else if (corr_factor_avg + 0.4 < 0) {

            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1 * r_wheel_gain * (servo_pos * 0.09));

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, (servo_pos * 0.09 * 2));
            //printf("L: %f\n", pulse);
        }
        else {
            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -1*r_wheel_gain * (servo_pos * 0.09));

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, (servo_pos * 0.09));
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
