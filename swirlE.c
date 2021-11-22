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

    int loopctr = 0;
    printf("red test\n");
    int blue_ctr = 0;

    //while(!(robot_move_cup_up(frequency_hz) == 1));
    //rc_usleep(1000000/2.0);
    double pulseR = 0.0;
    double pulseL = 0.0;
    while(0){
        double l_red_val = colour_sensor_red(CS_OUT1);
        rc_usleep(1);
        double r_red_val = colour_sensor_red(CS_OUT2);
        l_r_avg = rolling_avg(l_red_arr,&l_red_val,&l_r_sum);
        rc_usleep(1);
        r_r_avg = rolling_avg(r_red_arr,&r_red_val,&r_r_sum);
        //printf("%f,%f\n",l_r_avg,r_r_avg);

//        double l_green_val = colour_sensor_green(CS_OUT1);
//        double r_green_val = colour_sensor_green(CS_OUT2);
//        l_g_avg = rolling_avg(l_green_arr,&l_green_val,&l_g_sum);
//        r_g_avg = rolling_avg(r_green_arr,&r_green_val,&r_g_sum);
//
//        double l_blue_val = colour_sensor_blue(CS_OUT1);
//        double r_blue_val = colour_sensor_blue(CS_OUT2);
//        l_b_avg = rolling_avg(l_blue_arr,&l_blue_val,&l_b_sum);
//        r_b_avg = rolling_avg(r_blue_arr,&r_blue_val,&r_b_sum);


        printf("%f,%f\n",l_r_avg,r_r_avg);
    }
    while(1) {
        servo_pos += direction * sweep_limit / frequency_hz;

        if (servo_pos > sweep_limit) {
            servo_pos = sweep_limit;
        }

        //ch = 7; right servo
        pulseR = r_wheel_gain * (servo_pos * max_speed);
        pulseR = pulseR < 1.5 ? pulseR : 1.5;
        rc_servo_send_pulse_normalized(7, -pulseR);

        //ch = 8; left servo
        pulseL = (servo_pos * max_speed);
        pulseL = pulseL < 1.5 ? pulseL : 1.5;
        rc_servo_send_pulse_normalized(8, pulseL);

        double l_red_val = colour_sensor_red(CS_OUT1);
        double r_red_val = colour_sensor_red(CS_OUT2);
        l_r_avg = rolling_avg(l_red_arr,&l_red_val,&l_r_sum);
        r_r_avg = rolling_avg(r_red_arr,&r_red_val,&r_r_sum);
        //printf("%f,%f\n",l_r_avg,r_r_avg);

        double l_green_val = colour_sensor_green(CS_OUT1);
        double r_green_val = colour_sensor_green(CS_OUT2);
        l_g_avg = rolling_avg(l_green_arr,&l_green_val,&l_g_sum);
        r_g_avg = rolling_avg(r_green_arr,&r_green_val,&r_g_sum);

        double l_blue_val = colour_sensor_blue(CS_OUT1);
        double r_blue_val = colour_sensor_blue(CS_OUT2);
        l_b_avg = rolling_avg(l_blue_arr,&l_blue_val,&l_b_sum);
        r_b_avg = rolling_avg(r_blue_arr,&r_blue_val,&r_b_sum);

        double blue_delta_l = l_b_avg - (l_r_avg + l_g_avg) / 2.0; //blue should be higher
        double blue_delta_r = r_b_avg - (r_r_avg + r_g_avg) / 2.0; //blue should be higher
        //when over blue
        //printf("%f\n", blue_delta_l);
        if (blue_delta_l > 0) {
            blue_ctr++;
        }
        if (blue_delta_l < 0) {
            blue_ctr = 0;
        }
        if (blue_ctr > 2) {
            blue_ctr = 0;
            printf("\nsaw blue\n");
            break;
        }

    }
//    rc_usleep(1000000/2.0);
//    while(!(robot_move_forward_bullseye(frequency_hz))==1);
//    rc_usleep(1000000/2.0);
//    while(!(robot_move_cup_down(frequency_hz)==1));
//    rc_usleep(1000000/2.0);
//    while(!(robot_turn_cw(110,frequency_hz)));
////    rc_usleep(1000000/2.0);
////    while(!(robot_turn_cw(90,frequency_hz)));
//    rc_usleep(1000000/2.0);
//    robot_forward(5,frequency_hz);

    //robot_move_cup_up(frequency_hz);
    while (1) {
        loopctr++;

        leftC_sense = colour_sensor_red(CS_OUT1) + colour_sensor_green(CS_OUT1) + colour_sensor_blue(CS_OUT1); //left
        rc_usleep(1);
        rightC_sense = colour_sensor_red(CS_OUT2) + colour_sensor_green(CS_OUT2) + colour_sensor_blue(CS_OUT2); //right

        corr_factor = gain * (rightC_sense - leftC_sense) / (leftC_sense + rightC_sense);

//        sum = sum - corr_arr[avg_val_ctr];
//        corr_arr[avg_val_ctr] = corr_factor;
//        sum = sum + corr_factor;
//        avg_val_ctr = (avg_val_ctr + 1) % WINDOW; //window size
//        corr_factor_avg = sum / WINDOW;
        corr_factor_avg = rolling_avg(corr_arr,&corr_factor,&sum);

        //printf("%f\n",corr_factor_avg);

        servo_pos += direction * sweep_limit / frequency_hz;

        if (servo_pos > sweep_limit) {
            servo_pos = sweep_limit;
        }


        if (corr_factor_avg - 1.10/1.0000 > 0) {

            //ch = 7; right servo, -1 pulse
            pulseR = r_wheel_gain * (servo_pos * max_speed * 6);
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = 0 * servo_pos * max_speed;
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        } else if (corr_factor_avg + 1.10/1.0000 < 0) {
            //ch = 7; right servo
            pulseR = r_wheel_gain * 0 * (servo_pos * max_speed);
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = (servo_pos * max_speed * 6);
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        } else if (corr_factor_avg - 1.0/1.0000 > 0) {
            //ch = 7; right servo
            pulseR = r_wheel_gain * (servo_pos * max_speed * 6);
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = servo_pos * max_speed * 0.25;
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        } else if (corr_factor_avg + 1.0/1.0000 < 0) {
            //ch = 7; right servo
            pulseR = r_wheel_gain * (servo_pos * max_speed)*0.25;
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = servo_pos * max_speed * 6;
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        } else if (corr_factor_avg - 0.9/1.0000 > 0) {
            //ch = 7; right servo
            pulseR = r_wheel_gain * (servo_pos * max_speed * 6);
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = servo_pos * max_speed * 0.5;
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        } else if (corr_factor_avg + 0.9/1.0000 < 0) {
            //ch = 7; right servo
            pulseR = r_wheel_gain * (servo_pos * max_speed)*0.5;
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = servo_pos * max_speed * 6;
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        } else if (corr_factor_avg - 0.8/1.0000 > 0) {
            //ch = 7; right servo
            pulseR = r_wheel_gain * (servo_pos * max_speed * 5);
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = servo_pos * max_speed * 0.75;
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        } else if (corr_factor_avg + 0.8/1.0000 < 0) {
            //ch = 7; right servo
            pulseR = r_wheel_gain * (servo_pos * max_speed) * 0.75;
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = servo_pos * max_speed * 5;
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        } else if (corr_factor_avg - 0.7/1.0000 > 0) {
            //ch = 7; right servo
            pulseR = r_wheel_gain * (servo_pos * max_speed * 4);
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = servo_pos * max_speed;
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        } else if (corr_factor_avg + 0.7/1.0000 < 0) {
            //ch = 7; right servo
            pulseR = r_wheel_gain * (servo_pos * max_speed);
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = servo_pos * max_speed * 4;
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        }
        else if (corr_factor_avg - 0.60/1.0000 > 0) {
            //ch = 7; right servo
            pulseR = r_wheel_gain * (servo_pos * max_speed * 3);
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = servo_pos * max_speed;
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        } else if (corr_factor_avg + 0.60/1.0000 < 0) {
            //ch = 7; right servo
            pulseR = r_wheel_gain * (servo_pos * max_speed);
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = (servo_pos * max_speed * 3);
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        }
        else if (corr_factor_avg - 0.5/1.0000 > 0) {
            //ch = 7; right servo
            pulseR = r_wheel_gain * (servo_pos * max_speed * 2);
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = servo_pos * max_speed;
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        } else if (corr_factor_avg + 0.5/1.0000 < 0) {
            //ch = 7; right servo
            pulseR = r_wheel_gain * (servo_pos * max_speed);
            pulseR = pulseR < 1.5 ? pulseR : 1.5;
            rc_servo_send_pulse_normalized(7, -pulseR);

            //ch = 8; left servo
            pulseL = (servo_pos * max_speed * 2);
            pulseL = pulseL < 1.5 ? pulseL : 1.5;
            rc_servo_send_pulse_normalized(8, pulseL);
        }
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


        //printf("%d\n",loopctr) ;
        if(loopctr>10) {
//            double l_red_val = colour_sensor_red(CS_OUT1);
//            double r_red_val = colour_sensor_red(CS_OUT2);
//            l_r_avg = rolling_avg(l_red_arr,&l_red_val,&l_r_sum);
//            r_r_avg = rolling_avg(r_red_arr,&r_red_val,&r_r_sum);
//            //printf("%f,%f\n",l_r_avg,r_r_avg);
//
//            double l_green_val = colour_sensor_green(CS_OUT1);
//            double r_green_val = colour_sensor_green(CS_OUT2);
//            l_g_avg = rolling_avg(l_green_arr,&l_green_val,&l_g_sum);
//            r_g_avg = rolling_avg(r_green_arr,&r_green_val,&r_g_sum);


            double l_blue_val = colour_sensor_blue(CS_OUT1);
            rc_usleep(1);
            double r_blue_val = colour_sensor_blue(CS_OUT2);
            rc_usleep(1);
            l_b_avg = rolling_avg(l_blue_arr,&l_blue_val,&l_b_sum);
            rc_usleep(1);
            r_b_avg = rolling_avg(r_blue_arr,&r_blue_val,&r_b_sum);
            rc_usleep(1);
            printf("%f,%f,%f\n",l_r_avg,l_g_avg,l_b_avg);


            double blue_delta_l = l_b_avg - (l_r_avg + l_g_avg) / 2.0; //blue should be higher
            double blue_delta_r = r_b_avg - (r_r_avg + r_g_avg) / 2.0; //blue should be higher
            //when over blue
            //printf("%f\n", blue_delta_l);
            if (blue_delta_l > 0 || blue_delta_r > 0) {
                blue_ctr++;
            }
            if (blue_delta_l < 0 || blue_delta_r < 0) {
                blue_ctr = 0;
            }
            if (blue_ctr > 2) {
                blue_ctr = 0;
                break;
                //printf("\nsaw blue\n");
            }
        }
        double pulseL = 0.0;
        double pulseR = 0.0;
        servo_pos += direction * sweep_limit / frequency_hz;

        if (servo_pos > sweep_limit) {
            servo_pos = sweep_limit;
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
