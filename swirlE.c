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

    printf("red test\n");
    //robot_forward(1,frequency_hz);

    double pulseR = 0.0;
    double pulseL = 0.0;
    int bluectr = 0;
    bool saw_blue = false;
    //setting states for operation
    bool state[4] = {1, 0, 0, 0}; //initial state
    //state is arranged as: forward/back, capturing, dropping, returning
    robot_move_cup_up(frequency_hz); //cup starts down
    rc_usleep(SLEEP_TIME);
    //MAIN CODE:
    while(1) {
        if(!state[1] && !state[2]) { //line following state X00X
            rc_usleep(20);
            double l_red_val = colour_sensor_red(CS_OUT1);
            rc_usleep(20);
            double r_red_val = colour_sensor_red(CS_OUT2);
            rc_usleep(20);
            l_r_avg = rolling_avg(l_red_arr, &l_red_val, &l_r_sum);
            rc_usleep(20);
            r_r_avg = rolling_avg(r_red_arr, &r_red_val, &r_r_sum);
            rc_usleep(20);
            double l_green_val = colour_sensor_green(CS_OUT1);
            rc_usleep(20);
            double r_green_val = colour_sensor_green(CS_OUT2);
            rc_usleep(20);
            l_g_avg = rolling_avg(l_green_arr, &l_green_val, &l_g_sum);
            rc_usleep(20);
            r_g_avg = rolling_avg(r_green_arr, &r_green_val, &r_g_sum);
            rc_usleep(20);


            double l_blue_val = colour_sensor_blue(CS_OUT1);
            rc_usleep(10);
            double r_blue_val = colour_sensor_blue(CS_OUT2);
            rc_usleep(10);
            l_b_avg = rolling_avg(l_blue_arr, &l_blue_val, &l_b_sum);
            rc_usleep(10);
            r_b_avg = rolling_avg(r_blue_arr, &r_blue_val, &r_b_sum);
            rc_usleep(10);

            leftC_sense = l_r_avg + l_g_avg + l_b_avg; //left

            rightC_sense = r_r_avg + r_g_avg + r_b_avg; //right

            corr_factor = gain * (rightC_sense - leftC_sense) / (leftC_sense + rightC_sense);
            corr_factor_avg = rolling_avg(corr_arr, &corr_factor, &sum);
            servo_pos += direction * sweep_limit / frequency_hz;

            if (servo_pos > sweep_limit) {
                servo_pos = sweep_limit;
            }
            printf("%f\n", corr_factor_avg);
            //line following state: 1000 and 0000
            if (1) { //only turn if seeing red
                printf("\nturning\n");
                if ((l_r_avg + 300) < LEFT_RED_LINE) { //left sensor greater than right, turn right

                    //ch = 7; right servo, -1 pulse
                    pulseR = r_wheel_gain * (servo_pos * max_speed * 10);
                    pulseR = pulseR < 1.5 ? pulseR : 1.5;
                    rc_servo_send_pulse_normalized(7, -pulseR);

                    //ch = 8; left servo
                    pulseL = 0 * servo_pos * max_speed;
                    pulseL = pulseL < 1.5 ? pulseL : 1.5;
                    rc_servo_send_pulse_normalized(8, pulseL);
                } else if ((r_r_avg + 300) < RIGHT_RED_LINE) { //right sensor greater than left, turn left
                    //ch = 7; right servo
                    pulseR = r_wheel_gain * 0 * (servo_pos * max_speed);
                    pulseR = pulseR < 1.5 ? pulseR : 1.5;
                    rc_servo_send_pulse_normalized(7, -pulseR);

                    //ch = 8; left servo
                    pulseL = (servo_pos * max_speed * 10);
                    pulseL = pulseL < 1.5 ? pulseL : 1.5;
                    rc_servo_send_pulse_normalized(8, pulseL);
                }
                else if ((l_r_avg + 200) < LEFT_RED_LINE) { //left sensor greater than right, turn right

                    //ch = 7; right servo, -1 pulse
                    pulseR = r_wheel_gain * (servo_pos * max_speed * 5);
                    pulseR = pulseR < 1.5 ? pulseR : 1.5;
                    rc_servo_send_pulse_normalized(7, -pulseR);

                    //ch = 8; left servo
                    pulseL = 0.5 * servo_pos * max_speed;
                    pulseL = pulseL < 1.5 ? pulseL : 1.5;
                    rc_servo_send_pulse_normalized(8, pulseL);
                } else if ((r_r_avg + 200) < RIGHT_RED_LINE) { //right sensor greater than left, turn left
                    //ch = 7; right servo
                    pulseR = r_wheel_gain * 0.5 * (servo_pos * max_speed);
                    pulseR = pulseR < 1.5 ? pulseR : 1.5;
                    rc_servo_send_pulse_normalized(7, -pulseR);

                    //ch = 8; left servo
                    pulseL = (servo_pos * max_speed * 5);
                    pulseL = pulseL < 1.5 ? pulseL : 1.5;
                    rc_servo_send_pulse_normalized(8, pulseL);
                }
                else if ((l_r_avg + 100) < LEFT_RED_LINE) { //left sensor greater than right, turn right

                    //ch = 7; right servo, -1 pulse
                    pulseR = r_wheel_gain * (servo_pos * max_speed * 5);
                    pulseR = pulseR < 1.5 ? pulseR : 1.5;
                    rc_servo_send_pulse_normalized(7, -pulseR);

                    //ch = 8; left servo
                    pulseL = 1 * servo_pos * max_speed;
                    pulseL = pulseL < 1.5 ? pulseL : 1.5;
                    rc_servo_send_pulse_normalized(8, pulseL);
                } else if ((r_r_avg + 100) < RIGHT_RED_LINE) { //right sensor greater than left, turn left
                    //ch = 7; right servo
                    pulseR = r_wheel_gain * 1 * (servo_pos * max_speed);
                    pulseR = pulseR < 1.5 ? pulseR : 1.5;
                    rc_servo_send_pulse_normalized(7, -pulseR);

                    //ch = 8; left servo
                    pulseL = (servo_pos * max_speed * 5);
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
            }
            if(state[0] && !state[1] && !state[2] && !state[3]) { //if going towards bullseye in state 1000
                if((l_b_avg + 100) < BLUE_THRESHOLD || (r_b_avg + 100) < BLUE_THRESHOLD) {
                    bluectr++;
                }
                if((l_b_avg + 100) > BLUE_THRESHOLD || (r_b_avg + 100) > BLUE_THRESHOLD) {
                    bluectr = 0;
                }
                if(bluectr >= 1) {
                    saw_blue = true;
                }
                if(saw_blue) {
                    printf("\nsaw blue, going to capture mode\n");
                    //state = {1, 1, 0, 0}; //change state to capture mode
                    //pause change to blue temporarily
                    state[0] = 1;
                    state[1] = 1;
                    state[2] = 0;
                    state[3] = 0;


                }
            }
            if(!state[0] && !state[1] && !state[2] && !state[3]) { //if sees tree on the way back in state 0000
                if(distance_measurement_left() < 12) {
                    printf("\nsaw dropzone, going to drop off mode\n");
                    //state = {0,0,1,0};
                    state[0] = 0;
                    state[1] = 0;
                    state[2] = 1;
                    state[3] = 0;
                }
            }
            if(!state[0] && !state[1] && !state[2] && state[3]) { //returning after drop off in state 0001
                if((r_r_avg - DOUBLE_RED_THRESHOLD) > (r_g_avg+r_b_avg)/2.0 || (l_r_avg - DOUBLE_RED_THRESHOLD) > (l_g_avg+l_b_avg)/2.0) {
                    printf("\nend of course, shutting off\n");
                    break;
                }
            }

        }
        if(state[0] && state[1] && !state[2] && !state[3]) { //capturing state 1100
            //stop
            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, 0);

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, 0);

            robot_move_forward_bullseye(frequency_hz);
            rc_usleep(SLEEP_TIME);
            robot_move_cup_down(frequency_hz);
            rc_usleep(SLEEP_TIME);
            //robot_turn_cw(110*2,frequency_hz);
            robot_turn_ninety(frequency_hz,1);
            //state = {0, 0, 0, 0}; //set state back to line following
            state[0] = 0;
            state[1] = 0;
            state[2] = 0;
            state[3] = 0;
        }
        if(!state[0] && !state[1] && state[2] && !state[3]) { //drop off state 0010
            //stop
            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, 0);

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, 0);
            rc_usleep(SLEEP_TIME);
            robot_turn_cw((110.0)*3.0,frequency_hz); //turn cw 270 to get drop zone on left
            rc_usleep(SLEEP_TIME);
            robot_move_cup_up(frequency_hz); //let go of lego man
            rc_usleep(SLEEP_TIME);
            robot_turn_cw(110.0,frequency_hz); //turn cw 90
            rc_usleep(SLEEP_TIME);
            //state = {0,0,0,1};
            state[0] = 0;
            state[1] = 0;
            state[2] = 0;
            state[3] = 1;
        }

    }

    // turn off power rail and cleanup
    rc_servo_power_rail_en(0);
    rc_servo_cleanup();
    rc_dsm_cleanup();
    return 0;
}
