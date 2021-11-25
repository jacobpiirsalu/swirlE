/**
 *@file swirlE.c
 *@example
 * NOTE - MVP-1.0 BRANCH WAS KNOWN TO BE WORKING
 * AT 10:40pm 2021-11-24
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
#include <signal.h>
#include <rc/button.h>
#include <stdio.h>
#include <signal.h>
#include <rc/button.h>
#include <rc/time.h>


#include "swirlelib.h" //all robot functions written here

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

bool start = false;
bool stop = false;
static void __on_pause_press(void)
{
    printf("Pause Pressed\n");
    stop = true;
    return;
}
static void __on_pause_release(void)
{
    printf("Pause Released\n");
    return;
}
static void __on_mode_press(void)
{
    printf("Mode Pressed\n");
    start = true;
    return;
}
static void __on_mode_release(void)
{
    printf("Mode Released\n");
    return;
}

int main() {
    // initialize pause and mode buttons
    if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
                      RC_BTN_DEBOUNCE_DEFAULT_US)){
        fprintf(stderr,"ERROR: failed to init buttons\n");
        return -1;
    }
    if(rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH,
                      RC_BTN_DEBOUNCE_DEFAULT_US)){
        fprintf(stderr,"ERROR: failed to init buttons\n");
        return -1;
    }
    // set signal handler so the loop can exit cleanly
    //signal(SIGINT, __signal_handler);
    //running = 1;
    // Assign callback functions
    rc_button_set_callbacks(RC_BTN_PIN_PAUSE, __on_pause_press, __on_pause_release);
    rc_button_set_callbacks(RC_BTN_PIN_MODE, __on_mode_press, __on_mode_release);
    //toggle leds till the program state changes
    //printf("Press buttons to see response\n");


    int loopctr = 0;
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
    double max_speed = 0.08*1;//base speed of swirlE
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

    //printf("red test\n");
    //robot_forward(1,frequency_hz);

    double pulseR = 0.0;
    double pulseL = 0.0;
    int bluectr = 0;
    int redctr = 0;
    bool saw_blue = false;
    bool saw_red = false;
    //setting states for operation
    bool state[4] = {1, 0, 0, 0}; //initial state
    //rc_usleep(5e6);
    //state is arranged as: forward/back, capturing, dropping, returning
//    rc_usleep(SLEEP_TIME*100);
//    robot_turn_ninety_cw(frequency_hz);
//    rc_usleep(SLEEP_TIME*100);
//    robot_turn_ninety_ccw(frequency_hz);
    //robot_turn_one_eighty(frequency_hz,1);
    printf("waiting for button press\n");
    while(start == false) {
        //printf("waiting for button press\n");
        rc_usleep(1e4);
    }
    robot_move_cup_up(frequency_hz); //cup starts down

    //MAIN CODE:
    printf("starting main\n");
    while(1) {
        if(stop == true) {
            break;
        }
        //printf("%d\n",loopctr);
        loopctr++;
        //printf("%d\n",loopctr);
        if(!state[1] && !state[2]) { //line following state X00X
            //printf("\nline following\n");
            rc_usleep(20);
            double l_red_val = colour_sensor_red(CS_OUT1);
            rc_usleep(20);

            double r_red_val = colour_sensor_red(CS_OUT2);
            rc_usleep(20);
            //printf("\n%f  %f",l_red_val,r_red_val);

            //printf("\nsaw red\n");
            l_r_avg = rolling_avg(l_red_arr, &l_red_val, &l_r_sum);
            //rc_usleep(20);
            r_r_avg = rolling_avg(r_red_arr, &r_red_val, &r_r_sum);
            //rc_usleep(20);
            double l_green_val = colour_sensor_green(CS_OUT1);
            rc_usleep(20);
            double r_green_val = colour_sensor_green(CS_OUT2);
            rc_usleep(20);
            l_g_avg = rolling_avg(l_green_arr, &l_green_val, &l_g_sum);
            //rc_usleep(20);
            r_g_avg = rolling_avg(r_green_arr, &r_green_val, &r_g_sum);
            //rc_usleep(20);
            printf("%f %f\n",l_r_avg,r_r_avg);

            double l_blue_val = colour_sensor_blue(CS_OUT1);
            rc_usleep(20);
            double r_blue_val = colour_sensor_blue(CS_OUT2);
            rc_usleep(20);
            l_b_avg = rolling_avg(l_blue_arr, &l_blue_val, &l_b_sum);
            rc_usleep(20);
            r_b_avg = rolling_avg(r_blue_arr, &r_blue_val, &r_b_sum);
            rc_usleep(20);
            //printf("\nsaw color\n");
            if(state[0] && !state[1] && !state[2] && !state[3]) { //if going towards bullseye in state 1000
                if(loopctr > BULLSEYE_LOOP_CTR) {
                    if (((l_b_avg) < BLUE_THRESHOLD + 100 || (r_b_avg) < BLUE_THRESHOLD + 100)) {
                        bluectr++;
                    }
                    if (((l_b_avg) > BLUE_THRESHOLD + 100 || (r_b_avg) > BLUE_THRESHOLD + 100)) {
                        bluectr = 0;
                    }
                    if (bluectr >= 4) { //4
                        saw_blue = true;
                    }
                    if (saw_blue) {
                        //printf("\nsaw blue, going to capture mode\n");
                        //state = {1, 1, 0, 0}; //change state to capture mode
                        //pause change to blue temporarily
                        state[0] = 1;
                        state[1] = 1;
                        state[2] = 0;
                        state[3] = 0;
                    }
                }
            }

            servo_pos += direction * sweep_limit / frequency_hz;

            if (servo_pos > sweep_limit) {
                servo_pos = sweep_limit;
            }
            //printf("\nturning\n");
            //printf("%f %f\n", l_r_avg, r_r_avg);
            if ((l_r_avg + 300) < LEFT_RED_LINE) { //left sensor greater than right, turn right

                //ch = 7; right servo, -1 pulse
                pulseR = r_wheel_gain * (servo_pos * max_speed * 10/2.0);
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
                pulseL = (servo_pos * max_speed * 10/2.0);
                pulseL = pulseL < 1.5 ? pulseL : 1.5;
                rc_servo_send_pulse_normalized(8, pulseL);
            }
            else if ((l_r_avg + 200) < LEFT_RED_LINE) { //left sensor greater than right, turn right

                //ch = 7; right servo, -1 pulse
                pulseR = r_wheel_gain * (servo_pos * max_speed * 5/2.0);
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
                pulseR = r_wheel_gain * (servo_pos * max_speed * 5/2.0);
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

            //printf("%f\n", corr_factor_avg);
            //line following state: 1000 and 0000


            if(!state[0] && !state[1] && !state[2] && !state[3]) { //if sees tree on the way back in state 0000
                if(distance_measurement_left() < 12) {
                    //printf("\nsaw dropzone, going to drop off mode\n");
                    //state = {0,0,1,0};
                    state[0] = 0;
                    state[1] = 0;
                    state[2] = 1;
                    state[3] = 0;
                }
            }
            if(!state[0] && !state[1] && !state[2] && state[3]) { //returning after drop off in state 0001
                if ((l_r_avg + 200) < LEFT_RED_LINE && (r_r_avg + 200) < RIGHT_RED_LINE) {
                    redctr++;
                }
                if ((l_r_avg + 200) > LEFT_RED_LINE && (r_r_avg + 200) > RIGHT_RED_LINE) {
                    redctr = 0;
                }
                if (redctr >= 4) { //4
                    saw_red = true;
                }
                if(saw_red) {
                    break;
                }
            }

        }
        if(state[0] && state[1] && !state[2] && !state[3]) { //capturing state 1100

            robot_move_forward_bullseye(frequency_hz);
            rc_usleep(SLEEP_TIME*10);
            robot_move_cup_down(frequency_hz);
            rc_usleep(SLEEP_TIME*10);
            robot_back(10,frequency_hz);
            //robot_turn_cw(110*2,frequency_hz);
            robot_turn_one_eighty(frequency_hz,1);

            rc_usleep(SLEEP_TIME*10);
            //robot_back(5,frequency_hz);


            //state = {0, 0, 0, 0}; //set state back to line following
            state[0] = 0;
            state[1] = 0;
            state[2] = 0;
            state[3] = 0;
        }
        if(!state[0] && !state[1] && state[2] && !state[3]) { //drop off state 0010
//            //stop
//            //ch = 7; right servo
//            rc_servo_send_pulse_normalized(7, 0);
//
//            //ch = 8; left servo
//            rc_servo_send_pulse_normalized(8, 0);
//            rc_usleep(SLEEP_TIME);
//            robot_turn_ninety_ccw(frequency_hz);
//            rc_usleep(SLEEP_TIME*10);
//            robot_back(3,frequency_hz);
//            rc_usleep(SLEEP_TIME*10);
//
//            robot_move_cup_up(frequency_hz); //let go of lego man
//            rc_usleep(SLEEP_TIME);
//            robot_forward(1,frequency_hz);
//            rc_usleep(SLEEP_TIME*10);
//            robot_turn_ninety_cw(frequency_hz); //turn cw 90
//            rc_usleep(SLEEP_TIME);
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
    // cleanup and exit
    rc_button_cleanup();
    return 0;
}
