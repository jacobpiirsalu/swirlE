/**
 *@file construction_check.c
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


#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
#define POWER "5V"
#define TRIGGER_PORT 1
#define TRIGGER_PIN 25
#define ECHO_PORT 1
#define ECHO_PIN 17
#define GND "0V"

//define colour sensor pins
#define CS_S2 2 //GPIO2_2, LED RED, GREEN
#define CS_S3 3 //GPIO2_3, LED GREEN, WHITE
#define CS_OUT1 2 //GPIO3_2, PURPLE
#define CS_OUT2 1 //GPIO3_1, BLU

static int running;

void robot_forward(int numRotations, int frequency_hz) {
    double servo_pos = 0;
    double direction = 1;    // switches between 1 &-1 in sweep mode
    double sweep_limit = 1.5 * direction;

    // Main loop runs at frequency_hz
    int rotationsCompleted = 0;
    while (running) {
        // scale with frequency
        servo_pos += direction * sweep_limit / frequency_hz;
        // reset pulse width at end of sweep

        if (servo_pos > sweep_limit) {
            servo_pos = sweep_limit;
            rotationsCompleted++;
            //direction = -1;
        }

        /**
          if(servo_pos < (-sweep_limit)){
          servo_pos = -sweep_limit;
          direction = 1;
          }

                    **/
        // send result

        //ch = 7;
        if (rc_servo_send_pulse_normalized(7, -servo_pos / 3.35) == -1) return -1;

        //ch = 8;
        if (rc_servo_send_pulse_normalized(8, servo_pos / 4) == -1) return -1;

        // sleep roughly enough to maintain frequency_hz
        rc_usleep(1000000 / frequency_hz);

        if (rotationsCompleted >= numRotations * 4) break;

    }
}

void robot_back(int numRotations, int frequency_hz) {
    double servo_pos = 0;
    double direction = -1;    // switches between 1 &-1 in sweep mode
    double sweep_limit = 1.5 * direction;

    // Main loop runs at frequency_hz
    int rotationsCompleted = 0;
    while (running) {
        // scale with frequency
        servo_pos += direction * sweep_limit / frequency_hz;
        // reset pulse width at end of sweep
        if (servo_pos > sweep_limit) {
            servo_pos = sweep_limit;
            rotationsCompleted++;
            //direction = -1;
        }

        /**
          if(servo_pos < (-sweep_limit)){
          servo_pos = -sweep_limit;
          direction = 1;
          }

                    **/
        // send result

        //ch = 7;
        if (rc_servo_send_pulse_normalized(7, -servo_pos / 4) == -1) return -1;

        //ch = 8;
        if (rc_servo_send_pulse_normalized(8, servo_pos / 3.6) == -1) return -1;

        // sleep roughly enough to maintain frequency_hz
        rc_usleep(1000000 / frequency_hz);

        if (rotationsCompleted >= numRotations * 4) break;

    }
}

void robot_turn_cw(double degrees, int frequency_hz) {
    double reqRotations = degrees / 2.0 * (1.5 / 180.0);
    //(degrees - degrees*(12.0/180.0))/2.0 *(1.5/180.0);
    //degrees/2.0 *(1.5/180.0);
    double servo_pos = 0;
    double direction = 1;    // switches between 1 &-1 in sweep mode
    double sweep_limit = 1.5 * direction;
    double net_servo_pos = 0;
    //int rotationsCompleted = 0;

    // Main loop runs at frequency_hz
    int rotationsCompleted = 0;
    while (running) {
        // scale with frequency
        servo_pos += direction * sweep_limit / frequency_hz;
        // reset pulse width at end of sweep

        net_servo_pos = servo_pos + rotationsCompleted * 1.5;
        printf("%f: %f\n", net_servo_pos * (180 / 1.5) * 2, servo_pos * (180 / 1.5) * 2);
        if (servo_pos > sweep_limit) {
            servo_pos = sweep_limit;
            rotationsCompleted++;
            //direction = -1;
        }

        /**
          if(servo_pos < (-sweep_limit)){
          servo_pos = -sweep_limit;
          direction = 1;
          }

                    **/
        // send result

        //ch = 7;
        if (rc_servo_send_pulse_normalized(7, servo_pos / 2.8) == -1) return -1;

        //ch = 8;
        if (rc_servo_send_pulse_normalized(8, servo_pos / 2.8) == -1) return -1;

        // sleep roughly enough to maintain frequency_hz
        rc_usleep(1000000 / frequency_hz);

        if (net_servo_pos > reqRotations * 2.8) break;

    }
}

void robot_move_cup_down(int frequency_hz) {
    double direction = 1.0;
    int counter = 0;
    double degrees = 100;
    double reqRotations = degrees / 2.0 * (1.5 / 180.0);
    //(degrees - degrees*(12.0/180.0))/2.0 *(1.5/180.0);
    //degrees/2.0 *(1.5/180.0);
    double servo_pos = 0;
    //double direction = 1;	// switches between 1 &-1 in sweep mode
    double sweep_limit = 1.5 * direction;
    double net_servo_pos = 0;
    //int rotationsCompleted = 0;

    // Main loop runs at frequency_hz
    int rotationsCompleted = 0;
    while (running) {
        // scale with frequency
        servo_pos += direction * sweep_limit / frequency_hz;
        // reset pulse width at end of sweep

        net_servo_pos = (servo_pos + direction * rotationsCompleted * 1.5);
        printf("%f: %f\n", net_servo_pos * (180 / 1.5) * 2, servo_pos * (180 / 1.5) * 2);
        if (servo_pos > sweep_limit) {
            servo_pos = sweep_limit;
            rotationsCompleted++;
            //direction = -1;
        }

        /**
          if(servo_pos < (-sweep_limit)){
          servo_pos = -sweep_limit;
          direction = 1;
          }

                    **/
        // send result

        //ch = 7;
        if (rc_servo_send_pulse_normalized(6, servo_pos / 4) == -1) return -1;
        counter++;

        //ch = 8;
        //if(rc_servo_send_pulse_normalized(8,servo_pos/2.8)==-1) return -1;

        // sleep roughly enough to maintain frequency_hz
        rc_usleep(1000000 / frequency_hz);

        if (counter > 39) break;

    }
}

void robot_move_cup_up(int frequency_hz) {
    double direction = -1.0;
    int counter = 0;
    double degrees = 100;
    double reqRotations = degrees / 2.0 * (1.5 / 180.0);
    //(degrees - degrees*(12.0/180.0))/2.0 *(1.5/180.0);
    //degrees/2.0 *(1.5/180.0);
    double servo_pos = 0;
    //double direction = 1;	// switches between 1 &-1 in sweep mode
    double sweep_limit = 1.5 * direction;
    double net_servo_pos = 0;
    //int rotationsCompleted = 0;

    // Main loop runs at frequency_hz
    int rotationsCompleted = 0;
    while (running) {
        // scale with frequency
        servo_pos += direction * sweep_limit / frequency_hz;
        // reset pulse width at end of sweep

        net_servo_pos = (servo_pos + direction * rotationsCompleted * 1.5);
        printf("%f: %f\n", net_servo_pos * (180 / 1.5) * 2, servo_pos * (180 / 1.5) * 2);
        if (servo_pos > sweep_limit) {
            servo_pos = sweep_limit;
            rotationsCompleted++;
            //direction = -1;
        }

        /**
          if(servo_pos < (-sweep_limit)){
          servo_pos = -sweep_limit;
          direction = 1;
          }

                    **/
        // send result

        //ch = 7;
        if (rc_servo_send_pulse_normalized(6, servo_pos / 4) == -1) return -1;
        counter++;

        //ch = 8;
        //if(rc_servo_send_pulse_normalized(8,servo_pos/2.8)==-1) return -1;

        // sleep roughly enough to maintain frequency_hz
        rc_usleep(1000000 / frequency_hz);

        if (counter > 30) break;

    }
}

double distance_measurement_left() {
    int counterRise = 0;
    int counterFall = 0;
    double distance;
    uint64_t *timeOccuredRise = malloc(sizeof(uint64_t));
    uint64_t *timeOccuredFall = malloc(sizeof(uint64_t));

    rc_gpio_set_value(1, 17, 0);
    rc_usleep(2);
    rc_gpio_set_value(1, 17, 1);
    rc_usleep(10);
    rc_gpio_set_value(1, 17, 0);

    while (!(rc_gpio_poll(1, 25, 100, timeOccuredRise) ==
             1))    //while the rising edge on echo hasn't occured yet, wait
    {
        counterRise++;
        if (counterRise > 3)
            break;
    }

    //if echo pin has rising edge, record time occurred

    while (!(rc_gpio_poll(1, 25, 100, timeOccuredFall) == 2)) {
        counterFall++;
        if (counterFall > 3)
            break;
    }    //while the falling edge on echo hasn't occured yet, wait
    uint64_t pulseStart = *timeOccuredRise;    //risingEdge
    uint64_t pulseEnd = *timeOccuredFall;    //falling edge

    uint64_t pulseDuration = pulseEnd - pulseStart;
    distance = (double) pulseDuration * 343 * 10e-8 * 0.5;
    if (distance < 9)
        distance *= 2;
    printf("\ndistance value = %f cm\n", distance);
    free(timeOccuredRise);
    free(timeOccuredFall);
    timeOccuredRise = NULL;
    timeOccuredFall = NULL;
    return distance;

}

double distance_measurement_right() {
    int counterRise = 0;
    int counterFall = 0;
    double distance;
    uint64_t *timeOccuredRise = malloc(sizeof(uint64_t));
    uint64_t *timeOccuredFall = malloc(sizeof(uint64_t));

    rc_gpio_set_value(3, 17, 0);
    rc_usleep(2);
    rc_gpio_set_value(3, 17, 1);
    rc_usleep(10);
    rc_gpio_set_value(3, 17, 0);

    while (!(rc_gpio_poll(3, 20, 100, timeOccuredRise) ==
             1))    //while the rising edge on echo hasn't occured yet, wait
    {
        counterRise++;
        if (counterRise > 3)
            break;
    }

    //if echo pin has rising edge, record time occurred

    while (!(rc_gpio_poll(3, 20, 100, timeOccuredFall) == 2)) {
        counterFall++;
        if (counterFall > 3)
            break;
    }    //while the falling edge on echo hasn't occured yet, wait
    uint64_t pulseStart = *timeOccuredRise;    //risingEdge
    uint64_t pulseEnd = *timeOccuredFall;    //falling edge

    uint64_t pulseDuration = pulseEnd - pulseStart;
    distance = (double) pulseDuration * 343 * 10e-8 * 0.5;
    if (distance < 9)
        distance *= 2;
    printf("\ndistance value = %f cm\n", distance);
    free(timeOccuredRise);
    free(timeOccuredFall);
    timeOccuredRise = NULL;
    timeOccuredFall = NULL;
    return distance;

}

double colour_sensor_red(int OUT) {

    double timeElapsed = 0.0;
    double freq = 0.0;

    //READ READ S2 = L, S3 = L
    rc_gpio_set_value(2, CS_S2, 0); //set S2 L
    rc_gpio_set_value(2, CS_S3, 0); //set S3 L
    uint64_t *timeRiseDummy = malloc(sizeof(uint64_t));

    uint64_t timeRise1 = 0;
    uint64_t timeRise2 = 0;

    uint64_t *timeRise1Ptr = malloc(sizeof(uint64_t));
    uint64_t *timeFall1Ptr = malloc(sizeof(uint64_t));

    while (!rc_gpio_poll(3, OUT, 10000, timeRise1Ptr) == 1);
    while (!rc_gpio_poll(3, OUT, 10000, timeFall1Ptr) == 2);

    freq = (-1e9 / ((double) *timeRise1Ptr - (double) *timeFall1Ptr)) * 2.0;
    return freq;
    //frequency = (1/((timeElapsed)*(5)*10e-9));
    //printf("time elapsed %"PRIu64 "\n", timeElapsed);
    //printf("%f  ", freq);
    // interrupt handler to catch ctrl-c
}

double colour_sensor_green(int OUT) {

    double timeElapsed = 0.0;
    double freq = 0.0;

    //READ READ S2 = L, S3 = L
    rc_gpio_set_value(2, CS_S2, 1); //set S2 L
    rc_gpio_set_value(2, CS_S3, 1); //set S3 L
    uint64_t *timeRiseDummy = malloc(sizeof(uint64_t));

    uint64_t timeRise1 = 0;
    uint64_t timeRise2 = 0;

    uint64_t *timeRise1Ptr = malloc(sizeof(uint64_t));
    uint64_t *timeFall1Ptr = malloc(sizeof(uint64_t));

    while (!rc_gpio_poll(3, OUT, 10000, timeRise1Ptr) == 1);
    while (!rc_gpio_poll(3, OUT, 10000, timeFall1Ptr) == 2);

    freq = (-1e9 / ((double) *timeRise1Ptr - (double) *timeFall1Ptr)) * 2.0;
    return freq;
    //frequency = (1/((timeElapsed)*(5)*10e-9));
    //printf("time elapsed %"PRIu64 "\n", timeElapsed);
    //printf("%f  ", freq);
    // interrupt handler to catch ctrl-c
}

double colour_sensor_blue(int OUT) {

    double timeElapsed = 0.0;
    double freq = 0.0;

    //READ READ S2 = L, S3 = L
    rc_gpio_set_value(2, CS_S2, 0); //set S2 L
    rc_gpio_set_value(2, CS_S3, 1); //set S3 L
    uint64_t *timeRiseDummy = malloc(sizeof(uint64_t));

    uint64_t timeRise1 = 0;
    uint64_t timeRise2 = 0;

    uint64_t *timeRise1Ptr = malloc(sizeof(uint64_t));
    uint64_t *timeFall1Ptr = malloc(sizeof(uint64_t));

    while (!rc_gpio_poll(3, OUT, 10000, timeRise1Ptr) == 1);
    while (!rc_gpio_poll(3, OUT, 10000, timeFall1Ptr) == 2);

    freq = (-1e9 / ((double) *timeRise1Ptr - (double) *timeFall1Ptr)) * 2.0;
    return freq;
    //frequency = (1/((timeElapsed)*(5)*10e-9));
    //printf("time elapsed %"PRIu64 "\n", timeElapsed);
    //printf("%f  ", freq);
    // interrupt handler to catch ctrl-c
}

static void __signal_handler(__attribute__((unused)) int dummy) {
    running = 0;
    return;
}

int main() {
    //printf("hello world\n");
    double servo_pos = 0;
    double sweep_limit = 1.5;
    uint64_t dsm_nanos = 0;
    int width_us = 0;
    int ch = 0;    // channel to test, 0 means all channels
    double direction = 1;    // switches between 1 &-1 in sweep mode
    //test_mode_t mode = DISABLED;	//start mode disabled
    int frequency_hz = 50;    // default 50hz frequency to send pulses
    int i, c, radio_ch;

    //initialize gpio
    rc_gpio_init_event(1, 25, 0, GPIOEVENT_REQUEST_BOTH_EDGES);    //echo - yellow
    rc_gpio_init(1, 17, GPIOHANDLE_REQUEST_OUTPUT);    //trigger - green
    rc_gpio_init_event(3, 20, 0, GPIOEVENT_REQUEST_BOTH_EDGES);    //echo - yellow
    rc_gpio_init(3, 17, GPIOHANDLE_REQUEST_OUTPUT);    //trigger - green

    ///initialize gpio for Ultrasonic
    //rc_gpio_init_event(1, 25, 0, GPIOEVENT_REQUEST_BOTH_EDGES); //echo - yellow (US 1)
    //rc_gpio_init(1, 17, GPIOHANDLE_REQUEST_OUTPUT); //trigger - green (US 2)

    //initialize gpio for Colour sensors
    rc_gpio_init(2, CS_S2, GPIOHANDLE_REQUEST_OUTPUT); //CS, S2
    rc_gpio_init(2, CS_S3, GPIOHANDLE_REQUEST_OUTPUT); //CS, S3
    rc_gpio_init_event(3, CS_OUT1, 0, GPIOEVENT_REQUEST_RISING_EDGE); //CS1, get output
    rc_gpio_init_event(3, CS_OUT2, 0, GPIOEVENT_REQUEST_RISING_EDGE); //CS2, get output

    //uint64_t *timeOccured = malloc(sizeof(uint64_t));
    //printf("\ngpio init\n");
    //rc_gpio_init(1,25,GPIOHANDLE_REQUEST_BOTH_EDGES);
    //rc_gpio_init(1,17,GPIOHANDLE_REQUEST_INPUT);

    // initialize PRU
    if (rc_servo_init()) return -1;

    // turn on power
    ///printf("Turning On 6V Servo Power Rail\n");
    rc_servo_power_rail_en(1);
    // print out what the program is doing
    //printf("\n");
    //if (ch == 0) printf("Sending on all channels.\n");

    //printf("Sweeping servos back/forth between +-%f\n", sweep_limit);
    //printf("Pulse Frequency: %d\n", frequency_hz);

    running = 1;

    //robot_turn_cw(180,frequency_hz);
    //robot_forward(100,frequency_hz);
    //rc_usleep(100e6);
    //robot_turn_cw(90,frequency_hz);

    //CUP UP AND DOWN:
    //robot_move_cup_down(frequency_hz);
    /**
  int counter = 0;
  while(counter < 250) {
    rc_usleep(10000);
    counter++;
  } //pause
  **/
    //robot_move_cup_up(frequency_hz);

    //ROBOT FORWARDS AND BACK:

    //robot_forward(20,frequency_hz);
    /**
    int counter = 0;
  while(counter < 50) {
    rc_usleep(10000);
    counter++;
  } //pause
    //robot_back(20,frequency_hz);


    bool forward;

    servo_pos = 0;

    while (1)
    {
        forward = true;
        servo_pos += direction *sweep_limit / frequency_hz;
        if (servo_pos > sweep_limit)
        {
            servo_pos = sweep_limit;
            //rotationsCompleted++;
            //direction = -1;
        }

        //ch = 7;
        if (rc_servo_send_pulse_normalized(7, -servo_pos / 3.35) == -1) return -1;

        //ch = 8;
        if (rc_servo_send_pulse_normalized(8, servo_pos / 4) == -1) return -1;

        // sleep roughly enough to maintain frequency_hz
        rc_usleep(1000000 / frequency_hz);

        //rc_usleep(500000);
        double object_dist_left = distance_measurement_left();
        double object_dist_right = distance_measurement_right();
        if (object_dist_left < 2.0)
        {
            //rc_servo_power_rail_en(7);	//cut servo power to stop
            // rc_servo_power_rail_en(8);	//cut servo power to stop
      /**
      int counter = 0;
      while(counter < 100) {
      rc_usleep(10000);
      counter++;
      } //pause
      **/
    //robot_turn_cw(90, frequency_hz);	//turn 90deg
    //		break;
    //	}

//		if (object_dist_right < 2.0)
//		{
    /**
    int counter = 0;
    while(counter < 100) {
    rc_usleep(10000);
    counter++;
    } //pause
    **/
    //rc_servo_power_rail_en(7);	//cut servo power to stop
    // rc_servo_power_rail_en(8);	//cut servo power to stop
    //robot_turn_cw(90, frequency_hz);	//turn 90deg
//			break;
//		}
//	}

    //robot_move_cup_up(frequency_hz);
    /**
    counter=0;
    while(counter < 100) {
        rc_usleep(10000);
        counter++;
        } //pause
    robot_turn_cw(180,frequency_hz);
    counter = 0;
        while(counter < 100) {
        rc_usleep(10000);
        counter++;
        } //pause
    robot_turn_cw(180,frequency_hz);
     **/
    printf("line-following-test\n");

    double leftC_sense = 0.0;
    double rightC_sense = 0.0;
    direction = 1;
    bool forward;
    double left_corr_factor = 0.0;
    double right_corr_factor = 0.0;
    double left_servo_kp = 0.0;
    double right_servo_kp = 0.0;
    sweep_limit = 1.5;
    double corr_factor = 0.0;
    servo_pos = 0;
    double max_speed = 10.0;
    double gain = 10;
    double r_wheel_gain = 1.4;
    while (1) {
        //printf("hello world\n");
        //colour_sensor(CS_OUT1);//left sensor
        //colour_sensor(CS_OUT2);//right sensor
        leftC_sense = colour_sensor_red(CS_OUT1) + colour_sensor_green(CS_OUT1) + colour_sensor_blue(CS_OUT1);
        rightC_sense = colour_sensor_red(CS_OUT2) + colour_sensor_green(CS_OUT2) + colour_sensor_blue(CS_OUT2);

        //right_corr_factor = leftC_sense / (leftC_sense + rightC_sense);
        //left_corr_factor = rightC_sense / (leftC_sense + rightC_sense);
        right_corr_factor = gain*abs(rightC_sense - leftC_sense)/(leftC_sense + rightC_sense);
        left_corr_factor = gain*abs(leftC_sense - rightC_sense)/(leftC_sense + rightC_sense);

        printf("R %f L %f\n", rightC_sense, leftC_sense);
        //printf("%f    %f\n",colour_sensor_red(CS_OUT1)+colour_sensor_green(CS_OUT1)+colour_sensor_blue(CS_OUT1), colour_sensor_red(CS_OUT2)+colour_sensor_green(CS_OUT2)+colour_sensor_blue(CS_OUT2));
        servo_pos += direction * sweep_limit / frequency_hz;

        if (servo_pos > sweep_limit) {
            servo_pos = sweep_limit;
            //rotationsCompleted++;
            //direction = -1;
        }

        if (rightC_sense > leftC_sense + 350) {
            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -servo_pos *7*1/4* r_wheel_gain/max_speed);

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, servo_pos * .5/4/max_speed);
        }
        else if (leftC_sense > rightC_sense + 350) {
            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -servo_pos * .5/4* r_wheel_gain/max_speed);

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, servo_pos * 7/4 * 1/max_speed);

        }
        else if (rightC_sense > leftC_sense + 250) {
            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -servo_pos * 6/4*1* r_wheel_gain/max_speed);

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, servo_pos * .8/4/max_speed);
        }
        else if (leftC_sense > rightC_sense + 250) {
            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -servo_pos * .8/4* r_wheel_gain/max_speed);

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, servo_pos * 6/4 * 1/max_speed);

        }
//        else if(rightC_sense > leftC_sense + 100) {
//            //ch = 7; right servo
//            rc_servo_send_pulse_normalized(7, -servo_pos * 1/4 * 1.1 * r_wheel_gain / max_speed);
//
//            //ch = 8; left servo
//            rc_servo_send_pulse_normalized(8, servo_pos * 1/4 / max_speed);
//        }
//        else if (leftC_sense > rightC_sense + 100) {
//            //ch = 7; right servo
//            rc_servo_send_pulse_normalized(7, -servo_pos * 1/4* r_wheel_gain/max_speed);
//
//            //ch = 8; left servo
//            rc_servo_send_pulse_normalized(8, servo_pos * 1 * 1.1/4/max_speed);
//
//        }
        else {
            //ch = 7; right servo
            rc_servo_send_pulse_normalized(7, -servo_pos * 1/1.0* r_wheel_gain/max_speed);

            //ch = 8; left servo
            rc_servo_send_pulse_normalized(8, servo_pos * 1/1.0/max_speed);
        }


        /**
        //ch = 7; right servo
        rc_servo_send_pulse_normalized(7, -servo_pos * r_wheel_gain/max_speed);

        //ch = 8; left servo
        rc_servo_send_pulse_normalized(8, servo_pos * 1/max_speed);
        **/

        // sleep roughly enough to maintain frequency_hz
        rc_usleep(1000000 / frequency_hz);



        //rc_usleep(50000/2);
    }
    //rc_usleep(50000);
    //robot_turn_cw(180,frequency_hz);
    //robot_back(100,frequency_hz);
    //rc_usleep(50000);
    // turn off power rail and cleanup
    rc_servo_power_rail_en(0);



    rc_servo_cleanup();
    rc_dsm_cleanup();
    return 0;
}
