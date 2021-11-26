//
// Created by jacob on 2021-11-19.
//

#ifndef SWIRLE_SWIRLELIB_H
#define SWIRLE_SWIRLELIB_H
#define POWER "5V"
#define TRIGGER_PORT 1
#define TRIGGER_PIN 25
#define ECHO_PORT 1
#define ECHO_PIN 17
#define GND "0V"

//define colour sensor pins
#define CS_S2 2 //GPIO2_2, LED RED, GREEN
#define CS_S3 3 //GPIO2_3, LED GREEN, WHITE
#define CS_OUT1 1//GPIO3_2, PURPLE -> left colour sensor -> 2 prev
#define CS_OUT2 2//GPIO3_1, BLU -> right colour sensor //swap -> 1

#define WINDOW 15 //10 accepted

//colour detection thresholds
#define TURN_THRESHOLD 0.1
#define SLEEP_TIME 100000
#define DOUBLE_RED_THRESHOLD 50
#define LEFT_WOOD_RED 1300
//#define LEFT_RED_LINE 870
//#define LEFT_RED_LINE 750
#define LEFT_RED_LINE 6000
#define RIGHT_WOOD_RED 1050
//#define RIGHT_RED_LINE 750
//#define RIGHT_RED_LINE 870
#define RIGHT_RED_LINE 6000
//#define BLUE_THRESHOLD 500 //for the course
#define BLUE_THRESHOLD 6000 //for the course
//#define BLUE_THRESHOLD 625
#define GREEN_SEE_BLUE_THRESHOLD 800


//#define BULLSEYE_LOOP_CTR 5000
#define BULLSEYE_LOOP_CTR 10000
//blue low, green higher than baseline (phat margin) -> blue

#define COLOUR_TIMEOUT 50

static int running = 1;

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
        if (rc_servo_send_pulse_normalized(7, -servo_pos / 10) == -1) return -1;

        //ch = 8;
        if (rc_servo_send_pulse_normalized(8, servo_pos / 10) == -1) return -1;

        // sleep roughly enough to maintain frequency_hz
        rc_usleep(1000000 / frequency_hz);

        if (rotationsCompleted >= numRotations * 10) break;

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
        if (rc_servo_send_pulse_normalized(7, -servo_pos*0.1) == -1) return -1;

        //ch = 8;
        if (rc_servo_send_pulse_normalized(8, servo_pos*0.1*1.35) == -1) return -1;

        // sleep roughly enough to maintain frequency_hz
        rc_usleep(1000000 / frequency_hz);

        if (rotationsCompleted >= numRotations * 10) break;

    }
}

int robot_turn_cw(double degrees, int frequency_hz) {
    double reqRotations = degrees / 2.0 * (1.5 / 180.0);
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
    return 1;
}

int robot_move_cup_down(int frequency_hz) {
    double direction = 1.0;
    int counter = 0;
    double net_servo_pos = 0.0;
    double degrees = 100;
    double reqRotations = degrees / 2.0 * (1.5 / 180.0);
    double servo_pos = 0;
    //double direction = 1;	// switches between 1 &-1 in sweep mode
    double sweep_limit = 1.5 * direction;

    // Main loop runs at frequency_hz
    int rotationsCompleted = 0;
    while (running) {
        // scale with frequency
        servo_pos += direction * sweep_limit / frequency_hz;
        // reset pulse width at end of sweep

        net_servo_pos = (servo_pos + direction * rotationsCompleted * 1.5);
        //printf("%f: %f\n", net_servo_pos * (180 / 1.5) * 2, servo_pos * (180 / 1.5) * 2);
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

        //ch = 6; cup servo
        if (rc_servo_send_pulse_normalized(6, servo_pos / 4) == -1) return -1;
        counter++;

        // sleep roughly enough to maintain frequency_hz
        rc_usleep(1000000 / frequency_hz);

        if (counter > 40) break;

    }
    return 1;
}

int robot_move_cup_up(int frequency_hz) {
    double direction = -1.0;
    int counter = 0;
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
        //printf("%f: %f\n", net_servo_pos * (180 / 1.5) * 2, servo_pos * (180 / 1.5) * 2);
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

        if (counter > 35) break;

    }
    return 1;
}
int robot_turn_one_eighty(int frequency_hz, double direction) {
    //double direction = 1.0;
    int counter = 0;
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
        //printf("%f: %f\n", net_servo_pos * (180 / 1.5) * 2, servo_pos * (180 / 1.5) * 2);
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
        if (rc_servo_send_pulse_normalized(7, servo_pos*.1) == -1) return -1;
        counter++;

        //ch = 8;
        if(rc_servo_send_pulse_normalized(8, servo_pos*.1)==-1) return -1;

        // sleep roughly enough to maintain frequency_hz
        rc_usleep(1000000 / frequency_hz);

        if (counter > 135) break;

    }
    return 1;
}
int robot_turn_ninety_cw(int frequency_hz) {
    double direction = 1.0;
    int counter = 0;
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
        //printf("%f: %f\n", net_servo_pos * (180 / 1.5) * 2, servo_pos * (180 / 1.5) * 2);
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
        if (rc_servo_send_pulse_normalized(7, servo_pos*.1) == -1) return -1;
        counter++;

        //ch = 8;
        if(rc_servo_send_pulse_normalized(8, servo_pos*.1)==-1) return -1;

        // sleep roughly enough to maintain frequency_hz
        rc_usleep(1000000 / frequency_hz);

        if (counter > 80) break;

    }
    return 1;
}
int robot_turn_ninety_ccw(int frequency_hz) {
    double direction = -1.0;
    int counter = 0;
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
        //printf("%f: %f\n", net_servo_pos * (180 / 1.5) * 2, servo_pos * (180 / 1.5) * 2);
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
        if (rc_servo_send_pulse_normalized(7, servo_pos*.1) == -1) return -1;
        counter++;

        //ch = 8;
        if(rc_servo_send_pulse_normalized(8, servo_pos*.1)==-1) return -1;

        // sleep roughly enough to maintain frequency_hz
        rc_usleep(1000000 / frequency_hz);

        if (counter > 95) break;

    }
    return 1;
}
int robot_move_forward_bullseye(int frequency_hz) {
    double direction = 1.0;
    int counter = 0;
    double servo_pos = 0;
    //double direction = 1;	// switches between 1 &-1 in sweep mode
    double sweep_limit = 1.5 * direction;
    double net_servo_pos = 0;
    //int rotationsCompleted = 0;
    double max_speed = 0.1*1.0; //base speed of swirlE
    // - works decently at 0.08*1.25 and window size 3
    //safe speed is 0.08

    double r_wheel_gain = 1.35; //1.35 when full battery 1.5 when under 50%

    // Main loop runs at frequency_hz
    int rotationsCompleted = 0;
    while (running) {
        servo_pos += direction * sweep_limit / frequency_hz;

        if (servo_pos > sweep_limit) {
            servo_pos = sweep_limit;
        }

        //ch = 7;
        if (rc_servo_send_pulse_normalized(7, servo_pos*.1*1.35) == -1) return -1;

        //ch = 8;
        if(rc_servo_send_pulse_normalized(8,-servo_pos*.1)==-1) return -1;

        // sleep roughly enough to maintain frequency_hz
        rc_usleep(1000000 / frequency_hz);
        counter++;
        if (counter > 5) break;//35/10.0

    }
    return 1;
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
    //printf("\ndistance value = %f cm\n", distance);
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
    //printf("\ndistance value = %f cm\n", distance);
    free(timeOccuredRise);
    free(timeOccuredFall);
    timeOccuredRise = NULL;
    timeOccuredFall = NULL;
    return distance;

}

double colour_sensor_red(int OUT) {
    double freq = 0.0;
    int counterRise = 0;
    //printf("about to set gpio values\n");
    //READ READ S2 = L, S3 = L
    rc_gpio_set_value(2, CS_S2, 0); //set S2 L
    rc_gpio_set_value(2, CS_S3, 0); //set S3 L
    //printf("set values successfully\n");
    uint64_t *timeRise1Ptr = malloc(sizeof(uint64_t));
    uint64_t *timeFall1Ptr = malloc(sizeof(uint64_t));
    //printf("pointers allocated\n");
    while (!(rc_gpio_poll(3, OUT, COLOUR_TIMEOUT, timeRise1Ptr) == 1)) {
        counterRise++;
        //printf("%f\n",counterRise);
        if (counterRise > 3) {
            counterRise = 0;
            break;
        }

    }
    counterRise = 0;
    while (!(rc_gpio_poll(3, OUT, COLOUR_TIMEOUT, timeFall1Ptr) == 1)) {
        counterRise++;
        //printf("%f\n",counterRise);
        if (counterRise > 3) {
            counterRise = 0;
            break;
        }
    }

    freq = (-1e9 / ((double) *timeRise1Ptr - (double) *timeFall1Ptr)) * 2.0;
    free(timeRise1Ptr);
    timeRise1Ptr = NULL;
    free(timeFall1Ptr);
    timeFall1Ptr = NULL;
    //printf("about to return\n");
    return freq;
}

double colour_sensor_green(int OUT) {
    double freq = 0.0;
    int counterRise = 0;
    //READ READ S2 = L, S3 = L
    rc_gpio_set_value(2, CS_S2, 1); //set S2 L
    rc_gpio_set_value(2, CS_S3, 1); //set S3 L

    uint64_t *timeRise1Ptr = malloc(sizeof(uint64_t));
    uint64_t *timeFall1Ptr = malloc(sizeof(uint64_t));

    while (!(rc_gpio_poll(3, OUT, COLOUR_TIMEOUT, timeRise1Ptr) == 1)) {
        counterRise++;
        if (counterRise > 3) {
            counterRise = 0;
            break;
        }
    }
    counterRise = 0;
    while (!(rc_gpio_poll(3, OUT, COLOUR_TIMEOUT, timeFall1Ptr) == 1)){
        counterRise++;
        if (counterRise > 3) {
            counterRise = 0;
            break;
        }
    }

    freq = (-1e9 / ((double) *timeRise1Ptr - (double) *timeFall1Ptr)) * 2.0;
    free(timeRise1Ptr);
    timeRise1Ptr = NULL;
    free(timeFall1Ptr);
    timeFall1Ptr = NULL;
    return freq;

}

double colour_sensor_blue(int OUT) {
    double freq = 0.0;
    int counterRise = 0;
    //READ READ S2 = L, S3 = L
    rc_gpio_set_value(2, CS_S2, 0); //set S2 L
    rc_gpio_set_value(2, CS_S3, 1); //set S3 L

    uint64_t *timeRise1Ptr = malloc(sizeof(uint64_t));
    uint64_t *timeFall1Ptr = malloc(sizeof(uint64_t));

    while (!(rc_gpio_poll(3, OUT, COLOUR_TIMEOUT, timeRise1Ptr) == 1)){
        counterRise++;
        if (counterRise > 3) {
            counterRise = 0;
            break;
        }
    }
    counterRise = 0;
    while (!(rc_gpio_poll(3, OUT, COLOUR_TIMEOUT, timeFall1Ptr) == 1)){
        counterRise++;
        if (counterRise > 3) {
            counterRise = 0;
            break;
        }
    }

    freq = (-1e9 / ((double) *timeRise1Ptr - (double) *timeFall1Ptr)) * 2.0;
    free(timeRise1Ptr);
    timeRise1Ptr = NULL;
    free(timeFall1Ptr);
    timeFall1Ptr = NULL;
    return freq;
}

static void __signal_handler(__attribute__((unused)) int dummy) {
    running = 0;
    return;
}

void robot_gpio_init() {
    //initialize gpio for Ultrasonic
    rc_gpio_init_event(1, 25, 0, GPIOEVENT_REQUEST_BOTH_EDGES);    //echo - yellow
    rc_gpio_init(1, 17, GPIOHANDLE_REQUEST_OUTPUT);    //trigger - green
    rc_gpio_init_event(3, 20, 0, GPIOEVENT_REQUEST_BOTH_EDGES);    //echo - yellow
    rc_gpio_init(3, 17, GPIOHANDLE_REQUEST_OUTPUT);    //trigger - green

    //initialize gpio for Colour sensors
    rc_gpio_init(2, CS_S2, GPIOHANDLE_REQUEST_OUTPUT); //CS, S2
    rc_gpio_init(2, CS_S3, GPIOHANDLE_REQUEST_OUTPUT); //CS, S3
    rc_gpio_init_event(3, CS_OUT1, 0, GPIOEVENT_REQUEST_RISING_EDGE); //CS1, get output
    rc_gpio_init_event(3, CS_OUT2, 0, GPIOEVENT_REQUEST_RISING_EDGE); //CS2, get output

}

bool is_green_detected(int OUTLeft, int OUTRight) {
    if ((colour_sensor_red(OUTLeft) + colour_sensor_blue(OUTLeft)) / 2 > (colour_sensor_green(OUTLeft) + 800)) {
        return true;
    }
    if ((colour_sensor_red(OUTRight) + colour_sensor_blue(OUTRight)) / 2 > (colour_sensor_green(OUTRight) + 800)) {
        return true;
    }
    return false;
}

bool is_blue_detected(int OUTLeft, int OUTRight) {
    if ((colour_sensor_red(OUTLeft) + colour_sensor_green(OUTLeft)) / 2 > (colour_sensor_blue(OUTLeft) + 1500)) {
        return true;
    }
    if ((colour_sensor_red(OUTRight) + colour_sensor_green(OUTRight)) / 2 > (colour_sensor_blue(OUTRight) + 1500)) {
        return true;
    }
    return false;
}
double rolling_avg(double *readings, double *sum, int *index, double raw_val) {
    double value = 0.0;
    double average = 0.0;
    *sum = *sum - readings[*index];       // Remove the oldest entry from the sum
    value = raw_val;       // Read the next sensor value
    readings[*index] = value;           // Add the newest reading to the window
    *sum = *sum + value;                 // Add the newest reading to the sum
    *index = (*index+1) % WINDOW;   // Increment the index, and wrap to 0 if it exceeds the window size

    average = *sum / WINDOW;      // Divide the sum of the window by the window size for the result
    return average;
}

#endif //SWIRLE_SWIRLELIB_H
