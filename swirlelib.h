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
#define CS_OUT1 2 //GPIO3_2, PURPLE -> left colour sensor
#define CS_OUT2 1 //GPIO3_1, BLU -> right colour sensor

static int running;

void robot_forward(int numRotations, int frequency_hz)
{
    double servo_pos = 0;
    double direction = 1;    // switches between 1 &-1 in sweep mode
    double sweep_limit = 1.5 * direction;

    // Main loop runs at frequency_hz
    int rotationsCompleted = 0;
    while (running)
    {
        // scale with frequency
        servo_pos += direction * sweep_limit / frequency_hz;
        // reset pulse width at end of sweep

        if (servo_pos > sweep_limit)
        {
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

void robot_back(int numRotations, int frequency_hz)
{
    double servo_pos = 0;
    double direction = -1;    // switches between 1 &-1 in sweep mode
    double sweep_limit = 1.5 * direction;

    // Main loop runs at frequency_hz
    int rotationsCompleted = 0;
    while (running)
    {
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
    while (running)
    {
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

void robot_move_cup_down(int frequency_hz)
{
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
    while (running)

    {
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

void robot_move_cup_up(int frequency_hz)
{
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

double distance_measurement_left()
{
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

    while (!(rc_gpio_poll(1, 25, 100, timeOccuredFall) == 2))
    {
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

double distance_measurement_right()
{
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

double colour_sensor_red(int OUT)
{

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

double colour_sensor_green(int OUT)
{

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

double colour_sensor_blue(int OUT)
{

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

static void __signal_handler(__attribute__((unused)) int dummy)
{
    running = 0;
    return;
}
void robot_gpio_init()
{
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
#endif //SWIRLE_SWIRLELIB_H
