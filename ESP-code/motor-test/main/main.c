#include <stddef.h>
#include <stdio.h>

// include FreeRTOS headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// include driver libraries
#include "driver/gpio.h"
#include "driver/ledc.h"

#define RIGHT_MOTOR_PIN 25
#define RIGHT_MOTOR_DIR_PIN 26
#define LEFT_MOTOR_PIN 27
#define LEFT_MOTOR_DIR_PIN 9
#define WEAPON_MOTOR_PIN 10
#define WEAPON_MOTOR_DIR_PIN 13
#define LED_PIN 2

#define LED_CHANNEL LEDC_CHANNEL_0
#define RIGHT_MOTOR_CHANNEL LEDC_CHANNEL_1
#define LEFT_MOTOR_CHANNEL LEDC_CHANNEL_2
#define WEAPON_MOTOR_CHANNEL LEDC_CHANNEL_3

float ledBrightness = 0;
float rightMotorSpeed = 0;
float leftMotorSpeed = 0;
float weaponMotorSpeed = 0;

// motor task
void motor_task() {
    // update motor outputs
    // LED
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LED_CHANNEL, (int)(1024*ledBrightness));
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LED_CHANNEL);

    // RIGHT MOTOR
    if (rightMotorSpeed < 0) {
        gpio_set_level(RIGHT_MOTOR_DIR_PIN, 0);
    }
    else {
        gpio_set_level(RIGHT_MOTOR_DIR_PIN, 1);
    }
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, RIGHT_MOTOR_CHANNEL, abs((int)(1024*rightMotorSpeed)));
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, RIGHT_MOTOR_CHANNEL);

    // LEFT MOTOR
    if (leftMotorSpeed < 0) {
        gpio_set_level(LEFT_MOTOR_DIR_PIN, 0);
    }
    else {
        gpio_set_level(LEFT_MOTOR_DIR_PIN, 1);
    }
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEFT_MOTOR_CHANNEL, abs((int)(1024*leftMotorSpeed)));
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEFT_MOTOR_CHANNEL);

    // WEAPON MOTOR
    if (weaponMotorSpeed < 0) {
        gpio_set_level(WEAPON_MOTOR_DIR_PIN, 0);
    }
    else {
        gpio_set_level(WEAPON_MOTOR_DIR_PIN, 1);
    }
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, WEAPON_MOTOR_CHANNEL, abs((int)(1024*weaponMotorSpeed)));
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, WEAPON_MOTOR_CHANNEL);

    // log motor outputs
    printf("LED: %f, RIGHT: %f, LEFT: %f, WEAPON: %f\n", ledBrightness, rightMotorSpeed, leftMotorSpeed, weaponMotorSpeed);
}

void pin_setup() {
    // timer configuration
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT, // 0-1023
        .freq_hz = 20000, // 20 kHz
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&ledc_timer);

    // LED PIN CHANNEL
    ledc_channel_config_t led_ledc_channel = {
        .channel = LED_CHANNEL,
        .duty = 0,
        .gpio_num = LED_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&led_ledc_channel);

    // RIGHT MOTOR PIN CHANNEL
    ledc_channel_config_t rmotor_ledc_channel = {
        .channel = RIGHT_MOTOR_CHANNEL,
        .duty = 0,
        .gpio_num = RIGHT_MOTOR_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&rmotor_ledc_channel);

    // LEFT MOTOR PIN CHANNEL
    ledc_channel_config_t lmotor_ledc_channel = {
        .channel = LEFT_MOTOR_CHANNEL,
        .duty = 0,
        .gpio_num = LEFT_MOTOR_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&lmotor_ledc_channel);

    // WEAPON MOTOR PIN CHANNEL
    ledc_channel_config_t wmotor_ledc_channel = {
        .channel = WEAPON_MOTOR_CHANNEL,
        .duty = 0,
        .gpio_num = WEAPON_MOTOR_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&wmotor_ledc_channel);
}

int app_main(void) {

    pin_setup();

    char line[1024];

    while (1) {

        line[0] = '\0';

        scanf("%1023s", line);

        switch(line[0]) {
            case 'Z':
                ledBrightness = atof((char *)line + 1);
                break;
            case 'R':
                rightMotorSpeed = atof((char* )line + 1);
                break;
            case 'L':
                leftMotorSpeed = atof((char *)line + 1);
                break;
            case 'W':
                weaponMotorSpeed = atof((char *)line + 1);
                break;
            default:
                break;
        }

        motor_task();
    }

    return 0;
}
