/*
 * Copyright (C) 2020 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/*
 *  main.c
 *
 *  Minimal main application that initializes BTstack, prepares the example and enters BTstack's Run Loop.
 *
 *  If needed, you can create other threads. Please note that BTstack's API is not thread-safe and can only be
 *  called from BTstack timers or in response to its callbacks, e.g. packet handlers.
 */

#include <stddef.h>
#include <stdio.h>

#include "btstack_port_esp32.h"
#include "btstack_run_loop.h"
#include "btstack_stdio_esp32.h"
#include "hci_dump.h"
#include "hci_dump_embedded_stdout.h"

// include FreeRTOS headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// include driver libraries
#include "driver/gpio.h"
#include "driver/ledc.h"

// warn about unsuitable sdkconfig
#include "sdkconfig.h"
#if !CONFIG_BT_ENABLED
#error "Bluetooth disabled - please set CONFIG_BT_ENABLED via menuconfig -> Component Config -> Bluetooth -> [x] Bluetooth"
#endif
#if !CONFIG_BT_CONTROLLER_ONLY
#error "Different Bluetooth Host stack selected - please set CONFIG_BT_CONTROLLER_ONLY via menuconfig -> Component Config -> Bluetooth -> Host -> Disabled"
#endif
#if ESP_IDF_VERSION_MAJOR >= 5
#if !CONFIG_BT_CONTROLLER_ENABLED
#error "Different Bluetooth Host stack selected - please set CONFIG_BT_CONTROLLER_ENABLED via menuconfig -> Component Config -> Bluetooth -> Controller -> Enabled"
#endif
#endif

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
