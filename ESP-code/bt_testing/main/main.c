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
#include "bt_testing.c"

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

#define RIGHT_MOTOR_PIN 0 // TODO: set correct pin
#define LEFT_MOTOR_PIN 0
#define WEAPON_MOTOR_PIN 0
#define LED_PIN 2

extern int btstack_main(int argc, const char * argv[]);

extern char lineBuffer[1024]; // 1024 bytes

TaskHandle_t xMotorTaskHandle = NULL;
TaskHandle_t xSensorTaskHandle = NULL;

extern SemaphoreHandle_t lineBufferMutex;

extern uint16_t rfcomm_channel_id;

float rightMotorSpeed = 0;
float leftMotorSpeed = 0;
float weaponMotorSpeed = 0;

// sensor task
void sensor_task(void *pvParameters){
    float batTemp = 0;
    float batVoltage = 0;
    while (1){
        // create random sensor data for testing
        batTemp = (float)(rand() % 100);
        batVoltage = (float)(rand() % 100);

        // write to lineBuffer
        xSemaphoreTake(lineBufferMutex, portMAX_DELAY);
        sprintf((char *)lineBuffer, "batTemp:%f batVoltage:%f", batTemp, batVoltage);
        xSemaphoreGive(lineBufferMutex);

        // request bluetooth send
        if (rfcomm_channel_id){
            rfcomm_request_can_send_now_event(rfcomm_channel_id);
        }

        // wait 3 seconds
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

// motor task
void motor_task(void *pvParameters){
    while (1){
        // update motor outputs (currently only LED)
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, (int)(1024*rightMotorSpeed));
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

        // suspend task (activate on new data)
        vTaskSuspend(NULL);
    }
}

int app_main(void){

    // Enable buffered stdout
    btstack_stdio_init();

    // Configure BTstack for ESP32 VHCI Controller
    btstack_init();

    // Setup Bluetooth
    btstack_main(0, NULL);

    // Setup other pins
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 5000,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };

    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = LED_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    };

    ledc_channel_config(&ledc_channel);

    // Start other tasks (reading sensor data, update motor outputs)
    xTaskCreate(motor_task, "MotorTask", 2048, NULL, 5, &xMotorTaskHandle);

    // Enter run loop (forever)
    btstack_run_loop_execute();

    return 0;
}
