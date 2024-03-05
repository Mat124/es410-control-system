#include <stddef.h>
#include <stdio.h>

#include "btstack_port_esp32.h"
#include "btstack_run_loop.h"
#include "btstack_stdio_esp32.h"
#include "hci_dump.h"
#include "hci_dump_embedded_stdout.h"

#include "bt_handler.c"

// include FreeRTOS headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// include driver libraries
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_adc/adc_oneshot.h"

#include "mpu6050.h"

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

#define TEST 0

#define RIGHT_MOTOR_PIN 25
#define RIGHT_MOTOR_DIR_PIN GPIO_NUM_26
#define LEFT_MOTOR_PIN 27
#define LEFT_MOTOR_DIR_PIN GPIO_NUM_9
#define WEAPON_MOTOR_PIN 10
#define WEAPON_MOTOR_DIR_PIN GPIO_NUM_13
#define LED_PIN 2

#define LED_CHANNEL LEDC_CHANNEL_0
#define RIGHT_MOTOR_CHANNEL LEDC_CHANNEL_1
#define LEFT_MOTOR_CHANNEL LEDC_CHANNEL_2
#define WEAPON_MOTOR_CHANNEL LEDC_CHANNEL_3

#define BAT_VOLTAGE_IO_PIN 39
#define BAT_TEMP_IO_PIN 36
#define ULTRASONIC_TRIG_IO_PIN GPIO_NUM_34
#define ULTRASONIC_ECHO_IO_PIN 35

#define VOLTAGE 'V'
#define TEMPERATURE 'T'
#define X_ACCEL 'X'
#define Y_ACCEL 'Y'
#define Z_ACCEL 'Z'
#define X_GYRO 'x'
#define Y_GYRO 'y'
#define Z_GYRO 'z'

extern int btstack_main(int argc, const char * argv[]);

extern char lineBuffer[1024]; // 1024 bytes

TaskHandle_t xMotorTaskHandle = NULL;
TaskHandle_t xSensorTaskHandle = NULL;

adc_oneshot_unit_handle_t adc1;
adc_channel_t batVoltageChannel, batTempChannel, ultrasonicEchoChannel;

static mpu6050_handle_t mpu6050dev;

extern SemaphoreHandle_t lineBufferMutex;

extern uint16_t rfcomm_channel_id;

float ledBrightness = 0;
float rightMotorSpeed = 0;
float leftMotorSpeed = 0;
float weaponMotorSpeed = 0;

// sensor task
void sensor_task(void *pvParameters){
    int batTempReading, batVoltageReading, ultrasonicReading;
    float batTemp = 0;
    float batVoltage = 0;
    while (1){
        // read battery sensors
        adc_oneshot_read(adc1, )
        batVoltage = 12 * (adc_oneshot_read(BAT_VOLTAGE_CHANNEL) * (1.8 - 0.1)) / 4095; // reads between 0.1v to 1.8v, voltage divider 1/12
        batTemp = 3 * (adc1_get_raw(BAT_TEMP_CHANNEL) * (1.8 - 1)) / 4095;

        // read ultrasonic sensor
        gpio_set_level(ULTRASONIC_TRIG_IO_PIN, 1);
        vTaskDelay(0.01 / portTICK_PERIOD_MS); // 10us
        gpio_set_level(ULTRASONIC_TRIG_IO_PIN, 0);
        int64_t pulse_start, pulse_end;
        while (adc1_get_raw(ULTRASONIC_ECHO_CHANNEL) == 0) {
            pulse_start = esp_timer_get_time();
        }

        
        
        // create random sensor data for testing
        if (TEST) {
            batTemp = (float)(rand() % 100);
            batVoltage = (float)(rand() % 100);
        }

        // write to lineBuffer
        xSemaphoreTake(lineBufferMutex, portMAX_DELAY);
        sprintf((char *)lineBuffer, "T%f V%f", batTemp, batVoltage); //TODO: include other sensor data
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
void motor_task(void *pvParameters) {
    while (1){
        // update motor outputs (currently only LED)
        // LED
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LED_CHANNEL, (int)(1024*ledBrightness));
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LED_CHANNEL);

        // RIGHT MOTOR
        if (rightMotorSpeed < 0.0) {
            gpio_set_level(RIGHT_MOTOR_DIR_PIN, 0);
        }
        else {
            gpio_set_level(RIGHT_MOTOR_DIR_PIN, 1);
        }
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, RIGHT_MOTOR_CHANNEL, abs((int)(1024*rightMotorSpeed)));
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, RIGHT_MOTOR_CHANNEL);

        // LEFT MOTOR
        if (leftMotorSpeed < 0.0) {
            gpio_set_level(LEFT_MOTOR_DIR_PIN, 0);
        }
        else {
            gpio_set_level(LEFT_MOTOR_DIR_PIN, 1);
        }
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEFT_MOTOR_CHANNEL, abs((int)(1024*leftMotorSpeed)));
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEFT_MOTOR_CHANNEL);

        // WEAPON MOTOR
        if (weaponMotorSpeed < 0.0) {
            gpio_set_level(WEAPON_MOTOR_DIR_PIN, 0);
        }
        else {
            gpio_set_level(WEAPON_MOTOR_DIR_PIN, 1);
        }
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, WEAPON_MOTOR_CHANNEL, abs((int)(1024*weaponMotorSpeed)));
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, WEAPON_MOTOR_CHANNEL);

        // log motor outputs
        printf("LED: %f, RIGHT: %f, LEFT: %f, WEAPON: %f\n", ledBrightness, rightMotorSpeed, leftMotorSpeed, weaponMotorSpeed);

        // suspend task (activate on new data)
        vTaskSuspend(NULL);
    }
}

void pin_setup() {
    // ADC
    adc_oneshot_unit_init_cfg_t adc1UnitCfg = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&adc1UnitCfg, &adc1);
    adc_oneshot_chan_cfg_t adc1ChannelCfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_6,
    };
    adc_oneshot_chan_cfg_t adc1ChannelCfg2 = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_0,
    };
    adc_oneshot_io_to_channel(BAT_VOLTAGE_IO_PIN, ADC_UNIT_1, &batVoltageChannel);
    adc_oneshot_io_to_channel(BAT_TEMP_IO_PIN, ADC_UNIT_1, &batTempChannel);
    adc_oneshot_io_to_channel(ULTRASONIC_ECHO_IO_PIN, ADC_UNIT_1, &ultrasonicEchoChannel);
    adc_oneshot_config_channel(adc1, batVoltageChannel, &adc1ChannelCfg);
    adc_oneshot_config_channel(adc1, batTempChannel, &adc1ChannelCfg);
    adc_oneshot_config_channel(adc1, ultrasonicEchoChannel, &adc1ChannelCfg2);

    gpio_set_direction(ULTRASONIC_TRIG_IO_PIN, GPIO_MODE_OUTPUT);

    // init MPU6050
    mpu6050dev = mpu6050_init();

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
    gpio_set_direction(RIGHT_MOTOR_DIR_PIN, GPIO_MODE_OUTPUT);
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
    // Init mutex
    lineBufferMutex = xSemaphoreCreateMutex();

    // BT setup
    btstack_stdio_init();
    btstack_init();
    btstack_main(0, NULL);

    // Setup other pins
    pin_setup();

    // Start other tasks (reading sensor data, update motor outputs)
    xTaskCreate(motor_task, "MotorTask", 2048, NULL, 5, &xMotorTaskHandle);
    xTaskCreate(sensor_task, "SensorTask", 2048, NULL, 5, &xSensorTaskHandle);

    // Enter run loop
    btstack_run_loop_execute();

    return 0;
}
