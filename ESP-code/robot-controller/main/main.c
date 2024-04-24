#include <stddef.h>
#include <stdio.h>

// BTstack includes
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
#include "driver/i2c.h"
#include "driver/mcpwm_cap.h"

#include "esp_adc/adc_oneshot.h"

#include "esp_timer.h"

#include "mpu6050.h"

#include "esp_private/esp_clk.h"

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

// defining the pin numbers for future use
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
#define ULTRASONIC_TRIG_IO_PIN 5
#define ULTRASONIC_ECHO_IO_PIN 2

#define VOLTAGE 'V'
#define TEMPERATURE 'T'
#define X_ACCEL 'X'
#define Y_ACCEL 'Y'
#define Z_ACCEL 'Z'
#define X_GYRO 'x'
#define Y_GYRO 'y'
#define Z_GYRO 'z'

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

// declaring external functions and variables
extern int btstack_main(int argc, const char * argv[]);
extern char lineBuffer[1024]; // 1024 bytes
extern SemaphoreHandle_t lineBufferMutex;
extern uint16_t rfcomm_channel_id;

// global variables
TaskHandle_t xMotorTaskHandle = NULL;
TaskHandle_t xSensorTaskHandle = NULL;

adc_oneshot_unit_handle_t adc1;
adc_channel_t batVoltageChannel, batTempChannel, ultrasonicEchoChannel;

static mpu6050_handle_t mpu6050dev;

float ledBrightness = 0;
float rightMotorSpeed = 0;
float leftMotorSpeed = 0;
float weaponMotorSpeed = 0;

// motor task
void motor_task(void *pvParameters) {
    vTaskSuspend(NULL); // suspend task until explicitly started for the first time
    while (1){
        // on-board LED
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LED_CHANNEL, (int)(1024*ledBrightness));
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LED_CHANNEL);

        // RIGHT MOTOR
        // if speed is negative, set direction pin to 0 so motor spins in reverse
        if (rightMotorSpeed < 0.0) {
            gpio_set_level(RIGHT_MOTOR_DIR_PIN, 0);
        }
        else {
            gpio_set_level(RIGHT_MOTOR_DIR_PIN, 1);
        }
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, RIGHT_MOTOR_CHANNEL, abs((int)(1024*rightMotorSpeed)));
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, RIGHT_MOTOR_CHANNEL);

        // LEFT MOTOR
        // if speed is negative, set direction pin to 0 so motor spins in reverse
        if (leftMotorSpeed < 0.0) {
            gpio_set_level(LEFT_MOTOR_DIR_PIN, 0);
        }
        else {
            gpio_set_level(LEFT_MOTOR_DIR_PIN, 1);
        }
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEFT_MOTOR_CHANNEL, abs((int)(1024*leftMotorSpeed)));
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEFT_MOTOR_CHANNEL);

        // WEAPON MOTOR
        // if speed is negative, set direction pin to 0 so motor spins in reverse
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

        // suspend task (activated on new data), allows other tasks to run
        vTaskSuspend(NULL);
    }
}

// sensor_task
// This task reads sensor data and writes to lineBuffer every 20ms or so, and
// requests a bluetooth send. Battery overheating and low voltage is also checked 
// here, and currently stops the motors if either is detected.
void sensor_task(void *pvParameters){
    int batTempReading, batVoltageReading;
    float batTemp = 0;
    float batVoltage = 0;
    uint32_t tof_ticks;
    float ultrasonicDistance = 0;
    mpu6050_acce_value_t acceValues;
    mpu6050_gyro_value_t gyroValues;

    vTaskSuspend(NULL); // suspend task until explicitly started for the first time

    while (1){
        // read battery sensors
        adc_oneshot_read(adc1, batVoltageChannel, &batVoltageReading);
        adc_oneshot_read(adc1, batTempChannel, &batTempReading);
        // reads up to 2.2v, voltage divider gives 1.5/11.5 of the battery voltage
        batVoltage = 7.666666666 * (batVoltageReading * 2.2) / 4095; 
        // reads up to 3.3v, 10mV per degree kelvin and no voltage divider
        batTemp = 3.3 * ((float)batTempReading / 4095.0) * 100.0;

        // if battery overheat (>63c) or low voltage, stop motors
        if (batTemp >= 330 || batVoltage < 11) {
            rightMotorSpeed = 0;
            leftMotorSpeed = 0;
            weaponMotorSpeed = 0;
        }

        // read ultrasonic sensor
        // time (ticks) between sending and receiving the pulse is proportional to the distance
        gpio_set_level(ULTRASONIC_TRIG_IO_PIN, 1);
        vTaskDelay(20 / portTICK_PERIOD_MS); // 20ms delay for the pulse to be sent
        gpio_set_level(ULTRASONIC_TRIG_IO_PIN, 0);
        if (xTaskNotifyWait(0x00, ULONG_MAX, &tof_ticks, pdMS_TO_TICKS(1000)) == pdTRUE) {
            float pulse_width_us = tof_ticks * (1000000.0 / esp_clk_apb_freq());
            if (pulse_width_us < 35000) {
                ultrasonicDistance = (float) pulse_width_us / 5800;
            }
        }

        // read MPU6050 data into acceValues and gyroValues
        mpu6050_get_acce(mpu6050dev, &acceValues);
        mpu6050_get_gyro(mpu6050dev, &gyroValues);

        // create random sensor data for testing
        if (TEST) {
            batTemp = (float)(rand() % 100);
            batVoltage = (float)(rand() % 100);
            ultrasonicDistance = (float)(rand() % 100);
        }

        // write to lineBuffer
        // take mutex lock for threading safety - prevents lineBuffer from being written to by multiple tasks at once
        xSemaphoreTake(lineBufferMutex, portMAX_DELAY);
        sprintf((char *)lineBuffer, "T%f V%f D%f X%f Y%f Z%f x%f y%f z%f ", 
                                batTemp, batVoltage, 
                                ultrasonicDistance, 
                                acceValues.acce_x, acceValues.acce_y, acceValues.acce_z, 
                                gyroValues.gyro_x, gyroValues.gyro_y, gyroValues.gyro_z);
        xSemaphoreGive(lineBufferMutex);

        // request bluetooth send
        if (rfcomm_channel_id){
            rfcomm_request_can_send_now_event(rfcomm_channel_id);
        }
    }
}

// i2c_bus_init
// This function initializes the i2c bus for the MPU6050 sensor
void i2c_bus_init(void) {
    i2c_config_t conf;

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// ultrasonic sensor callback
static bool ultrasonic_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *event_data, void *sensor_task_handle)
{
    static uint32_t cap_val_begin_of_sample = 0;
    static uint32_t cap_val_end_of_sample = 0;
    TaskHandle_t task_to_notify = (TaskHandle_t)sensor_task_handle;
    BaseType_t high_task_wakeup = pdFALSE; // platform dependent

    if (event_data->cap_edge == MCPWM_CAP_EDGE_POS) {
        // store the timestamp when pos edge is detected
        cap_val_begin_of_sample = event_data->cap_value;
        cap_val_end_of_sample = cap_val_begin_of_sample;
    } else {
        cap_val_end_of_sample = event_data->cap_value;
        uint32_t tof_ticks = cap_val_end_of_sample - cap_val_begin_of_sample;

        // notify the task to calculate the distance
        xTaskNotifyFromISR(task_to_notify, tof_ticks, eSetValueWithOverwrite, &high_task_wakeup);
    }

    return high_task_wakeup == pdTRUE;
}

// setup
// This function initializes the ADC, GPIO, I2C, and LEDC drivers, and sets up the MPU6050 sensor
void setup() {
    // ADC
    adc_oneshot_unit_init_cfg_t adc1UnitCfg = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&adc1UnitCfg, &adc1);
    // attenuation of 6dB for battery voltage, 12dB for battery temperature
    // sets the voltage ranges to 0-2.2v and 0-3.3v respectively
    adc_oneshot_chan_cfg_t adc1ChannelCfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_6,
    };
    adc_oneshot_chan_cfg_t adc1ChannelCfg2 = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    printf("ADC initialized\n");
    adc_unit_t adcUnit;
    // convert the pins to ADC channels, required to read the data
    adc_oneshot_io_to_channel(BAT_TEMP_IO_PIN, &adcUnit, &batTempChannel);
    adc_oneshot_io_to_channel(BAT_VOLTAGE_IO_PIN, &adcUnit, &batVoltageChannel);
    printf("Converted to channel\n");
    // configure the channels to read data
    adc_oneshot_config_channel(adc1, batVoltageChannel, &adc1ChannelCfg);
    adc_oneshot_config_channel(adc1, batTempChannel, &adc1ChannelCfg2);
    printf("Setup channel\n");

    // init MPU6050
    i2c_bus_init();
    printf("Setup i2c bus\n");
    // create and configure the MPU6050 sensor
    // uses the new i2c bus we made
    mpu6050dev = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    // configure the sensor to use 8g for accelerometer and 1000dps for gyroscope
    mpu6050_config(mpu6050dev, ACCE_FS_8G, GYRO_FS_1000DPS);
    mpu6050_wake_up(mpu6050dev);
    printf("MPU6050 initialised\n");

    // timer configuration, common for all PWM signals
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT, // 0-1023 range
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
    gpio_set_direction(LEFT_MOTOR_DIR_PIN, GPIO_MODE_OUTPUT);
    ledc_channel_config_t lmotor_ledc_channel = {
        .channel = LEFT_MOTOR_CHANNEL,
        .duty = 0,
        .gpio_num = LEFT_MOTOR_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&lmotor_ledc_channel);

    // WEAPON MOTOR PIN CHANNEL
    gpio_set_direction(WEAPON_MOTOR_DIR_PIN, GPIO_MODE_OUTPUT);
    ledc_channel_config_t wmotor_ledc_channel = {
        .channel = WEAPON_MOTOR_CHANNEL,
        .duty = 0,
        .gpio_num = WEAPON_MOTOR_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&wmotor_ledc_channel);

    // SONAR SETUP
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 0,
    };
    mcpwm_new_capture_timer(&cap_conf, &cap_timer);

    mcpwm_cap_channel_handle_t cap_chan = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf = {
        .gpio_num = ULTRASONIC_ECHO_IO_PIN,
        .prescale = 1,
        .flags.neg_edge = true,
        .flags.pos_edge = true,
        .flags.pull_up = true,
    };
    mcpwm_new_capture_channel(cap_timer, &cap_ch_conf, &cap_chan);

    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = ultrasonic_callback,
    };
    mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, xSensorTaskHandle);
    mcpwm_capture_channel_enable(cap_chan);
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << ULTRASONIC_TRIG_IO_PIN,
    };
    gpio_config(&io_conf);
    // drive low by default
    gpio_set_level(ULTRASONIC_TRIG_IO_PIN, 0);

    mcpwm_capture_timer_enable(cap_timer);
    mcpwm_capture_timer_start(cap_timer);
}

int app_main(void) {
    // Create mutex for lineBuffer
    lineBufferMutex = xSemaphoreCreateMutex();

    // BT setup
    btstack_stdio_init();
    btstack_init();
    btstack_main(0, NULL);

    // Start other tasks (reading sensor data, update motor outputs)
    xTaskCreate(motor_task, "MotorTask", 2048, NULL, 5, &xMotorTaskHandle);
    xTaskCreate(sensor_task, "SensorTask", 2048, NULL, 5, &xSensorTaskHandle);

    // Setup pins, devices
    setup();

    // Start tasks
    vTaskResume(xMotorTaskHandle);
    vTaskResume(xSensorTaskHandle);

    // Enter run loop
    btstack_run_loop_execute();

    return 0;
}
