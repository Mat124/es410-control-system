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

#include "esp_adc/adc_oneshot.h"

#include "esp_timer.h"

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
#define ULTRASONIC_TRIG_IO_PIN GPIO_NUM_5
#define ULTRASONIC_ECHO_IO_PIN GPIO_NUM_2

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
    int batTempReading, batVoltageReading;
    float batTemp = 0;
    float batVoltage = 0;
    float ultrasonicDistance = 0;
    mpu6050_acce_value_t acceValues;
    mpu6050_gyro_value_t gyroValues;

    while (1){
        // read battery sensors
        adc_oneshot_read(adc1, batVoltageChannel, &batVoltageReading);
        adc_oneshot_read(adc1, batTempChannel, &batTempReading);
        batVoltage = 7.666666666 * (batVoltageReading * 2.2) / 4095; // reads between 0.1v to 1.8v, voltage divider 1/11.5
        batTemp = 3.3 * ((float)batTempReading / 4095.0) * 100.0; // in kelvin
        // printf("direct readings: %d, %d\n", batVoltageReading, batTempReading);
        // printf("read battery sensors: %f, %f\n", batVoltage, batTemp);

        // battery overheat or low voltage, stop motors
        if (batTemp >= 330 || batVoltage < 11) {
            rightMotorSpeed = 0;
            leftMotorSpeed = 0;
            weaponMotorSpeed = 0;
        }

        // read ultrasonic sensor
        int64_t pulseStart, pulseEnd = 0;
        pulseStart = esp_timer_get_time();
        gpio_set_level(ULTRASONIC_TRIG_IO_PIN, 1);
        vTaskDelay(20 / portTICK_PERIOD_MS); // 20ms
        gpio_set_level(ULTRASONIC_TRIG_IO_PIN, 0);
        while (gpio_get_level(ULTRASONIC_ECHO_IO_PIN) == 0 && pulseEnd - pulseStart < 100000) {
            pulseEnd = esp_timer_get_time();
        }
        ultrasonicDistance = ((pulseEnd - pulseStart) * 1000000 * 340) / 2; // distance in m
        // printf("read ultrasonic sensor: %f\n", ultrasonicDistance);

        // read MPU6050
        mpu6050_get_acce(mpu6050dev, &acceValues);
        mpu6050_get_gyro(mpu6050dev, &gyroValues);
        // printf("read MPU6050: %f, %f, %f, %f, %f, %f\n", acceValues.acce_x, acceValues.acce_y, acceValues.acce_z, gyroValues.gyro_x, gyroValues.gyro_y, gyroValues.gyro_z);

        // if autonomous weapon mode, check ultrasonic distance < 10cm and activate weapon motor (?)

        // create random sensor data for testing
        if (TEST) {
            batTemp = (float)(rand() % 100);
            batVoltage = (float)(rand() % 100);
            ultrasonicDistance = (float)(rand() % 100);
        }

        // write to lineBuffer
        xSemaphoreTake(lineBufferMutex, portMAX_DELAY);
        sprintf((char *)lineBuffer, "T%f V%f D%f X%f Y%f Z%f x%f y%f z%f", 
                                batTemp, batVoltage, 
                                ultrasonicDistance, 
                                acceValues.acce_x, acceValues.acce_y, acceValues.acce_z, 
                                gyroValues.gyro_x, gyroValues.gyro_y, gyroValues.gyro_z); //TODO: include other sensor data
        xSemaphoreGive(lineBufferMutex);

        // request bluetooth send
        if (rfcomm_channel_id){
            rfcomm_request_can_send_now_event(rfcomm_channel_id);
        }
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

void setup() {
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
        .atten = ADC_ATTEN_DB_12,
    };
    printf("ADC initialized\n");
    adc_unit_t adcUnit;
    adc_oneshot_io_to_channel(BAT_TEMP_IO_PIN, &adcUnit, &batTempChannel);
    adc_oneshot_io_to_channel(BAT_VOLTAGE_IO_PIN, &adcUnit, &batVoltageChannel);
    printf("Converted to channel\n");
    adc_oneshot_config_channel(adc1, batVoltageChannel, &adc1ChannelCfg);
    adc_oneshot_config_channel(adc1, batTempChannel, &adc1ChannelCfg2);
    printf("Setup channel\n");

    gpio_set_direction(ULTRASONIC_TRIG_IO_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ULTRASONIC_ECHO_IO_PIN, GPIO_MODE_INPUT);
    printf("GPIO set directions\n");

    // init MPU6050
    i2c_bus_init();
    printf("Setup i2c bus\n");
    mpu6050dev = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    mpu6050_config(mpu6050dev, ACCE_FS_8G, GYRO_FS_1000DPS);
    mpu6050_wake_up(mpu6050dev);
    printf("MPU6050 initialised\n");

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

    // Setup pins, devices
    setup();

    // Start other tasks (reading sensor data, update motor outputs)
    xTaskCreate(motor_task, "MotorTask", 2048, NULL, 5, &xMotorTaskHandle);
    xTaskCreate(sensor_task, "SensorTask", 2048, NULL, 5, &xSensorTaskHandle);

    // Enter run loop
    btstack_run_loop_execute();

    return 0;
}
