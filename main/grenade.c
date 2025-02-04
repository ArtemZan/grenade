#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include "unity.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "esp_system.h"
#include "esp_log.h"
#include "linear_algebra.h"
#include "esp_sntp.h"
#include <inttypes.h>
#include <math.h>

#define I2C_MASTER_SCL_IO 22      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static const char *TAG = "mpu6050 test";

static mpu6050_handle_t mpu6050 = NULL;

// 0 - Pull the safety pin off and let the handle move and after 3 seconds it will explode
// 1 - Explodes the moment it hits the ground
// 2 - PIR activated
int mode = 0;

void mpu6050_get_orientation(VectorFloat *const i, VectorFloat *const j, VectorFloat *const k)
{
    int16_t quaternion[4];

    mpu6050_dmp_get_quaternion(quaternion);

    Quaternion q = {
        (float)quaternion[0] / 16384.0f,
        (float)quaternion[1] / 16384.0f,
        (float)quaternion[2] / 16384.0f,
        (float)quaternion[3] / 16384.0f};

    i->x = 1;
    i->y = 0;
    i->z = 0;

    j->x = 0;
    j->y = 1;
    j->z = 0;

    k->x = 0;
    k->y = 0;
    k->z = 1;

    rotateWithQuaternion(i, &q);
    rotateWithQuaternion(j, &q);
    rotateWithQuaternion(k, &q);
}

void explode()
{
}

void setups_buttons()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1 << 4);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
}

void app_main()
{
    printf("Hi");

    mpu6050_init(&mpu6050, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);

    const hit_theshold = 0.5;

    VectorFloat prev_i;
    VectorFloat prev_j;
    VectorFloat prev_k;

    VectorFloat i;
    VectorFloat j;
    VectorFloat k;

    while (true)
    {
        mpu6050_get_current_FIFO_packet(mpu6050);
        mpu6050_get_orientation(&i, &j, &k);
        printf("(%f, %f, %f)\n", k.x, k.y, k.z);

        float change = sqrt(powf(getDistance(i, prev_i), 2) + powf(getDistance(j, prev_j), 2) + powf(getDistance(k, prev_k), 2));

        switch (mode)
        {
        case 0:
            if (gpio_get_level(4) == 1)
            {
                vTaskDelay(3000 / portTICK_PERIOD_MS);
                explode();
            }
            break;
        case 1:
            if (change > hit_theshold)
            {
                explode();
            }
        case 2:

        default:
            break;
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}