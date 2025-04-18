#include "driver/ledc.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// #include <stdio.h>
#include "unity.h"
// #include "driver/i2c.h"
#include "mpu6050.h"
#include "esp_system.h"
// #include "esp_log.h"
#include "linear_algebra.h"
#include "esp_sntp.h"
#include <inttypes.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "ssd1306.h"
#include "font8x8_basic.h"

/*
 You have to set this config value with menuconfig
 CONFIG_INTERFACE

 for i2c
 CONFIG_MODEL
 CONFIG_SDA_GPIO
 CONFIG_SCL_GPIO
 CONFIG_RESET_GPIO

 for SPI
 CONFIG_CS_GPIO
 CONFIG_DC_GPIO
 CONFIG_RESET_GPIO
*/

#define I2C_MASTER_SCL_IO 21	  /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 22	  /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_1  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */
#define MODE_BUTTON_GPIO 26
#define RESET_BUTTON_GPIO 27
#define HANDLE_SENSOR_GPIO 25
#define PIR_SENSOR_GPIO 15

static const char *TAG = "Grenade";

static mpu6050_handle_t mpu6050 = NULL;
SSD1306_t dev;

// 0 - Pull the safety pin off and let the handle move and after 3 seconds it will explode
// 1 - Explodes the moment it hits the ground
// 2 - PIR activated
int mode = 0;

// 0 - Normal
// 1 - Exploded (reset expected)
int state = 0;

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
	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
	//ssd1306_display_text_x3(&dev, 0, "BOOM!", 5, false);
	state = 1;

	gpio_set_level(18, 1);
	
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	gpio_set_level(18, 0);

	ssd1306_clear_screen(&dev, true);
	ssd1306_contrast(&dev, 0x00);
	ssd1306_display_text(&dev, 3, "Grenade V1.0", 14, false);
}

void setup_ignition() {
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = (1 << 18);
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
	gpio_config(&io_conf);
}

void setups_buttons()
{
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = (1 << MODE_BUTTON_GPIO) | (1 << RESET_BUTTON_GPIO) | (1 << HANDLE_SENSOR_GPIO);
	io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	gpio_config(&io_conf);
}

void count_down()
{
	ESP_LOGI(TAG, "Count down");

	ssd1306_clear_screen(&dev, true);
	ssd1306_contrast(&dev, 0xff);
	uint8_t image[24];

	for (int font = 0x33; font > 0x30; font--)
	{
		ESP_LOGI(TAG, "%d", font);
		memset(image, 0, sizeof(image));
		ssd1306_display_image(&dev, 1, (7 * 8 - 1), image, 8);
		memcpy(image, font8x8_basic_tr[font], 8);
		// if (dev._flip) ssd1306_flip(image, 8);
		ssd1306_display_image(&dev, 1, (7 * 8 - 1), image, 8);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	ESP_LOGI(TAG, "Count down complete");
}

void app_main(void)
{
	setups_buttons();
	setup_ignition();

#if 1
	int center, top, bottom;
	char lineChar[20];

#if CONFIG_I2C_INTERFACE
	ESP_LOGI(TAG, "INTERFACE is i2c");
	ESP_LOGI(TAG, "CONFIG_SDA_GPIO=%d", CONFIG_SDA_GPIO);
	ESP_LOGI(TAG, "CONFIG_SCL_GPIO=%d", CONFIG_SCL_GPIO);
	ESP_LOGI(TAG, "CONFIG_RESET_GPIO=%d", CONFIG_RESET_GPIO);
	i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
#endif // CONFIG_I2C_INTERFACE

#if CONFIG_FLIP
	dev._flip = true;
	ESP_LOGW(TAG, "Flip upside down");
#endif

#if CONFIG_SSD1306_128x64
	ESP_LOGI(TAG, "Panel is 128x64");
	ssd1306_init(&dev, 128, 64);
#endif // CONFIG_SSD1306_128x64
#if CONFIG_SSD1306_128x32
	ESP_LOGI(TAG, "Panel is 128x32");
	ssd1306_init(&dev, 128, 32);
#endif // CONFIG_SSD1306_128x32

	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
	ssd1306_display_text_x3(&dev, 0, "Hello", 5, false);
	vTaskDelay(1000 / portTICK_PERIOD_MS);

#if CONFIG_SSD1306_128x64
	top = 2;
	center = 3;
	bottom = 8;
	// ssd1306_display_text(&dev, 0, "SSD1306 128x64", 14, false);
	// ssd1306_display_text(&dev, 1, "ABCDEFGHIJKLMNOP", 16, false);
	// ssd1306_display_text(&dev, 2, "abcdefghijklmnop", 16, false);
	// ssd1306_display_text(&dev, 3, "Hello World!!", 13, false);
	// ssd1306_clear_line(&dev, 4, true);
	// ssd1306_clear_line(&dev, 5, true);
	// ssd1306_clear_line(&dev, 6, true);
	// ssd1306_clear_line(&dev, 7, true);
	// ssd1306_display_text(&dev, 4, "SSD1306 128x64", 14, true);
	// ssd1306_display_text(&dev, 5, "ABCDEFGHIJKLMNOP", 16, true);
	// ssd1306_display_text(&dev, 6, "abcdefghijklmnop", 16, true);
	// ssd1306_display_text(&dev, 7, "Hello World!!", 13, true);
#endif // CONFIG_SSD1306_128x64

#if CONFIG_SSD1306_128x32
	top = 1;
	center = 1;
	bottom = 4;
	ssd1306_display_text(&dev, 0, "SSD1306 128x32", 14, false);
	ssd1306_display_text(&dev, 1, "Hello World!!", 13, false);
	// ssd1306_clear_line(&dev, 2, true);
	// ssd1306_clear_line(&dev, 3, true);
	ssd1306_display_text(&dev, 2, "SSD1306 128x32", 14, true);
	ssd1306_display_text(&dev, 3, "Hello World!!", 13, true);
#endif // CONFIG_SSD1306_128x32
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	// Display Count Down
	// uint8_t image[24];
	// memset(image, 0, sizeof(image));
	// ssd1306_display_image(&dev, top, (6 * 8 - 1), image, sizeof(image));
	// ssd1306_display_image(&dev, top + 1, (6 * 8 - 1), image, sizeof(image));
	// ssd1306_display_image(&dev, top + 2, (6 * 8 - 1), image, sizeof(image));
	// for (int font = 0x34; font > 0x30; font--)
	// {
	// 	memset(image, 0, sizeof(image));
	// 	ssd1306_display_image(&dev, top + 1, (7 * 8 - 1), image, 8);
	// 	memcpy(image, font8x8_basic_tr[font], 8);
	// 	// if (dev._flip) ssd1306_flip(image, 8);
	// 	ssd1306_display_image(&dev, top + 1, (7 * 8 - 1), image, 8);
	// 	vTaskDelay(1000 / portTICK_PERIOD_MS);
	// }

	// Scroll Up
	// ssd1306_clear_screen(&dev, false);
	// ssd1306_contrast(&dev, 0xff);
	// ssd1306_display_text(&dev, 0, "---Scroll  UP---", 16, true);
	// //ssd1306_software_scroll(&dev, 7, 1);
	// ssd1306_software_scroll(&dev, (dev._pages - 1), 1);
	// for (int line=0;line<bottom+10;line++) {
	// 	lineChar[0] = 0x01;
	// 	sprintf(&lineChar[1], " Line %02d", line);
	// 	ssd1306_scroll_text(&dev, lineChar, strlen(lineChar), false);
	// 	vTaskDelay(500 / portTICK_PERIOD_MS);
	// }
	// vTaskDelay(3000 / portTICK_PERIOD_MS);

	// Scroll Down
	// ssd1306_clear_screen(&dev, false);
	// ssd1306_contrast(&dev, 0xff);
	// ssd1306_display_text(&dev, 0, "--Scroll  DOWN--", 16, true);
	// //ssd1306_software_scroll(&dev, 1, 7);
	// ssd1306_software_scroll(&dev, 1, (dev._pages - 1) );
	// for (int line=0;line<bottom+10;line++) {
	// 	lineChar[0] = 0x02;
	// 	sprintf(&lineChar[1], " Line %02d", line);
	// 	ssd1306_scroll_text(&dev, lineChar, strlen(lineChar), false);
	// 	vTaskDelay(500 / portTICK_PERIOD_MS);
	// }
	// vTaskDelay(3000 / portTICK_PERIOD_MS);

	// Page Down
	// ssd1306_clear_screen(&dev, false);
	// ssd1306_contrast(&dev, 0xff);
	// ssd1306_display_text(&dev, 0, "---Page	DOWN---", 16, true);
	// ssd1306_software_scroll(&dev, 1, (dev._pages-1) );
	// for (int line=0;line<bottom+10;line++) {
	// 	//if ( (line % 7) == 0) ssd1306_scroll_clear(&dev);
	// 	if ( (line % (dev._pages-1)) == 0) ssd1306_scroll_clear(&dev);
	// 	lineChar[0] = 0x02;
	// 	sprintf(&lineChar[1], " Line %02d", line);
	// 	ssd1306_scroll_text(&dev, lineChar, strlen(lineChar), false);
	// 	vTaskDelay(500 / portTICK_PERIOD_MS);
	// }
	// vTaskDelay(3000 / portTICK_PERIOD_MS);

	// Horizontal Scroll
	// ssd1306_clear_screen(&dev, false);
	// ssd1306_contrast(&dev, 0xff);
	// ssd1306_display_text(&dev, center, "Horizontal", 10, false);
	// ssd1306_hardware_scroll(&dev, SCROLL_RIGHT);
	// vTaskDelay(5000 / portTICK_PERIOD_MS);
	// ssd1306_hardware_scroll(&dev, SCROLL_LEFT);
	// vTaskDelay(5000 / portTICK_PERIOD_MS);
	// ssd1306_hardware_scroll(&dev, SCROLL_STOP);

	// Vertical Scroll
	// ssd1306_clear_screen(&dev, false);
	// ssd1306_contrast(&dev, 0xff);
	// ssd1306_display_text(&dev, center, "Vertical", 8, false);
	// ssd1306_hardware_scroll(&dev, SCROLL_DOWN);
	// vTaskDelay(2000 / portTICK_PERIOD_MS);
	// ssd1306_hardware_scroll(&dev, SCROLL_UP);
	// vTaskDelay(2000 / portTICK_PERIOD_MS);
	// ssd1306_hardware_scroll(&dev, SCROLL_STOP);

	// // Invert
	// ssd1306_clear_screen(&dev, true);
	// ssd1306_contrast(&dev, 0xff);
	// ssd1306_display_text(&dev, center, "  Good Bye!!", 12, true);
	// vTaskDelay(1000 / portTICK_PERIOD_MS);

	// Fade Out
	// ssd1306_fadeout(&dev);

#if 0
	// Fade Out
	for(int contrast=0xff;contrast>0;contrast=contrast-0x20) {
		ssd1306_contrast(&dev, contrast);
		vTaskDelay(40);
	}
#endif
#endif

	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
	ssd1306_display_text(&dev, center, "Grenade V1.0", 14, false);

	printf("Hi\n");

	mpu6050_init(&mpu6050, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);

	const float hit_theshold = 0.5;

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
		printf("loop\n");
		switch (mode)
		{
		case 0:
			if (gpio_get_level(HANDLE_SENSOR_GPIO) == 1)
			{
				count_down();
				explode();
			}
			break;
		case 1:
			// if (change > hit_theshold)
			// {
			// 	//explode();
			// }
		case 2:

		default:
			break;
		}

		if (gpio_get_level(MODE_BUTTON_GPIO) == 1) {
			mode = (mode + 1) % 3;
		}

		vTaskDelay(50 / portTICK_PERIOD_MS);
	}
}
