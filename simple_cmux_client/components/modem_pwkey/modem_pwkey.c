


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "modem_pwkey.h"


#include "esp_log.h"
const static char * const TAG = "modem_pwkey";


#define SIMCOM_PWRKEY_PIN 11
#define SIMCOM_STATUS_PIN 10




void init_modem_pwkey(void)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_reset_pin(SIMCOM_PWRKEY_PIN);
    gpio_reset_pin(SIMCOM_STATUS_PIN);
    gpio_pullup_en(SIMCOM_STATUS_PIN);

    /* Set the GPIO as a push/pull output */
    gpio_set_direction(SIMCOM_PWRKEY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SIMCOM_PWRKEY_PIN, 0); // switch off first.


    /* Set the GPIO as a push/pull output */
    gpio_set_direction(SIMCOM_STATUS_PIN, GPIO_MODE_INPUT);




    //
    // It is recommended to ensure that the VBAT voltage rises and stabilizes before pulling downthePWRKEY pin to start up.

    vTaskDelay(2000 / portTICK_PERIOD_MS);

}

//This is a BLOCKING Version.
//with PLC-Lib a non-Blocking Version is possible as well.

void power_up_modem_pwkey(void)
{
	//First we must read-in the Status:
	int status;
	status = !gpio_get_level(SIMCOM_STATUS_PIN);

	ESP_LOGI(TAG, "status = %i", status);

	//if we already on, we are done here:
	if(status) return;

	//else
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "POWER ON");
    gpio_set_level(SIMCOM_PWRKEY_PIN, 1); // switch on for 1s.
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "POWER ON OK");
    gpio_set_level(SIMCOM_PWRKEY_PIN, 0); // switch on done
    do {
    	status = !gpio_get_level(SIMCOM_STATUS_PIN);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    	ESP_LOGI(TAG, "status = %i", status);
	} while (!status);



}

void power_down_modem_pwkey(void)
{
	//First we must read-in the Status:
	int status;
	status = !gpio_get_level(SIMCOM_STATUS_PIN);

	ESP_LOGI(TAG, "status = %i", status);

	//if we already off, we are done here:
	if(!status) return;

	//else
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "POWER OFF");
    gpio_set_level(SIMCOM_PWRKEY_PIN, 1); // switch on for 1s.
    vTaskDelay(3500 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "POWER OFF OK");
    gpio_set_level(SIMCOM_PWRKEY_PIN, 0); // switch on done
    do {
    	status = !gpio_get_level(SIMCOM_STATUS_PIN);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    	ESP_LOGI(TAG, "status = %i", status);
	} while (status);



}



void power_reset_modem_pwkey(void)
{
	//After the PWRKEY continues to pull
	//down more than 12.6S, the system will
	//automatically reset.


	//First we must read-in the Status:
	int status;
	status = !gpio_get_level(SIMCOM_STATUS_PIN);

	ESP_LOGI(TAG, "status = %i", status);


	//else
	vTaskDelay(100 / portTICK_PERIOD_MS);

	ESP_LOGI(TAG, "RESET ON");
	gpio_set_level(SIMCOM_PWRKEY_PIN, 1); // switch on for 1s.
	vTaskDelay(14000 / portTICK_PERIOD_MS);
	ESP_LOGI(TAG, "RESET ON OK");
	gpio_set_level(SIMCOM_PWRKEY_PIN, 0); // switch on done


	vTaskDelay(1000 / portTICK_PERIOD_MS);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	status = !gpio_get_level(SIMCOM_STATUS_PIN);

	ESP_LOGI(TAG, "status = %i", status);







}


















