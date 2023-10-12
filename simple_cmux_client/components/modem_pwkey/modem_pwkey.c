


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "modem_pwkey.h"


#include "esp_log.h"
const static char * const TAG = "modem_pwkey";


#define SIMCOM_PWRKEY_PIN 11
#define SIMCOM_STATUS_PIN 10


#define GPIO_INPUT_STATUS  ((gpio_num_t)SIMCOM_STATUS_PIN)
#define GPIO_INPUT_PIN_SEL (1ULL<<GPIO_INPUT_STATUS)

#define GPIO_OUTPUT_PWRKEY  ((gpio_num_t)SIMCOM_PWRKEY_PIN)
#define GPIO_OUTPUT_PIN_SEL (1ULL<<GPIO_OUTPUT_PWRKEY)


void init_modem_pwkey(void)
{

    gpio_config_t io_conf_in = {};                     //zero-initialize the config structure.

    io_conf_in.intr_type = GPIO_INTR_DISABLE;          //disable interrupt
    io_conf_in.mode = GPIO_MODE_INPUT;                //set as output mode
    io_conf_in.pin_bit_mask = GPIO_INPUT_PIN_SEL;       //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf_in.pull_down_en = GPIO_PULLDOWN_DISABLE;   //disable pull-down mode
    io_conf_in.pull_up_en = GPIO_PULLUP_ENABLE;       //disable pull-up mode

    gpio_config(&io_conf_in);                          //configure GPIO with the given settings


    gpio_config_t io_conf_out = {};                     //zero-initialize the config structure.

    io_conf_out.intr_type = GPIO_INTR_DISABLE;          //disable interrupt
    io_conf_out.mode = GPIO_MODE_OUTPUT;                //set as output mode
    io_conf_out.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;       //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf_out.pull_down_en = GPIO_PULLDOWN_DISABLE;   //disable pull-down mode
    io_conf_out.pull_up_en = GPIO_PULLUP_DISABLE;       //disable pull-up mode

    gpio_config(&io_conf_out);                          //configure GPIO with the given settings

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
    gpio_set_level(GPIO_OUTPUT_PWRKEY, 1); // switch on for 1s.
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "POWER ON OK");
    gpio_set_level(GPIO_OUTPUT_PWRKEY, 0); // switch on done
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
    gpio_set_level(GPIO_OUTPUT_PWRKEY, 1); // switch on for 1s.
    vTaskDelay(3500 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "POWER OFF OK");
    gpio_set_level(GPIO_OUTPUT_PWRKEY, 0); // switch on done
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
	gpio_set_level(GPIO_OUTPUT_PWRKEY, 1); // switch on for 1s.
	vTaskDelay(14000 / portTICK_PERIOD_MS);
	ESP_LOGI(TAG, "RESET ON OK");
	gpio_set_level(GPIO_OUTPUT_PWRKEY, 0); // switch on done


	vTaskDelay(1000 / portTICK_PERIOD_MS);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	status = !gpio_get_level(SIMCOM_STATUS_PIN);

	ESP_LOGI(TAG, "status = %i", status);







}


















