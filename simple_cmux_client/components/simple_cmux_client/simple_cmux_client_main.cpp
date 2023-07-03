/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* PPPoS Client Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <cstring>
#include <iostream>
#include "modem_pwkey.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_event.h"
#include "cxx_include/esp_modem_dte.hpp"
#include "esp_modem_config.h"
#include "cxx_include/esp_modem_api.hpp"
#include "simple_mqtt_client.hpp"
#include "esp_vfs_dev.h"        // For optional VFS support
#include "esp_https_ota.h"      // For potential OTA configuration
#include "vfs_resource/vfs_create.hpp"

#if defined(CONFIG_EXAMPLE_MODEM_DEVICE_SHINY)
    #include "shiny_module_dce.hpp"
#elif defined(CONFIG_EXAMPLE_MODEM_DEVICE_SIM7070_GNSS)
    #include "SIM7070_gnss.hpp"
#elif defined(CONFIG_EXAMPLE_MODEM_DEVICE_A7672_GNSS)
    #include "A7672_gnss.hpp"
#endif



#if defined(CONFIG_EXAMPLE_MODEM_DEVICE_SHINY) || defined(CONFIG_EXAMPLE_MODEM_DEVICE_SIM7070_GNSS) || defined(CONFIG_EXAMPLE_MODEM_DEVICE_A7672_GNSS)
    #define SUPPORT_URC_HANDLER 1
#endif

#if defined(CONFIG_EXAMPLE_FLOW_CONTROL_NONE)
#define EXAMPLE_FLOW_CONTROL ESP_MODEM_FLOW_CONTROL_NONE
#elif defined(CONFIG_EXAMPLE_FLOW_CONTROL_SW)
#define EXAMPLE_FLOW_CONTROL ESP_MODEM_FLOW_CONTROL_SW
#elif defined(CONFIG_EXAMPLE_FLOW_CONTROL_HW)
#define EXAMPLE_FLOW_CONTROL ESP_MODEM_FLOW_CONTROL_HW
#endif

#define BROKER_URL CONFIG_BROKER_URI


using namespace esp_modem;

static const char *TAG = "cmux_example";

class StatusHandler {
public:
    static constexpr auto IP_Event      = SignalGroup::bit0;
    static constexpr auto MQTT_Connect  = SignalGroup::bit1;
    static constexpr auto MQTT_Data     = SignalGroup::bit2;

    StatusHandler()
    {
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, on_event, this));
    }

    ~StatusHandler()
    {
        esp_event_handler_unregister(IP_EVENT, ESP_EVENT_ANY_ID, on_event);
    }

    void handle_mqtt(MqttClient *client)
    {
        mqtt_client = client;
        client->register_handler(ESP_EVENT_ANY_ID, on_event, this);
    }

    esp_err_t wait_for(decltype(IP_Event) event, int milliseconds)
    {
        return signal.wait_any(event, milliseconds);
    }

    ip_event_t get_ip_event_type()
    {
        return ip_event_type;
    }

private:
    static void on_event(void *arg, esp_event_base_t base, int32_t event, void *data)
    {
        auto *handler = static_cast<StatusHandler *>(arg);
        if (base == IP_EVENT) {
            handler->ip_event(event, data);
        } else {
            handler->mqtt_event(event, data);
        }
    }

    void ip_event(int32_t id, void *data)
    {
        if (id == IP_EVENT_PPP_GOT_IP) {
            auto *event = (ip_event_got_ip_t *)data;
            ESP_LOGI(TAG, "IP          : " IPSTR, IP2STR(&event->ip_info.ip));
            ESP_LOGI(TAG, "Netmask     : " IPSTR, IP2STR(&event->ip_info.netmask));
            ESP_LOGI(TAG, "Gateway     : " IPSTR, IP2STR(&event->ip_info.gw));
            signal.set(IP_Event);
        } else if (id == IP_EVENT_PPP_LOST_IP) {
            signal.set(IP_Event);
        }
        ip_event_type = static_cast<ip_event_t>(id);
    }

    void mqtt_event(int32_t event, void *data)
    {
        if (mqtt_client && event == mqtt_client->get_event(MqttClient::Event::CONNECT)) {
            signal.set(MQTT_Connect);
        } else if (mqtt_client && event == mqtt_client->get_event(MqttClient::Event::DATA)) {
            ESP_LOGI(TAG, " TOPIC: %s", mqtt_client->get_topic(data).c_str());
            ESP_LOGI(TAG, " DATA: %s", mqtt_client->get_data(data).c_str());
            signal.set(MQTT_Data);
        }
    }

    esp_modem::SignalGroup signal{};
    MqttClient *mqtt_client{nullptr};
    ip_event_t ip_event_type;
};



#ifdef SUPPORT_URC_HANDLER
command_result handle_urc(uint8_t *data, size_t len)
{
    ESP_LOG_BUFFER_HEXDUMP("on_read", data, len, ESP_LOG_INFO);
    return command_result::TIMEOUT;
}
#endif


extern "C" void simple_cmux_client_main(void)
{
    /* Init and register system/core components */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());

    /* Configure and create the DTE */
    esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    /* setup UART specific configuration based on kconfig options */
    dte_config.uart_config.tx_io_num = CONFIG_EXAMPLE_MODEM_UART_TX_PIN;
    dte_config.uart_config.rx_io_num = CONFIG_EXAMPLE_MODEM_UART_RX_PIN;
    dte_config.uart_config.rts_io_num = CONFIG_EXAMPLE_MODEM_UART_RTS_PIN;
    dte_config.uart_config.cts_io_num = CONFIG_EXAMPLE_MODEM_UART_CTS_PIN;
    dte_config.uart_config.flow_control = EXAMPLE_FLOW_CONTROL;
#if CONFIG_EXAMPLE_USE_VFS_TERM == 1
    /* The VFS terminal is just a demonstration of using an abstract file descriptor
     * which implements non-block reads, writes and selects to communicate with esp-modem.
     * This configuration uses the same UART driver as the terminal created by `create_uart_dte()`,
     * so doesn't give any practical benefit besides the FD use demonstration and a placeholder
     * to use FD terminal for other devices
     */
    struct esp_modem_vfs_uart_creator uart_config = ESP_MODEM_VFS_DEFAULT_UART_CONFIG("/dev/uart/1");
    assert(vfs_create_uart(&uart_config, &dte_config.vfs_config) == true);

    auto dte = create_vfs_dte(&dte_config);
    esp_vfs_dev_uart_use_driver(uart_config.uart.port_num);
#else
    auto dte = create_uart_dte(&dte_config);
#endif // CONFIG_EXAMPLE_USE_VFS_TERM
    assert(dte);

    /* Configure the DCE */
    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(CONFIG_EXAMPLE_MODEM_PPP_APN);

    /* Configure the PPP netif */
    esp_netif_config_t netif_ppp_config = ESP_NETIF_DEFAULT_PPP();

    /* Create the PPP and DCE objects */

    esp_netif_t *esp_netif = esp_netif_new(&netif_ppp_config);
    assert(esp_netif);

#if CONFIG_EXAMPLE_MODEM_DEVICE_SHINY == 1
    auto dce = create_shiny_dce(&dce_config, dte, esp_netif);
#elif CONFIG_EXAMPLE_MODEM_DEVICE_BG96 == 1
    auto dce = create_BG96_dce(&dce_config, dte, esp_netif);
#elif CONFIG_EXAMPLE_MODEM_DEVICE_SIM800 == 1
    auto dce = create_SIM800_dce(&dce_config, dte, esp_netif);
#elif CONFIG_EXAMPLE_MODEM_DEVICE_SIM7000 == 1
    auto dce = create_SIM7000_dce(&dce_config, dte, esp_netif);
#elif CONFIG_EXAMPLE_MODEM_DEVICE_SIM7070 == 1
    auto dce = create_SIM7070_dce(&dce_config, dte, esp_netif);
#elif CONFIG_EXAMPLE_MODEM_DEVICE_SIM7070_GNSS == 1
    auto dce = create_SIM7070_GNSS_dce(&dce_config, dte, esp_netif);
#elif CONFIG_EXAMPLE_MODEM_DEVICE_A7600 == 1
    auto dce = create_A7600_dce(&dce_config, dte, esp_netif);
#elif CONFIG_EXAMPLE_MODEM_DEVICE_A7672_GNSS == 1
    auto dce = create_A7672_GNSS_dce(&dce_config, dte, esp_netif);
#elif CONFIG_EXAMPLE_MODEM_DEVICE_SIM7600 == 1
    auto dce = create_SIM7600_dce(&dce_config, dte, esp_netif);
#else
#error "Unsupported device"
#endif
    assert(dce);

#ifdef SUPPORT_URC_HANDLER
    ESP_LOGI(TAG, "Adding URC handler");
    dce->set_on_read(handle_urc);
#endif


    if (dte_config.uart_config.flow_control == ESP_MODEM_FLOW_CONTROL_HW) {


        //set this mode also to the DCE.
        if (command_result::OK != dce->set_flow_control(2, 2)) {
            ESP_LOGE(TAG, "Failed to set the set_flow_control mode");
            return;
        }
        ESP_LOGI(TAG, "set_flow_control OK");


    } else {
        ESP_LOGI(TAG, "not set_flow_control, because 2-wire mode active.");
    }
    
    

    dce->sync();
    dce->sync();

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    dce->sync();

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    dce->sync();

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    dce->sync();
    dce->sync();
    dce->sync();

    /* Setup basic operation mode for the DCE (pin if used, CMUX mode) */
#if CONFIG_EXAMPLE_NEED_SIM_PIN == 1
    bool pin_ok = true;
    if (dce->read_pin(pin_ok) == command_result::OK && !pin_ok) {
        ESP_MODEM_THROW_IF_FALSE(dce->set_pin(CONFIG_EXAMPLE_SIM_PIN) == command_result::OK, "Cannot set PIN!");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Need to wait for some time after unlocking the SIM
    }
#endif
#ifdef SUPPORT_URC_HANDLER
    ESP_LOGI(TAG, "Removing URC handler");
    dce->set_on_read(nullptr);
#endif
    if (dce->set_mode(esp_modem::modem_mode::CMUX_MANUAL_MODE)) {
        std::cout << "Modem has correctly entered CMUX_MANUAL_MODE" << std::endl;
    } else {
        ESP_LOGE(TAG, "Failed to configure CMUX_MANUAL_MODE... exiting");
        return;
    }
    if (dce->set_mode(esp_modem::modem_mode::CMUX_MANUAL_DATA)) {
        std::cout << "Modem has correctly entered CMUX_MANUAL_DATA" << std::endl;
    } else {
        ESP_LOGE(TAG, "Failed to configure CMUX_MANUAL_DATA... exiting");
        return;
    }
#ifdef SUPPORT_URC_HANDLER
    ESP_LOGI(TAG, "Adding URC handler");
    dce->set_on_read(handle_urc);
#endif

    /* Read some data from the modem */
    std::string str;
    int a;
#ifndef CONFIG_EXAMPLE_MODEM_DEVICE_SHINY
    while (dce->get_operator_name(str, a) != esp_modem::command_result::OK) {
        // Getting operator name could fail... retry after 500 ms
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    std::cout << "Operator name:" << str << std::endl;
#endif

#if defined(CONFIG_EXAMPLE_MODEM_DEVICE_A7600) || defined(CONFIG_EXAMPLE_MODEM_DEVICE_A7672_GNSS)
    if (dce->set_gnss_power_mode(1) == esp_modem::command_result::OK) {
        std::cout << "Modem set_gnss_power_mode: OK" << std::endl;
    }
#endif

    /* Try to connect to the network and publish an mqtt topic */
    StatusHandler handler;
    if (!handler.wait_for(StatusHandler::IP_Event, 60000)) {
        ESP_LOGE(TAG, "Cannot get IP within specified timeout... exiting");
        return;
    } else if (handler.get_ip_event_type() == IP_EVENT_PPP_GOT_IP) {
        std::cout << "Got IP address" << std::endl;

        /* When connected to network, subscribe and publish some MQTT data */
        MqttClient mqtt(BROKER_URL);
        handler.handle_mqtt(&mqtt);
        mqtt.connect();
        if (!handler.wait_for(StatusHandler::MQTT_Connect, 60000)) {
            ESP_LOGE(TAG, "Cannot connect to %s within specified timeout... exiting", BROKER_URL);
            return;
        }
        std::cout << "Connected" << std::endl;

        mqtt.subscribe("/topic/esp-modem");
        mqtt.publish("/topic/esp-modem", "Hello modem");
        if (!handler.wait_for(StatusHandler::MQTT_Data, 60000)) {
            ESP_LOGE(TAG, "Didn't receive published data within specified timeout... exiting");
            return;
        }
        std::cout << "Received MQTT data" << std::endl;

    } else if (handler.get_ip_event_type() == IP_EVENT_PPP_LOST_IP) {
        ESP_LOGE(TAG, "PPP client has lost connection... exiting");
        return;
    }

    /* Again reading some data from the modem */
    if (dce->get_imsi(str) == esp_modem::command_result::OK) {
        std::cout << "Modem IMSI number:" << str << std::endl;
    }



#if CONFIG_EXAMPLE_MODEM_DEVICE_SIM7070_GNSS == 1
    esp_modem_gps_t gps;

    for (int i = 0; i < 200; ++i) {
        if (dce->get_gnss_information_sim70xx(gps) == esp_modem::command_result::OK) {
            ESP_LOGI(TAG, "gps.run  %i",
                     gps.run);
            ESP_LOGI(TAG, "gps.fix  %i",
                     gps.fix);
            ESP_LOGI(TAG, "gps.date.year %i gps.date.month %i gps.date.day %i",
                     gps.date.year,   gps.date.month,   gps.date.day);
            ESP_LOGI(TAG, "gps.tim.hour %i gps.tim.minute %i   gps.tim.second %i   gps.tim.thousand %i",
                     gps.tim.hour,   gps.tim.minute,     gps.tim.second,     gps.tim.thousand);
            ESP_LOGI(TAG, "gps.latitude %f gps.longitude %f ",
                     gps.latitude,   gps.longitude );
            ESP_LOGI(TAG, "gps.altitude  %f",
                     gps.altitude);
            ESP_LOGI(TAG, "gps.speed  %f",
                     gps.speed);
            ESP_LOGI(TAG, "gps.cog  %f",
                     gps.cog);
            ESP_LOGI(TAG, "gps.fix_mode  %i",
                     gps.fix_mode);
            ESP_LOGI(TAG, "gps.dop_h %f gps.dop_p %f gps.dop_v %f ",
                     gps.dop_h,   gps.dop_p,   gps.dop_v );
            ESP_LOGI(TAG, "gps.sats_in_view  %i",
                     gps.sats_in_view);
            ESP_LOGI(TAG, "gps.hpa  %f gps.vpa  %f",
                     gps.hpa, gps.vpa);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); //Wait

    }
#endif  // CONFIG_EXAMPLE_MODEM_DEVICE_SIM7070_GNSS


#if CONFIG_EXAMPLE_PERFORM_OTA == 1
    esp_http_client_config_t config = { };
    config.skip_cert_common_name_check = true;
    config.url = CONFIG_EXAMPLE_PERFORM_OTA_URI;

    esp_err_t ret = esp_https_ota(&config);
    if (ret == ESP_OK) {
        esp_restart();
    } else {
        ESP_LOGE(TAG, "Firmware upgrade failed");
        return;
    }
#endif // CONFIG_EXAMPLE_PERFORM_OTA





    /**
     *
     * THIS simulates a accidentially Power off of the Modem.
     * we are still in CMUX Mode.
     *
     */


    // wait 10s
    vTaskDelay(10000 / portTICK_PERIOD_MS);


    //Power down
    power_down_modem_pwkey();


    // wait 10s
    vTaskDelay(10000 / portTICK_PERIOD_MS);


    // power Up
    power_up_modem_pwkey();













#ifdef SUPPORT_URC_HANDLER
    ESP_LOGI(TAG, "Removing URC handler");
    dce->set_on_read(nullptr);
#endif




    //Leave CMUX Mode totally

    if (dce->set_mode(esp_modem::modem_mode::CMUX_MANUAL_EXIT)) {
        std::cout << "Modem has correctly entered CMUX_MANUAL_EXIT" << std::endl;
    } else {
        ESP_LOGE(TAG, "Failed to configure CMUX_MANUAL_EXIT... exiting");
        return;
    }



    //now only 1 Terminal.
#ifdef SUPPORT_URC_HANDLER
    ESP_LOGI(TAG, "Adding URC handler");
    dce->set_on_read(handle_urc);
#endif


    // wait 10s
    vTaskDelay(10000 / portTICK_PERIOD_MS);


    /* Again reading some data from the modem */
    if (dce->get_imsi(str) == esp_modem::command_result::OK) {
        std::cout << "Modem IMSI number:" << str << std::endl;
    }



    if (dte_config.uart_config.flow_control == ESP_MODEM_FLOW_CONTROL_HW) {


        //set this mode also to the DCE.
        if (command_result::OK != dce->set_flow_control(2, 2)) {
            ESP_LOGE(TAG, "Failed to set the set_flow_control mode");
            return;
        }
        ESP_LOGI(TAG, "set_flow_control OK");


    } else {
        ESP_LOGI(TAG, "not set_flow_control, because 2-wire mode active.");
    }



    dce->sync();
    dce->sync();

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    dce->sync();

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    dce->sync();

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    dce->sync();
    dce->sync();
    dce->sync();






}
