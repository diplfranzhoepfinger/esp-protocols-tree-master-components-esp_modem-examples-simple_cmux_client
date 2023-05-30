idf_component_register(SRCS "simple_cmux_client_main.cpp"  "simple_mqtt_client.cpp"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES mqtt esp_netif esp_modem vfs esp_https_ota esp_partition SIM7070_gnss A7672_gnss shiny_module modem_pwkey)
