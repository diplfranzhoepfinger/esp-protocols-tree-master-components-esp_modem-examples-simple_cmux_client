set(PRIVREQ mqtt esp_netif esp_modem vfs esp_https_ota SIM7070_gnss A7672_gnss shiny_module modem_pwkey)

if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.0")
    list(APPEND PRIVREQ esp_partition)
else() 
    list(APPEND PRIVREQ "")
endif()

idf_component_register(SRCS "simple_cmux_client_main.cpp"  "simple_mqtt_client.cpp"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES  ${PRIVREQ})
