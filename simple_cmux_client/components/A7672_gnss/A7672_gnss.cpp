/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
//
// Created on: 23.08.2022
// Author: franz

#include <cstring>
#include <string_view>
#include <charconv>
#include <list>
#include "sdkconfig.h"
#include "cxx_include/esp_modem_dte.hpp"
#include "cxx_include/esp_modem_dce.hpp"
#include "esp_modem_config.h"
#include "cxx_include/esp_modem_api.hpp"
#include "cxx_include/esp_modem_command_library_utils.hpp"
#include "esp_log.h"
#include "generate/esp_modem_command_declare.inc"
#include "A7672_gnss.hpp"

using namespace esp_modem;
constexpr auto const TAG = "A7672_gnss";

//
// Define preprocessor's forwarding to dce_commands definitions
//

// Helper macros to handle multiple arguments of declared API
#define ARGS0
#define ARGS1 , p1
#define ARGS2 , p1 , p2
#define ARGS3 , p1 , p2 , p3
#define ARGS4 , p1 , p2 , p3, p4
#define ARGS5 , p1 , p2 , p3, p4, p5
#define ARGS6 , p1 , p2 , p3, p4, p5, p6

#define _ARGS(x)  ARGS ## x
#define ARGS(x)  _ARGS(x)

#define CMD_OK    (1)
#define CMD_FAIL  (2)

//
// Repeat all declarations and forward to the AT commands defined in esp_modem::dce_commands:: namespace
//
#define ESP_MODEM_DECLARE_DCE_COMMAND(name, return_type, num, ...) \
    return_type A7672::DCE_gnss::name(__VA_ARGS__) \
    {   \
        return dce_commands::name(this ARGS(num)); \
    }

DECLARE_ALL_COMMAND_APIS(return_type name(...) )

#undef ESP_MODEM_DECLARE_DCE_COMMAND


namespace gnss_factory {
using namespace esp_modem;
using namespace dce_factory;

class LocalFactory: public Factory {
    using DCE_gnss_ret = std::unique_ptr<A7672::DCE_gnss>;   // this custom Factory manufactures only unique_ptr<DCE>'s
public:
    static DCE_gnss_ret create(const dce_config *config, std::shared_ptr<DTE> dte, esp_netif_t *netif)
    {
        return Factory::build_generic_DCE<A7672_gnss, A7672::DCE_gnss, DCE_gnss_ret>
               (config, std::move(dte), netif);
    }
};

} // namespace gnss_factory

/**
 * @brief Helper create method which employs the DCE factory for creating DCE objects templated by a custom module
 * @return unique pointer of the resultant DCE
 */
std::unique_ptr<A7672::DCE_gnss> create_A7672_GNSS_dce(const esp_modem::dce_config *config,
        std::shared_ptr<esp_modem::DTE> dte,
        esp_netif_t *netif)
{
    return gnss_factory::LocalFactory::create(config, std::move(dte), netif);
}


/**
 * @brief Definition of the command API, which makes the Shiny::DCE "command-able class"
 * @param cmd Command to send
 * @param got_line Recv line callback
 * @param time_ms timeout in ms
 * @param separator line break separator
 * @return OK, FAIL or TIMEOUT
 */
esp_modem::command_result A7672::DCE_gnss::command(const std::string &cmd, esp_modem::got_line_cb got_line, uint32_t time_ms, const char separator)
{
    if (!handling_urc) {
        return dte->command(cmd, got_line, time_ms, separator);
    }
    handle_cmd = got_line;
    signal.clear(CMD_OK | CMD_FAIL);
    esp_modem::DTE_Command command{cmd};
    dte->write(command);
    signal.wait_any(CMD_OK | CMD_FAIL, time_ms);
    handle_cmd = nullptr;
    if (signal.is_any(CMD_OK)) {
        return esp_modem::command_result::OK;
    }
    if (signal.is_any(CMD_FAIL)) {
        return esp_modem::command_result::FAIL;
    }
    return esp_modem::command_result::TIMEOUT;
}

/**
 * @brief Handle received data
 *
 * @param data Data received from the device
 * @param len Length of the data
 * @return standard command return code (OK|FAIL|TIMEOUT)
 */
esp_modem::command_result A7672::DCE_gnss::handle_data(uint8_t *data, size_t len)
{
    if (std::memchr(data, '\n', len)) {
        if (handle_urc) {
            handle_urc(data, len);
        }
        if (handle_cmd) {
            auto ret = handle_cmd(data, len);
            if (ret == esp_modem::command_result::TIMEOUT) {
                return esp_modem::command_result::TIMEOUT;
            }
            if (ret == esp_modem::command_result::OK) {
                signal.set(CMD_OK);
            }
            if (ret == esp_modem::command_result::FAIL) {
                signal.set(CMD_FAIL);
            }
        }
    }
    return esp_modem::command_result::TIMEOUT;
}
