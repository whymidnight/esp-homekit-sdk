/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS products only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/* HomeKit Bridge Example
*/

#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include "driver/gpio.h"
#include "hal/gpio_types.h"

#include <hap_apple_servs.h>
#include <hap_apple_chars.h>

#include <iot_button.h>

#include <app_wifi.h>
#include <app_hap_setup_payload.h>

static const char *TAG = "Ritual Init";

#define BRIDGE_TASK_PRIORITY  1
#define BRIDGE_TASK_STACKSIZE 4 * 1024
#define BRIDGE_TASK_NAME      "hap_bridge"

#define NUM_BRIDGED_ACCESSORIES 8

/* Reset network credentials if button is pressed for more than 3 seconds and then released */
#define RESET_NETWORK_BUTTON_TIMEOUT        3

/* Reset to factory if button is pressed and held for more than 10 seconds */
#define RESET_TO_FACTORY_BUTTON_TIMEOUT     10

/* The button "Boot" will be used as the Reset button for the example */
#define RESET_GPIO  GPIO_NUM_0
/**
 * @brief The network reset button callback handler.
 * Useful for testing the Wi-Fi re-configuration feature of WAC2
 */
static void reset_network_handler(void* arg)
{
    hap_reset_network();
}
/**
 * @brief The factory reset button callback handler.
 */
static void reset_to_factory_handler(void* arg)
{
    hap_reset_to_factory();
}

/**
 * The Reset button  GPIO initialisation function.
 * Same button will be used for resetting Wi-Fi network as well as for reset to factory based on
 * the time for which the button is pressed.
 */
static void reset_key_init(uint32_t key_gpio_pin)
{
    button_handle_t handle = iot_button_create(key_gpio_pin, BUTTON_ACTIVE_LOW);
    iot_button_add_on_release_cb(handle, RESET_NETWORK_BUTTON_TIMEOUT, reset_network_handler, NULL);
    iot_button_add_on_press_cb(handle, RESET_TO_FACTORY_BUTTON_TIMEOUT, reset_to_factory_handler, NULL);
}

/* Mandatory identify routine for the accessory (bridge)
 * In a real accessory, something like LED blink should be implemented
 * got visual identification
 */
static int bridge_identify(hap_acc_t *ha)
{
    ESP_LOGI(TAG, "Bridge identified");
    return HAP_SUCCESS;
}

/* Mandatory identify routine for the bridged accessory
 * In a real bridge, the actual accessory must be sent some request to
 * identify itself visually
 */
static int accessory_identify(hap_acc_t *ha)
{
    hap_serv_t *hs = hap_acc_get_serv_by_uuid(ha, HAP_SERV_UUID_ACCESSORY_INFORMATION);
    hap_char_t *hc = hap_serv_get_char_by_uuid(hs, HAP_CHAR_UUID_NAME);
    const hap_val_t *val = hap_char_get_val(hc);
    char *name = val->s;

    ESP_LOGI(TAG, "Bridged Accessory %s identified", name);
    return HAP_SUCCESS;
}

/* A dummy callback for handling a write on the "On" characteristic of switch.
 * In an actual accessory, this should control the hardware
 */
static int switch_on(int gpio, bool value)
{
    ESP_LOGI(TAG, "Received Write. switch %s %d", value ? "On" : "Off", gpio);
    gpio_set_level(gpio, value ? 1 : 0);
    return 0;
}

/* A dummy callback for handling a write on the "On" characteristic of alternator.
 * In an actual accessory, this should control the hardware
 */
static int alternator_on(int gpio, bool value)
{
    ESP_LOGI(TAG, "Received Write. alternator %s, for %d", value ? "On" : "Off", gpio);
    /*
    gpio_set_level(gpio, value ? 1 : 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(gpio, value ? 0 : 1);
    */


    if (value) {
        gpio_set_level(gpio, value ? 1 : 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(gpio, value ? 0 : 1);
    }
    return 0;
}

/* A dummy callback for handling a write on the "On" characteristic of switch.
 * In an actual accessory, this should control the hardware
 */
static int switch_write(hap_write_data_t write_data[], int count,
        void *serv_priv, void *write_priv)
{
    int i, ret = HAP_SUCCESS;
    hap_write_data_t *write;
    for (i = 0; i < count; i++) {
        write = &write_data[i];
        if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_ON)) {
            char *name = (char *)serv_priv;
            char *nameParsed;
            char delim[3] = " ";
            sprintf(delim, "%s", " ");
            nameParsed = strtok(name, delim);
            ESP_LOGI(TAG, "Write called for Accessory %s", name);
            int gpio = atoi((char *) nameParsed);
            switch_on(gpio, write->val.b);
            hap_char_update_val(write->hc, &(write->val));
            *(write->status) = HAP_STATUS_SUCCESS;
        } else {
            *(write->status) = HAP_STATUS_RES_ABSENT;
        }
    }
    return ret;
}

/* A dummy callback for handling a write on the "On" characteristic of alternator.
 * In an actual accessory, this should control the hardware
 */
static int alternator_write(hap_write_data_t write_data[], int count,
        void *serv_priv, void *write_priv)
{
    int i, ret = HAP_SUCCESS;
    hap_write_data_t *write;
    for (i = 0; i < count; i++) {
        write = &write_data[i];
        if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_ON)) {
            char *name = (char *)serv_priv;
            char *nameParsed;
            char delim[3] = " ";
            sprintf(delim, "%s", " ");
            nameParsed = strtok(name, delim);
            ESP_LOGI(TAG, "Write called for Accessory %s", name);
            int gpio = atoi((char *) nameParsed);
            alternator_on(gpio, write->val.b);
            hap_char_update_val(write->hc, &(write->val));
            *(write->status) = HAP_STATUS_SUCCESS;
        } else {
            *(write->status) = HAP_STATUS_RES_ABSENT;
        }
    }
    return ret;
}

/*The main thread for handling the Bridge Accessory */
static void bridge_thread_entry(void *p)
{
    hap_acc_t *accessory;
    hap_serv_t *service;

    ESP_LOGI(TAG, "init");
    hap_init(HAP_TRANSPORT_WIFI);

    hap_acc_cfg_t cfg = {
        .name = "Esp-Bridge",
        .manufacturer = "Espressif",
        .model = "EspBridge01",
        .serial_num = "001122334455",
        .fw_rev = "0.9.0",
        .hw_rev = NULL,
        .pv = "1.1.0",
        .identify_routine = bridge_identify,
        .cid = HAP_CID_BRIDGE,
    };
    accessory = hap_acc_create(&cfg);

    uint8_t product_data[] = {'E','S','P','3','2','H','A','P'};
    hap_acc_add_product_data(accessory, product_data, sizeof(product_data));

    hap_add_accessory(accessory);

    uint8_t gpios[] = {32,33,25,15,17,22,12,13};
    for (uint8_t i = 0; i < NUM_BRIDGED_ACCESSORIES; i++) {
        ESP_LOGI(TAG, "init acc %d", i);
        uint8_t gpio = 0;
        char accessory_name[25] = {0};
        if ((i % 2) == 1) {
            gpio = gpios[i];
            sprintf(accessory_name, "%d-Light-Altr", gpio);
            // service = hap_serv_stateless_programmable_switch_create(0);
            service = hap_serv_switch_create(false);
            hap_serv_set_write_cb(service, alternator_write);
            gpio_reset_pin(gpio);
            gpio_set_direction(gpio, GPIO_MODE_OUTPUT);

            hap_acc_cfg_t bridge_cfg = {
                .name = accessory_name,
                .manufacturer = "Espressif",
                .model = "EspAlt01",
                .serial_num = "abcdefg",
                .fw_rev = "0.9.0",
                .hw_rev = NULL,
                .pv = "1.1.0",
                .identify_routine = accessory_identify,
                .cid = HAP_CID_BRIDGE,
            };
            accessory = hap_acc_create(&bridge_cfg);
            hap_serv_add_char(service, hap_char_name_create(accessory_name));
            hap_serv_set_priv(service, strdup(accessory_name));
            hap_acc_add_serv(accessory, service);
            hap_add_bridged_accessory(accessory, hap_get_unique_aid(accessory_name));
        }
        if ((i % 2) == 0) {
            gpio = gpios[i];
            sprintf(accessory_name, "%d-Light-Bulb", gpio);
            service = hap_serv_lightbulb_create(false);
            hap_serv_set_write_cb(service, switch_write);
            gpio_reset_pin(gpio);
            gpio_set_direction(gpio, GPIO_MODE_OUTPUT);

            hap_acc_cfg_t bridge_cfg = {
                .name = accessory_name,
                .manufacturer = "Espressif",
                .model = "EspFan01",
                .serial_num = "abcdefg",
                .fw_rev = "0.9.0",
                .hw_rev = NULL,
                .pv = "1.1.0",
                .identify_routine = accessory_identify,
                .cid = HAP_CID_BRIDGE,
            };
            accessory = hap_acc_create(&bridge_cfg);
            hap_serv_add_char(service, hap_char_name_create(accessory_name));
            hap_serv_set_priv(service, strdup(accessory_name));
            hap_acc_add_serv(accessory, service);
            hap_add_bridged_accessory(accessory, hap_get_unique_aid(accessory_name));
        }
        /*
        */
    }

    /* Register a common button for reset Wi-Fi network and reset to factory.
     */
    reset_key_init(RESET_GPIO);

    /* For production accessories, the setup code shouldn't be programmed on to
     * the device. Instead, the setup info, derived from the setup code must
     * be used. Use the factory_nvs_gen utility to generate this data and then
     * flash it into the factory NVS partition.
     *
     * By default, the setup ID and setup info will be read from the factory_nvs
     * Flash partition and so, is not required to set here explicitly.
     *
     * However, for testing purpose, this can be overridden by using hap_set_setup_code()
     * and hap_set_setup_id() APIs, as has been done here.
     */
#ifdef CONFIG_EXAMPLE_USE_HARDCODED_SETUP_CODE
    /* Unique Setup code of the format xxx-xx-xxx. Default: 111-22-333 */
    hap_set_setup_code(CONFIG_EXAMPLE_SETUP_CODE);
    /* Unique four character Setup Id. Default: ES32 */
    hap_set_setup_id(CONFIG_EXAMPLE_SETUP_ID);
#ifdef CONFIG_APP_WIFI_USE_WAC_PROVISIONING
    app_hap_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, true, cfg.cid);
#else
    app_hap_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, false, cfg.cid);
#endif
#endif

    /* Enable Hardware MFi authentication (applicable only for MFi variant of SDK) */

    /* Initialize Wi-Fi */
    app_wifi_init();

    /* After all the initializations are done, start the HAP core */
    hap_start();
    /* Start Wi-Fi */
    app_wifi_start(portMAX_DELAY);
    /* The task ends here. The read/write callbacks will be invoked by the HAP Framework */
    vTaskDelete(NULL);
}

void app_main()
{
    esp_wifi_set_protocol( WIFI_IF_AP, WIFI_PROTOCOL_LR );
    esp_wifi_set_protocol( WIFI_IF_STA, WIFI_PROTOCOL_LR );
    xTaskCreate(bridge_thread_entry, BRIDGE_TASK_NAME, BRIDGE_TASK_STACKSIZE, NULL, BRIDGE_TASK_PRIORITY, NULL);
}
