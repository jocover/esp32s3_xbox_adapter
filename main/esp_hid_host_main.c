/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "tinyusb.h"
#include "hid_ps4_driver.h"

#include "esp_hidh.h"
#include "esp_hid_gap.h"
#include "esp_timer.h"
static const char *TAG = "xbox";

#define XBOX_CONTROLLER_INDEX_BUTTONS_DIR 12
#define XBOX_CONTROLLER_INDEX_BUTTONS_MAIN 13
#define XBOX_CONTROLLER_INDEX_BUTTONS_CENTER 14
#define XBOX_CONTROLLER_INDEX_BUTTONS_SHARE 15



//static const uint16_t maxJoy = 0xffff;

//int64_t timer = 0;


hid_ps4_report_t ps4_report = {};


void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    switch (event)
    {
    case ESP_HIDH_OPEN_EVENT:
    {
        if (param->open.status == ESP_OK)
        {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
            esp_hidh_dev_dump(param->open.dev, stdout);
        }
        else
        {
            ESP_LOGE(TAG, " OPEN failed!");
        }
        break;
    }
    case ESP_HIDH_BATTERY_EVENT:
    {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
        break;
    }
    case ESP_HIDH_INPUT_EVENT:
    {
        //const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
        // ESP_LOGI(TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
        // ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);
        // ESP_LOGI(TAG, "id:%3u", param->input.report_id);

        // 1.25 ms
        // if (timer + 1250 > esp_timer_get_time())
        //{
        //     break;
        //}

        memset(&ps4_report, 0, sizeof(ps4_report));
        ps4_report.report_id = 0x1;

        /*
        bool btnA, btnB, btnX, btnY;
        bool btnShare, btnStart, btnSelect, btnXbox;
        // side top button
        bool btnLB, btnRB;
        // button on joy stick
        bool btnLS, btnRS;
       
        
        uint16_t joyLHori = maxJoy / 2;
        uint16_t joyLVert = maxJoy / 2;
        uint16_t joyRHori = maxJoy / 2;
        uint16_t joyRVert = maxJoy / 2;
        */
        bool btnDirUp, btnDirLeft, btnDirRight, btnDirDown;
        uint16_t joyLHori ;
        uint16_t joyLVert ;
        uint16_t joyRHori ;
        uint16_t joyRVert;
        uint16_t trigLT, trigRT;

        uint8_t btnBits;
        btnBits = param->input.data[XBOX_CONTROLLER_INDEX_BUTTONS_MAIN];

        if (btnBits & 0b00000001) // btnA
            ps4_report.buttons1 |= (1 << 5);

        if (btnBits & 0b00000010) // btnB
            ps4_report.buttons1 |= (1 << 6);

        if (btnBits & 0b00001000) // btnX
            ps4_report.buttons1 |= (1 << 4);

        if (btnBits & 0b00010000) // btnY
            ps4_report.buttons1 |= (1 << 7);

        if (btnBits & 0b01000000) // btnLB
            ps4_report.buttons2 |= (1 << 0);

        if (btnBits & 0b10000000) // btnRB
            ps4_report.buttons2 |= (1 << 1);

        /*
        btnA = btnBits & 0b00000001;
        btnB = btnBits & 0b00000010;
        btnX = btnBits & 0b00001000;
        btnY = btnBits & 0b00010000;
        btnLB = btnBits & 0b01000000;
        btnRB = btnBits & 0b10000000;
        */

        btnBits = param->input.data[XBOX_CONTROLLER_INDEX_BUTTONS_CENTER];

        if (btnBits & 0b00000100) // Select
            ps4_report.buttons2 |= (1 << 4);

        if (btnBits & 0b00001000) // Start
            ps4_report.buttons2 |= (1 << 5);

        

        if (btnBits & 0b00010000) // Xbox
            ps4_report.buttons3 |= (1 << 0);

        if (btnBits & 0b00100000) // LS
            ps4_report.buttons2 |= (1 << 6);

        if (btnBits & 0b01000000) // RS
            ps4_report.buttons2 |= (1 << 7);

 /*
        btnSelect = btnBits & 0b00000100;
        btnStart = btnBits & 0b00001000;
        btnXbox = btnBits & 0b00010000;
        btnLS = btnBits & 0b00100000;
        btnRS = btnBits & 0b01000000;
        */

        btnBits = param->input.data[XBOX_CONTROLLER_INDEX_BUTTONS_SHARE]; // btnShare

        if (btnBits & 0b00000001)
            ps4_report.buttons3 |= (1 << 1);

        /*
        btnShare = btnBits & 0b00000001;
        */

        btnBits = param->input.data[XBOX_CONTROLLER_INDEX_BUTTONS_DIR];
        btnDirUp = btnBits == 1 || btnBits == 2 || btnBits == 8;
        btnDirRight = 2 <= btnBits && btnBits <= 4;
        btnDirDown = 4 <= btnBits && btnBits <= 6;
        btnDirLeft = 6 <= btnBits && btnBits <= 8;
        

        if (btnDirUp && btnDirRight)
        {

            ps4_report.buttons1 |= 0x01;
        }
        else if (btnDirDown && btnDirLeft)
        {

            ps4_report.buttons1 |= 0x05;
        }
        else if (btnDirDown && btnDirRight)
        {

            ps4_report.buttons1 |= 0x03;
        }
        else if (btnDirUp && btnDirLeft)
        {

            ps4_report.buttons1 |= 0x07;
        }
        else if (btnDirUp)
        {

            ps4_report.buttons1 |= 0x00;
        }
        else if (btnDirRight)
        {

            ps4_report.buttons1 |= 0x02;
        }
        else if (btnDirDown)
        {

            ps4_report.buttons1 |= 0x04;
        }
        else if (btnDirLeft)
        {

            ps4_report.buttons1 |= 0x06;
        }
        else
        {

            ps4_report.buttons1 |= 0x08;
        }

        joyLHori = (uint16_t)param->input.data[0] | ((uint16_t)param->input.data[1] << 8); // 0-65535
        joyLVert = (uint16_t)param->input.data[2] | ((uint16_t)param->input.data[3] << 8);
        joyRHori = (uint16_t)param->input.data[4] | ((uint16_t)param->input.data[5] << 8);
        joyRVert = (uint16_t)param->input.data[6] | ((uint16_t)param->input.data[7] << 8);

        trigLT = (uint16_t)param->input.data[8] | ((uint16_t)param->input.data[9] << 8); // 0-1024
        trigRT = (uint16_t)param->input.data[10] | ((uint16_t)param->input.data[11] << 8);

        if (trigLT)
            ps4_report.buttons2 |= (1 << 2);

        if (trigRT)
            ps4_report.buttons2 |= (1 << 3);

        ps4_report.lt = trigLT / 4;
        ps4_report.rt = trigRT / 4;

        ps4_report.lx = joyLHori / 256;
        ps4_report.ly = joyLVert / 256;

        ps4_report.rx = joyRHori / 256;
        ps4_report.ry = joyRVert / 256;

       //ps4_report.timestamp = last_timestamp;

        // printf("A:%01x B:%01x X:%01x Y:%01x LB:%01x RB:%01x Select:%01x Start:%01x Xbox:%01x LS:%01x RS:%01x Share:%01x\n", btnA, btnB, btnX, btnY, btnLB, btnRB, btnSelect, btnStart, btnXbox, btnLS, btnRS, btnShare);

        // printf("lx:%04x ly:%04x rx:%04x ry:%04x LT:%04x RT:%01x \n", joyLHori, joyLVert, joyRHori, joyRVert, trigLT, trigRT);

        // memcpy(last_buf, param->input.data, 16);
        // timer = esp_timer_get_time();

        ps4_report.battery = 0 | (1 << 4) | 11;

        ps4_report.gyrox = 0;
        ps4_report.gyroy = 0;
        ps4_report.gyroz = 0;
        ps4_report.accelx = 0;
        ps4_report.accely = 0;
        ps4_report.accelz = 0;

        ps4_report.extension = 0x01;

        ps4_report.touchpad_event_active = 0;
        ps4_report.touchpad_counter = 0;
        ps4_report.touchpad1_touches = (1 << 7);
        ps4_report.touchpad2_touches = (1 << 7);

        ps4_report.unknown3[1] = 0x80;
        ps4_report.unknown3[5] = 0x80;
        ps4_report.unknown3[10] = 0x80;
        ps4_report.unknown3[14] = 0x80;
        ps4_report.unknown3[19] = 0x80;

       
        send_hid_ps4_report(&ps4_report);

        break;
    }
    case ESP_HIDH_FEATURE_EVENT:
    {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda),
                 esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id,
                 param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDH_CLOSE_EVENT:
    {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
        break;
    }
    default:
        ESP_LOGI(TAG, "EVENT: %d", event);
        break;
    }
}

#define SCAN_DURATION_SECONDS 5

void hid_task(void *pvParameters)
{
    size_t results_len = 0;
    esp_hid_scan_result_t *results = NULL;
    ESP_LOGI(TAG, "SCAN...");
    // start scan for HID devices
    esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results);
    ESP_LOGI(TAG, "SCAN: %u results", results_len);
    if (results_len)
    {
        esp_hid_scan_result_t *r = results;
        esp_hid_scan_result_t *cr = NULL;
        while (r)
        {
            printf("  %s: " ESP_BD_ADDR_STR ", ", (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ", ESP_BD_ADDR_HEX(r->bda));
            printf("RSSI: %d, ", r->rssi);
            printf("USAGE: %s, ", esp_hid_usage_str(r->usage));

            if (r->transport == ESP_HID_TRANSPORT_BLE)
            {
                cr = r;
                printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));
            }

            printf("NAME: %s ", r->name ? r->name : "");
            printf("\n");
            r = r->next;
        }
        if (cr)
        {
            // open the last result
            esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
        }
        // free the results
        esp_hid_scan_results_free(results);
    }
    vTaskDelete(NULL);
}

void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer,
                           uint16_t bufsize)
{
    //ESP_LOGI("PS4_DRV", "tud_hid_set_report_cb");

    hid_ps4_set_report_cb(itf, report_id, report_type, buffer, bufsize);
}

uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen)
{
    //ESP_LOGI("PS4_DRV", "tud_hid_get_report_cb");

    return hid_ps4_get_report_cb(itf, report_id, report_type, buffer, reqlen);
}

uint8_t const *tud_hid_descriptor_report_cb(uint8_t itf)
{

    return ps4_desc_hid_report;
}

void app_main(void)
{
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ps4_driver_init();

    ESP_LOGI(TAG, "USB initialization");

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = (const tusb_desc_device_t *)ds4_desc_device,
        .string_descriptor = ps4_string_descriptors,
        .string_descriptor_count = 4,
        .external_phy = false, // In the most cases you need to use a `false` value
        .configuration_descriptor = ps4_desc_cfg,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    ESP_ERROR_CHECK(esp_hid_gap_init(HIDH_BLE_MODE));

    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler));

    esp_hidh_config_t config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK(esp_hidh_init(&config));

    xTaskCreate(&hid_task, "hid_task", 6 * 1024, NULL, 2, NULL);
}
