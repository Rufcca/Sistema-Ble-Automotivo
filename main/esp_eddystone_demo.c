/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>

#include "esp_bt.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
#include "esp_gap_ble_api.h"
#include "freertos/FreeRTOS.h"

#include "esp_eddystone_protocol.h"
#include "esp_eddystone_api.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

char door_signal = 0;
char door_control = 0;
char key_found = 0;
struct timeval start_count;
struct timeval current_count;

static const char* DEMO_TAG = "S.A.B.L.E";

/* declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);
static void esp_eddystone_show_inform(const esp_eddystone_result_t* res,int rssi);

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

static void esp_eddystone_show_inform(const esp_eddystone_result_t* res , int rssi)
{
    switch(res->common.frame_type)
    {
        case EDDYSTONE_FRAME_TYPE_UID: 
		{           
            int i = 0;
			int tr = 0;
			char reference_bytes[] = {0x64,0x21,0x05,0xD5,0x14,0xE8,0x95,0xAD,0x70,0x58};
			while(i < 10)
			{
				if(reference_bytes[i] == res->inform.uid.namespace_id[i])
				{
					tr++;
				}
				i++;
			}
			if(tr == i)
			{
				key_found = 1;
				//ESP_LOGI(DEMO_TAG,"Chave encontrada , rssi = %d", rssi);
				gettimeofday(&start_count,NULL);
			}
			break;
        }
		default:
			break;
	}
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
    esp_err_t err;

    switch(event)
    {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
            uint32_t duration = 0;
            esp_ble_gap_start_scanning(duration);
            break;
        }
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: {
            if((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(DEMO_TAG,"inicio do escaneamento falhou: %s", esp_err_to_name(err));
            }
            else {
                ESP_LOGI(DEMO_TAG,"iniciando o escaneamento...");
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t* scan_result = (esp_ble_gap_cb_param_t*)param;
            switch(scan_result->scan_rst.search_evt)
            {
                case ESP_GAP_SEARCH_INQ_RES_EVT: {
                    esp_eddystone_result_t eddystone_res;
                    memset(&eddystone_res, 0, sizeof(eddystone_res));
                    esp_err_t ret = esp_eddystone_decode(scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len, &eddystone_res);
                    if (ret) {
                        return;
                    } else {
                        esp_eddystone_show_inform(&eddystone_res,scan_result->scan_rst.rssi);
                    }
                    break;
                }
                default:
                    break;
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:{
            if((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(DEMO_TAG,"desligamento de escaneamento falhou: %s", esp_err_to_name(err));
            }
            else {
                ESP_LOGI(DEMO_TAG,"desligamento de escaneamento foi um sucesso");
            }
            break;
        }
        default:{
            break;
		}
    }
}

void esp_eddystone_appRegister(void)
{
    esp_err_t status;

    ESP_LOGI(DEMO_TAG,"callback do eddystone registrado");

    /*<! register the scan callback function to the gap module */
    if((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(DEMO_TAG,"erro no registro do gap: %s", esp_err_to_name(status));
        return;
    }
}

void esp_eddystone_init(void)
{
    esp_bluedroid_init();
    esp_bluedroid_enable();
    esp_eddystone_appRegister();
}
void pin_config_check_state(int gpio_num)
{
	gpio_config_t door_conf;
	door_conf.pin_bit_mask = (1ULL<<gpio_num);
	door_conf.mode = GPIO_MODE_INPUT;
	door_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	door_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
	door_conf.intr_type = GPIO_INTR_DISABLE;
	gpio_config(&door_conf);
	door_signal = gpio_get_level(gpio_num);

}
void pin_config_set_state(int gpio_num)
{
	gpio_config_t door_state_control;
	door_state_control.pin_bit_mask = (1ULL<<gpio_num);
	door_state_control.mode = GPIO_MODE_OUTPUT;
	door_state_control.pull_up_en = GPIO_PULLUP_DISABLE;
	door_state_control.pull_down_en = GPIO_PULLDOWN_ENABLE;
	door_state_control.intr_type = GPIO_INTR_DISABLE;
	gpio_config(&door_state_control);
	gpio_set_level(gpio_num,1);
	vTaskDelay(250/portTICK_PERIOD_MS);
	gpio_set_level(gpio_num,0);
	door_state_control.mode = GPIO_MODE_INPUT;
	gpio_config(&door_state_control);
}
void vDoorControlTask(void *pvParameter)
{
	char last_d_c = 0;
	int64_t delta;
	esp_log_level_set("gpio", ESP_LOG_WARN);
	for(;;)
	{
		pin_config_check_state(GPIO_NUM_35);
		gettimeofday(&current_count,NULL);
		delta = (((int64_t)current_count.tv_sec *1E06 + (int64_t) current_count.tv_usec)- ((int64_t)start_count.tv_sec *1E06 + (int64_t)start_count.tv_usec)) ;
		if(delta >= 5E06)
		{
			key_found = 0;
			delta = 0;
			start_count.tv_sec = 0;
			start_count.tv_usec = 0;
			current_count.tv_sec = 0;
			current_count.tv_usec = 0;
		}
		if(!key_found && !door_signal)
		{
			door_control = 0;
		}
		else if(key_found && !door_signal)
		{
			door_control = 1;
		}
		else if(key_found && door_signal)
		{
			door_control = 1;
		}
		else
		{
			door_control = 0;
		}

		
		if(last_d_c != door_control)
		{
			if(door_control == 1)
			{
				ESP_LOGI(DEMO_TAG,"Função abertura de porta");
				gpio_config_t door_led;
				door_led.pin_bit_mask = (1ULL<<GPIO_NUM_6);
				door_led.mode = GPIO_MODE_OUTPUT;
				door_led.pull_up_en = GPIO_PULLUP_DISABLE;
				door_led.pull_down_en = GPIO_PULLDOWN_ENABLE;
				door_led.intr_type = GPIO_INTR_DISABLE;
				gpio_config(&door_led);
				gpio_set_level(GPIO_NUM_6,1);
				gpio_set_level(GPIO_NUM_5,0);
				
				pin_config_set_state(GPIO_NUM_35);
			}
			else
			{
				ESP_LOGI(DEMO_TAG,"Função fechamento de porta");
				gpio_config_t door_state;
				door_state.pin_bit_mask = (1ULL<<GPIO_NUM_5);
				door_state.mode = GPIO_MODE_OUTPUT;
				door_state.pull_up_en = GPIO_PULLUP_DISABLE;
				door_state.pull_down_en = GPIO_PULLDOWN_ENABLE;
				door_state.intr_type = GPIO_INTR_DISABLE;
				gpio_config(&door_state);
				gpio_set_level(GPIO_NUM_6,0);
				gpio_set_level(GPIO_NUM_5,1);

				pin_config_set_state(GPIO_NUM_36);
			}
			last_d_c = door_control;
		}
		
	vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    esp_eddystone_init();

    /*<! set scan parameters */
    esp_ble_gap_set_scan_params(&ble_scan_params);
	xTaskCreate(&vDoorControlTask,"DOOR CONTROL",4*2048,NULL,1,NULL);
}
