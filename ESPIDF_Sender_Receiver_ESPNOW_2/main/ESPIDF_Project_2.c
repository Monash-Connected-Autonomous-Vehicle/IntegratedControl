// heavily inspired by espnow example given by esp-idf (https://github.com/espressif/esp-idf/blob/master/examples/wifi/espnow)

// need to include wifi since ESP_NOW uses wifi hardware. It does not need to connect to wifi
// nvs_flash (non volatile memory) is needed to store wifi settings even after esp32 loses power
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h" // note that the functions, structs, etc defined by espressif have the prefix esp_now instead of espnow which are structs, functions defined by us
#include "esp_crc.h" 
#include "mirf.h"   // for nrf module, libary/driver from https://github.com/nopnop2002/esp-idf-mirf
#include "MY_ESPNOW.h"

#define ESPNOW_MAXDELAY 512

#if CONFIG_ADVANCED

void AdvancedSettings(NRF24_t * dev)
{
#if CONFIG_RF_RATIO_2M
	ESP_LOGW(pcTaskGetName(0), "Set RF Data Ratio to 2MBps");
	Nrf24_SetSpeedDataRates(dev, 1);
#endif // CONFIG_RF_RATIO_2M

#if CONFIG_RF_RATIO_1M
	ESP_LOGW(pcTaskGetName(0), "Set RF Data Ratio to 1MBps");
	Nrf24_SetSpeedDataRates(dev, 0);
#endif // CONFIG_RF_RATIO_2M

#if CONFIG_RF_RATIO_250K
	ESP_LOGW(pcTaskGetName(0), "Set RF Data Ratio to 250KBps");
	Nrf24_SetSpeedDataRates(dev, 2);
#endif // CONFIG_RF_RATIO_2M

	ESP_LOGW(pcTaskGetName(0), "CONFIG_RETRANSMIT_DELAY=%d", CONFIG_RETRANSMIT_DELAY);
	Nrf24_setRetransmitDelay(dev, CONFIG_RETRANSMIT_DELAY);
}
#endif // CONFIG_ADVANCED

#if CONFIG_RECEIVER
void receiver(void *pvParameters)
{
	ESP_LOGI(pcTaskGetName(0), "Start");
	NRF24_t dev;
	Nrf24_init(&dev);
	//uint8_t payload = 32;
    uint8_t payload = sizeof(ESDA_Payload_Struct);
	uint8_t channel = CONFIG_RADIO_CHANNEL;
	Nrf24_config(&dev, channel, payload);

	// Set my own address using 5 characters
	esp_err_t ret = Nrf24_setRADDR(&dev, (uint8_t *)"FGHIJ");
	if (ret != ESP_OK) {
		ESP_LOGE(pcTaskGetName(0), "nrf24l01 not installed");
		while(1) { vTaskDelay(1); }
	}

#if CONFIG_ADVANCED
	AdvancedSettings(&dev);
#endif // CONFIG_ADVANCED

	// Print settings
	Nrf24_printDetails(&dev);
	ESP_LOGI(pcTaskGetName(0), "Listening...");

	// uint8_t buf[32];
    uint8_t buf[sizeof(ESDA_Payload_Struct)];

	// Clear RX FiFo
	while(1) {
		if (Nrf24_dataReady(&dev) == false) break;
		Nrf24_getData(&dev, buf);
	}

	while(1) {
		// Wait for received data
		if (Nrf24_dataReady(&dev)) {
			Nrf24_getData(&dev, buf);
            ESDA_Payload_Struct *receivedData = (ESDA_Payload_Struct *)buf;
            ESP_LOGI(pcTaskGetName(0), "Got data: MessageID = %d, Value = %.2f", receivedData->messageID, receivedData->value);
			// ESP_LOGI(pcTaskGetName(0), "Got data:%s", buf);
			//ESP_LOG_BUFFER_HEXDUMP(pcTaskGetName(0), buf, payload, ESP_LOG_INFO);
		}
		vTaskDelay(1);
	}
}
#endif // CONFIG_RECEIVER


#if CONFIG_SENDER
void sender(void *pvParameters)
{
	ESP_LOGI(pcTaskGetName(0), "Start");
	NRF24_t dev;
	Nrf24_init(&dev);

    
    //uint8_t payload = 32;
	uint8_t payload = sizeof(ESDA_Payload_Struct_t); // Modified

    uint8_t channel = CONFIG_RADIO_CHANNEL;
	Nrf24_config(&dev, channel, payload);

	// Set destination address using 5 characters
	esp_err_t ret = Nrf24_setTADDR(&dev, (uint8_t *)"FGHIJ");
	if (ret != ESP_OK) {
		ESP_LOGE(pcTaskGetName(0), "nrf24l01 not installed");
		while(1) { vTaskDelay(1); }
	}

#if CONFIG_ADVANCED
	AdvancedSettings(&dev);
#endif // CONFIG_ADVANCED

	// Print settings
	Nrf24_printDetails(&dev);

    ESDA_Payload_Struct_t payloadData;

	uint8_t buf[32];
	while(1) {
		TickType_t nowTick = xTaskGetTickCount();
        payloadData.messageID = 5;
        payloadData.value = 12.34;

        Nrf24_send(&dev, (uint8_t*)&payloadData);

        //payloadData.tick = nowTick;
		// sprintf((char *)buf, "Hello World %"PRIu32, nowTick);
		// Nrf24_send(&dev, buf);
        

		vTaskDelay(1);
		ESP_LOGI(pcTaskGetName(0), "Wait for sending.....");
		if (Nrf24_isSend(&dev, 1000)) {
			ESP_LOGI(pcTaskGetName(0),"Send success:%s", buf);
		} else {
			ESP_LOGW(pcTaskGetName(0),"Send fail:");
		}
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}
#endif // CONFIG_SENDER

// c8:f0:9e:a2:f0:58

// GLOBALS
static const char* TAG = "espnow"; // logging
static QueueHandle_t s_espnow_queue; // used for task communication/synchronization since it is thread safe (data sent and retrieved are called events in this program)
static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xC8, 0xF0, 0x9E, 0xA2, 0xF0, 0x58}; // replace with receiver mac address
static uint16_t s_espnow_seq[ESPNOW_DATA_MAX] = {0, 0}; // used to track the two types of ESPNOW data (broadcast and unicast)

// function handle
static void my_espnow_deinit(espnow_send_param_t *send_param);

// note that wifi needs to start before using ESPNOW since it uses the wifi hardware
static void my_wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    // creates default event loop, to handle internal events such as wifi (connection, disconnection etc), IP events, etc
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    // configure mode in menuconfig
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE)); 
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
}

/* ESPNOW sending callback function
dont do lengthy operations, just post onto queue and handle it with a lower priority task (espnow_task).
Send results of esp now send operation to espnow_task through the queue\ */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    espnow_event_t evt;
    // recall espnow_event_t is a struct that contains union espnow_event_info_t
    // espnow_event_send_cb_t* send_cb = &evt.info.send_cb; 

    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error"); // LOGE means log error
        return;
    }

    evt.id = ESPNOW_SEND_CB; // ESPNOW_SEND_CB is enumerated by espnow_event_t
    memcpy(evt.info.send_cb.mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    evt.info.send_cb.status = status;

    if (xQueueSend(s_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "send queue fail"); // LOGW is for non critical errors
    }
}

/* ESPNOW receiver callback function
dont do lengthy operations, just post onto queue and handle it with a lower priority task (espnow_task).
Called when ESP32 receives data and sends received data to the queue for espnow_task*/
static void espnow_recv_cb(const esp_now_recv_info_t* recv_info, const uint8_t* data, int len) {
    espnow_event_t evt; // create an instance of espnow_event_t struct

    uint8_t* mac_addr = recv_info->src_addr;
    uint8_t* des_addr = recv_info->des_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
    }
    // determine if the packet is braodcast or unicast by looking at the destination address
    if (IS_BROADCAST_ADDR(des_addr)) {
        ESP_LOGD(TAG, "Receive broadcast ESPNOW data"); // log for debug
    }
    else {
        ESP_LOGD(TAG, "Receive unicast ESPNOW data");
    }

    // prepare the event, evt, for the queue 
    evt.id = ESPNOW_RECV_CB;
    memcpy(evt.info.recv_cb.mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    // allocate memory for received data
    evt.info.recv_cb.data = malloc(len); // note that evt.info.recv_cb.data is a pointer
    if (evt.info.recv_cb.data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(evt.info.recv_cb.data, data, len);
    evt.info.recv_cb.data_len = len;
    // send event (evt) to queue and check for errors
    if (xQueueSend(s_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(evt.info.recv_cb.data);
    }
}

// Parse received ESPNOW data from espnow_recv_cb
int espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t* state, uint16_t* seq, uint32_t* magic) {
    espnow_data_t* buf = (espnow_data_t *)data; // not sure why it is defined as uint8_t in the first place
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(espnow_data_t)) {
        ESP_LOGE(TAG, "receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }
    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    // if crc_cal is zero
    if (crc_cal == crc) {
        return buf->type; // tels us the type of data returned
    }
    return -1;
}

// prepare ESPNOW data to be sent
void espnow_data_prepare(espnow_send_param_t* send_param) {
    /* converts "buffer" (not buf), a pointer member of the "send_param" instance of the 
    espnow_send_param_t struct, from uint8_t* to a pointer to an "espnow_data_t" struct called "buf" */
    espnow_data_t* buf = (espnow_data_t *)send_param->buffer; 

    // if len field in send_param is too small program terminates
    assert(send_param->len >= sizeof(espnow_data_t));

    // determine if the packet is braodcast or unicast by looking at the destination address
    if (IS_BROADCAST_ADDR(send_param->dest_mac)) {
        buf->type = ESPNOW_DATA_BROADCAST;
    }
    else {
        buf->type = ESPNOW_DATA_UNICAST;
    }
    buf->state = send_param->state;
    buf->seq_num = s_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;

    // fill all remaining bytes with random values (first argument is pointer to buffer, second is length of buffer)
    esp_fill_random(buf->payload, send_param->len - sizeof(espnow_data_t));
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

// TASK: to process events from queue and handles sending and receiving ESPNOW data
static void my_espnow_task(void *pvParameter) {
    espnow_event_t evt; 
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    uint32_t recv_magic = 0;
    bool is_broadcast = false;
    int ret;

    vTaskDelay(5000/portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    /* start sending broadcast ESPNOW data */
    espnow_send_param_t* send_param = (espnow_send_param_t *)pvParameter; // cast parameter sent to this task prepared by my_espnow_init()
    // requires 3 args: peer mac address, data to be sent, and the length of the data
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        my_espnow_deinit(send_param);
        vTaskDelete(NULL); // delete current task, this one
    }

    // main loop - handles events from queue
    while(xQueueReceive(s_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        // handle different types of events (sending and receiving)
        vTaskDelay(5000/portTICK_PERIOD_MS);
        switch (evt.id) {
            case ESPNOW_SEND_CB:
            {
                // check if the data was sent to a broadcast address
                is_broadcast = IS_BROADCAST_ADDR(evt.info.send_cb.mac_addr);
                ESP_LOGD(TAG, "Send data to "MACSTR", status: %d", MAC2STR(evt.info.send_cb.mac_addr), evt.info.send_cb.status);
                // if the message was broadcast and broadcasting is disabled, exit the case
                if (is_broadcast && (send_param->broadcast == false)) {
                    break;
                }

                // if message is unicast, decrease the count of remaing messages to send
                if (!is_broadcast) {
                    send_param->count--;
                    // if all messages have been sent
                    if (send_param->count == 0) {
                        ESP_LOGI(TAG, "Send done");
                        my_espnow_deinit(send_param);
                        vTaskDelete(NULL); 
                    }
                }

                // delay before sending next message
                if (send_param->delay > 0) {
                    vTaskDelay(send_param->delay/portTICK_PERIOD_MS);
                }

                // log the next data to be sent
                ESP_LOGI(TAG, "Send data to "MACSTR"", MAC2STR(evt.info.send_cb.mac_addr));

                // prepare next data to be sent
                memcpy(send_param->dest_mac, evt.info.send_cb.mac_addr, ESP_NOW_ETH_ALEN);
                espnow_data_prepare(send_param); // fills send_param with data to be sent

                // attempt to send next data
                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                    ESP_LOGE(TAG, "Send error");
                    my_espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }
                break;
            }
            case ESPNOW_RECV_CB:
            {


                ret = espnow_data_parse(evt.info.recv_cb.data, evt.info.recv_cb.data_len, &recv_state, &recv_seq, &recv_magic);
                if (ret == ESPNOW_DATA_BROADCAST || ret == ESPNOW_DATA_UNICAST) {
                    // Cast the received data to the struct and print its values
                    ESDA_Payload_Struct_t *receivedData = (ESDA_Payload_Struct_t *)evt.info.recv_cb.data;
                    ESP_LOGI(TAG, "Received data: MessageID = %d, Value = %.2f", receivedData->messageID, receivedData->value);
                }
                free(evt.info.recv_cb.data);
                break;





                // // parse data and update state, sequence number and magic number
                // ret = espnow_data_parse(evt.info.recv_cb.data, evt.info.recv_cb.data_len, &recv_state, &recv_seq, &recv_magic);
                // free(evt.info.recv_cb.data); // idk

                // if (ret == ESPNOW_DATA_BROADCAST) {
                //     ESP_LOGI(TAG, "Receive %dth broadcase data from: "MACSTR", len: %d", recv_seq, MAC2STR(evt.info.recv_cb.mac_addr), evt.info.recv_cb.data_len);

                //     // if peer does not exist int peer list, add it
                //     if (!esp_now_is_peer_exist(evt.info.recv_cb.mac_addr)) {
                //         // allocate memory for new peer information
                //         esp_now_peer_info_t* peer = malloc(sizeof(esp_now_peer_info_t));
                //         if (peer == NULL) {
                //             ESP_LOGE(TAG, "Malloc peer information fail");
                //             my_espnow_deinit(send_param);
                //             vTaskDelete(NULL);
                //         }

                //         // initialize peer information
                //         memset(peer, 0, sizeof(esp_now_peer_info_t));
                //         peer->channel = CONFIG_ESPNOW_CHANNEL;
                //         peer->ifidx = ESPNOW_WIFI_IF;
                //         peer->encrypt = true;
                //         memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                //         memcpy(peer->peer_addr, evt.info.recv_cb.mac_addr, ESP_NOW_ETH_ALEN);
                //         // add this new peer to the peer list
                //         ESP_ERROR_CHECK(esp_now_add_peer(peer));
                //         free(peer);
                //     }
                    
                //     // device has received broadcast ESPNOW data
                //     if (send_param->state == 0) {
                //         send_param->state = 1;
                //     }

                //     // if the received broadcast data indicates that the other device has received broadcast data and the local magic number is larger, start sending unicast data
                //     if (recv_state == 1) {
                //         if (!send_param->unicast && send_param->magic >= recv_magic) {
                //             ESP_LOGI(TAG, "Start sending unicast data");
                //             ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(evt.info.recv_cb.mac_addr));

                //             // prepare and send unicast ESPNOW data
                //             memcpy(send_param->dest_mac, evt.info.recv_cb.mac_addr, ESP_NOW_ETH_ALEN);
                //             espnow_data_prepare(send_param);
                //             if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                //                 ESP_LOGE(TAG, "Send error");
                //                 my_espnow_deinit(send_param);
                //                 vTaskDelete(NULL);
                //             }
                //             else {
                //                 send_param->broadcast = false;
                //                 send_param->unicast = true;
                //             }
                //         }
                //     }
                // }

                // // if received data is unicast data
                // else if (ret == ESPNOW_DATA_UNICAST) {
                //     ESP_LOGI(TAG, "Received %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(evt.info.recv_cb.mac_addr), evt.info.recv_cb.data_len);

                //     // if unicast data is received, stop sending broadcast data
                //     send_param->broadcast = false;
                // }

                // // if data is neither broadcast or unicast then there is an error
                // else {
                //     ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(evt.info.recv_cb.mac_addr));
                // }
                // break;
            }

            // default case to handle unexpected event types
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }
}

// function that initialises ESPNOW in general and our task above
static esp_err_t my_espnow_init(void) {
    // declare a pointer for send parameters
    espnow_send_param_t* send_param;

    // create queue that was defined globally, then check if queue creation was successful
    s_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (s_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create queue fail");
        return ESP_FAIL;
    }

    //initialise ESPNOW and sending and receiving callback functions
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    // configure this through menuconfig and given Kconfig file
    #if CONFIG_ESPNOW_ENABLE_POWER_SAVE
        ESP_ERROR_CHECK(esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW));
        ESP_ERROR_CHECK(esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL));
    #endif

    // set the primary master key (PMK) for ESPNOW (change through menuconfig)
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK));

    // initialize peer information
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(s_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t)); // initialize peer
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF; //interface
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer)); 
    free(peer); // now that we have added the peer information to ESPNOW peer list we do not need "peer"

    // initialize sending parameters
    send_param = malloc(sizeof(espnow_send_param_t)); // note that malloc returns a pointer to the allocated space
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(s_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(espnow_send_param_t)); // intialize send_param
    send_param->unicast = false; // set to broadcast mode
    send_param->broadcast = true;
    send_param->state = 0;
    send_param->magic = esp_random();
    // use menuconfig for the following
    send_param->count = CONFIG_ESPNOW_SEND_COUNT; // number of packets to send
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = CONFIG_ESPNOW_SEND_LEN;
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(s_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    // set mac address for receiver 
    memcpy(send_param->dest_mac, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    espnow_data_prepare(send_param);

    // create ESPNOW task
    if (xTaskCreate(my_espnow_task, "espnow task", 2048, send_param, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void my_espnow_deinit(espnow_send_param_t* send_param) {
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(s_espnow_queue);
    esp_now_deinit();
}

void app_main(void)
{
    // initialize NVS (non volatile storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret); // if return (ret) is not ESP_OK the esp32 will reboot

    my_wifi_init();
    my_espnow_init();

    // #if CONFIG_RECEIVER
	//     xTaskCreate(&receiver, "RECEIVER", 1024*3, NULL, 2, NULL);
    // #endif

    // #if CONFIG_SENDER
	//     xTaskCreate(&sender, "SENDER", 1024*3, NULL, 2, NULL);
    // #endif
}